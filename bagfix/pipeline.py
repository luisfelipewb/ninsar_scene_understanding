"""Run a config's transform pipeline over a bag in a single read/write pass."""

from __future__ import annotations

import fnmatch
from dataclasses import dataclass, field
from pathlib import Path

import rclpy.serialization as ser

from . import bagio
from .config import Config
from .transforms import EmitContext, Transform, build


@dataclass
class Result:
    source: Path
    output: Path | None
    total_messages: int = 0
    deserialized: int = 0
    dropped: int = 0
    emitted: int = 0
    transforms: list[Transform] = field(default_factory=list)
    topics_in: int = 0
    topics_out: int = 0


def build_transforms(config: Config) -> list[Transform]:
    """Instantiate the enabled rules of a config, in pipeline order."""
    transforms = []
    for rule in config.pipeline:
        if not rule.enabled:
            continue
        try:
            transforms.append(build(rule.transform, rule.key, rule.options))
        except ValueError as exc:
            raise ValueError(f"rule '{rule.key}': {exc}") from exc
    return transforms


def run(
    source: str | Path,
    output: str | Path | None,
    config: Config,
    dry_run: bool = False,
    force: bool = False,
) -> Result:
    """Rewrite `source` into `output` applying the config pipeline.

    With `dry_run` nothing is written; every rule still reports what it matched.
    """
    source = bagio.resolve_bag(source)
    transforms = build_transforms(config)

    reader = bagio.open_reader(source)
    source_meta = {t.name: t for t in reader.get_all_topics_and_types()}
    types = {name: meta.type for name, meta in source_meta.items()}

    # topic -> transforms that want its payload; everything else is copied raw
    claims: dict[str, list[Transform]] = {}
    for topic, type_name in types.items():
        selected = [t for t in transforms if t.selects(topic, type_name)]
        if selected:
            claims[topic] = selected

    _warn_unmatched_patterns(transforms, types)

    # optional first pass, only for transforms that need bag-wide statistics
    preparers = [t for t in transforms if getattr(t, "needs_prepare", False)]
    if preparers:
        _prepare_pass(reader, claims, types, preparers)
        reader = bagio.open_reader(source)
    for transform in transforms:
        transform.prepare_done()

    # ---- topic table ------------------------------------------------------
    planned = dict(source_meta)
    for transform in transforms:
        planned = transform.plan_topics(planned)

    routes: dict[str, str | None] = {}
    for topic in source_meta:
        target: str | None = topic
        for transform in transforms:
            if target is None:
                break
            target = transform.route(target)
        if target is not None and target not in planned:
            target = None  # a transform dropped the topic from the table
        routes[topic] = target

    result = Result(
        source=source,
        output=None if dry_run else Path(output),
        topics_in=len(source_meta),
        topics_out=len(planned),
        transforms=transforms,
    )

    writer = None
    if not dry_run:
        bagio.prepare_output_dir(output, force=force)
        split = config.output_split
        if split is None:  # keep the source's splitting behaviour by default
            split = bagio.source_split_size(source)
        writer = bagio.open_writer(output, config.output_storage, split)
        for meta in planned.values():
            writer.create_topic(meta)

    # ---- single rewrite pass ---------------------------------------------
    first_stamp = last_stamp = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        result.total_messages += 1
        first_stamp = stamp if first_stamp is None else first_stamp
        last_stamp = stamp

        target = routes.get(topic, topic)
        if target is None:
            result.dropped += 1
            continue

        selected = claims.get(topic)
        if selected:
            msg = ser.deserialize_message(data, bagio.message_class(types[topic]))
            result.deserialized += 1
            for transform in selected:
                msg = transform.apply(topic, msg, stamp)
                if msg is None:
                    break
            if msg is None:
                result.dropped += 1
                continue
            data = ser.serialize_message(msg)

        if writer is not None:
            writer.write(target, data, stamp)

    # ---- messages synthesized by transforms -------------------------------
    ctx = EmitContext(
        start_time=first_stamp or 0,
        end_time=last_stamp or 0,
        existing_topics=set(source_meta),
    )
    for transform in transforms:
        for topic, msg, stamp in transform.emit(ctx):
            result.emitted += 1
            if writer is not None:
                writer.write(topic, ser.serialize_message(msg), stamp)

    if writer is not None:
        writer.close()
    for transform in transforms:
        transform.finish()

    return result


def _prepare_pass(reader, claims, types, preparers) -> None:
    preparing = {t for t in preparers}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        selected = [t for t in claims.get(topic, ()) if t in preparing]
        if not selected:
            continue
        msg = ser.deserialize_message(data, bagio.message_class(types[topic]))
        for transform in selected:
            transform.prepare(topic, msg, stamp)


def _warn_unmatched_patterns(transforms, types) -> None:
    """Flag configured topic globs that match nothing - the usual config typo."""
    for transform in transforms:
        for pattern in transform.patterns:
            hits = [t for t in types if fnmatch.fnmatchcase(t, pattern)]
            if not hits:
                transform.note(f"topic pattern '{pattern}' matched no topic in the bag")
            elif transform.accepts and not any(types[t] in transform.accepts for t in hits):
                transform.note(
                    f"topic pattern '{pattern}' matched {hits} but none has a type "
                    f"this transform handles ({', '.join(transform.accepts)})"
                )


def default_output(session: str, config: Config, root: str | Path) -> Path:
    """<output.root>/<session><output.suffix>/bag - mirrors the input layout."""
    return Path(root) / f"{session}{config.output_suffix}" / bagio.BAG_SUBDIR
