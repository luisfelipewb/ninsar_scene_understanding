"""Command line entry point: bagfix run / inspect / list-transforms."""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

from . import bagio, config as config_mod, convert_ros1, pipeline
from .transforms import REGISTRY

def _workspace_root() -> Path:
    """The colcon workspace root, where relative output paths are resolved.

    Found by walking up from this file to the directory holding `src/`, so it
    works both from the source tree and from an installed package. Falls back to
    the current directory when neither applies.
    """
    for directory in Path(__file__).resolve().parents:
        if (directory / "src").is_dir() and directory.name != "src":
            return directory
    return Path.cwd()


WORKSPACE = _workspace_root()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="bagfix", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="apply a config's pipeline to a bag")
    run.add_argument("config", help="robot config YAML (e.g. src/ninsar_bag_fix/config/pom_4x4.yaml)")
    run.add_argument("bag", help="source bag directory (or .bag file for ROS 1)")
    run.add_argument("-o", "--out", help="output bag directory (default: from config)")
    run.add_argument("--dry-run", action="store_true",
                     help="report what would change without writing a bag")
    run.add_argument("--force", action="store_true",
                     help="overwrite the output directory if it exists")
    run.add_argument("--set", dest="sets", action="append", default=[],
                     metavar="RULE.OPTION=VALUE", help="override a config option")
    run.set_defaults(func=cmd_run)

    inspect = sub.add_parser("inspect", help="summarise a bag's topics and headers")
    inspect.add_argument("bag")
    inspect.add_argument("--no-headers", action="store_true",
                         help="skip reading frame_ids (much faster on big bags)")
    inspect.set_defaults(func=cmd_inspect)

    listing = sub.add_parser("list-transforms", help="show the available transforms")
    listing.set_defaults(func=cmd_list)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (config_mod.ConfigError, convert_ros1.ConversionError, ValueError,
            FileExistsError, FileNotFoundError) as exc:
        # Flush first: stdout is block-buffered when piped, so without this the
        # error would surface above the context lines that led to it.
        sys.stdout.flush()
        print(f"error: {exc}", file=sys.stderr)
        return 1


# ---- commands -------------------------------------------------------------


def cmd_run(args) -> int:
    source = Path(args.bag)
    # the recording is identified by the folder above `bag/`, so the name is
    # taken from the original argument and survives the ROS 1 conversion
    session = bagio.session_name(source)
    cfg = config_mod.load(args.config, bag_name=session, sets=args.sets)

    root = Path(cfg.output_root)
    if not root.is_absolute():
        root = WORKSPACE / root

    ros1 = convert_ros1.find_ros1_bag(source)
    if ros1 is not None:
        if not cfg.convert:
            raise ValueError(
                f"{ros1} is a ROS 1 bag but {args.config} has no 'convert' section"
            )
        source = convert_ros1.convert(
            ros1,
            convert_ros1.converted_path(session, root),
            cfg.output_storage,
            cache=cfg.convert.get("cache", True),
        )

    source = bagio.resolve_bag(source)
    out = Path(args.out) if args.out else pipeline.default_output(session, cfg, root)

    print(f"config : {cfg.path} (robot {cfg.robot})")
    print(f"session: {session}")
    print(f"source : {source}")
    print(f"output : {'(dry run, nothing written)' if args.dry_run else out}")

    result = pipeline.run(source, out, cfg, dry_run=args.dry_run, force=args.force)
    return report(result, dry_run=args.dry_run)


def cmd_inspect(args) -> int:
    bag = bagio.resolve_bag(args.bag)
    reader = bagio.open_reader(bag)
    types = bagio.topic_types(reader)
    counts: Counter[str] = Counter()
    frames: dict[str, set[str]] = {t: set() for t in types}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        counts[topic] += 1
        if not args.no_headers and len(frames[topic]) < 4:
            frame = _peek_frame_id(data, types[topic])
            if frame is not None:
                frames[topic].add(frame)

    print(f"{bag}  (session {bagio.session_name(bag)}, "
          f"{bagio.detect_storage_id(bag)}, "
          f"{len(types)} topics, {sum(counts.values())} messages)\n")
    width = max((len(t) for t in types), default=10)
    for topic in sorted(types):
        line = f"  {topic:<{width}}  {counts[topic]:>8}  {types[topic]}"
        if frames[topic]:
            line += f"  frame_id={sorted(frames[topic])}"
        print(line)
    return 0


def cmd_list(args) -> int:
    for name in sorted(REGISTRY):
        doc = (REGISTRY[name].__doc__ or "").strip()
        print(f"\n{name}\n" + "\n".join(f"  {ln}" for ln in doc.splitlines()))
    return 0


# ---- reporting ------------------------------------------------------------


def report(result: pipeline.Result, dry_run: bool) -> int:
    print(f"\n{result.total_messages} messages read, "
          f"{result.deserialized} deserialized, "
          f"{result.dropped} dropped, {result.emitted} added")
    print(f"topics: {result.topics_in} in -> {result.topics_out} out\n")

    warnings = 0
    for transform in result.transforms:
        print(f"  [{transform.rule_id}] {transform.count} change(s)")
        if transform.sample:
            before, after = transform.sample
            print(f"      before: {before}")
            print(f"      after : {after}")
        for note in transform.notes:
            print(f"      note  : {note}")
        if transform.count == 0:
            warnings += 1
            print("      WARNING: this rule changed nothing - check its topics")

    if dry_run:
        print("\nDry run: no bag written.")
    else:
        print(f"\nDone -> {result.output}")
    return 2 if warnings else 0


def _peek_frame_id(data: bytes, type_name: str):
    """Best-effort frame_id read; returns None for messages without a header."""
    import rclpy.serialization as ser

    try:
        msg = ser.deserialize_message(data, bagio.message_class(type_name))
    except Exception:
        return None
    header = getattr(msg, "header", None)
    return header.frame_id if header is not None else None


if __name__ == "__main__":
    raise SystemExit(main())
