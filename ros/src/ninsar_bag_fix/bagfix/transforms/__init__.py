"""Transform registry.

A transform edits messages of the topics it selects. The pipeline deserializes a
message once, hands it to every transform that selected its topic (in config
order), and reserializes once. Topics no transform selects are copied raw.
"""

from __future__ import annotations

import fnmatch
from typing import Any, Iterable

REGISTRY: dict[str, type["Transform"]] = {}


def register(name: str):
    def decorator(cls):
        cls.name = name
        REGISTRY[name] = cls
        return cls

    return decorator


class UnknownTransform(Exception):
    pass


def build(transform: str, rule_id: str, options: dict[str, Any]) -> "Transform":
    cls = REGISTRY.get(transform)
    if cls is None:
        raise UnknownTransform(
            f"unknown transform '{transform}'; available: {', '.join(sorted(REGISTRY))}"
        )
    return cls(rule_id, options)


class Transform:
    """Base class. Subclasses override `apply` and/or the topic-level hooks."""

    name = "transform"
    #: transforms that need a first pass over the bag before rewriting
    needs_prepare = False
    #: message types this transform can handle; empty means "any"
    accepts: tuple[str, ...] = ()

    def __init__(self, rule_id: str, options: dict[str, Any]):
        self.rule_id = rule_id or self.name
        self.options = options
        self.enabled = bool(options.get("enabled", True))
        self.patterns = _as_list(options.get("topics"))
        self.count = 0
        self.sample: tuple[str, str] | None = None  # (before, after) for reporting
        self.notes: list[str] = []
        self.configure()

    # ---- hooks ------------------------------------------------------------

    def configure(self) -> None:
        """Validate and normalise `self.options`. Raise ValueError on bad input."""

    def selects(self, topic: str, type_name: str) -> bool:
        """True if this transform wants to see messages of `topic`."""
        if not self.patterns:
            return False
        if self.accepts and type_name not in self.accepts:
            return False
        return any(fnmatch.fnmatchcase(topic, p) for p in self.patterns)

    def preflight(self, source, types: dict[str, str]) -> None:
        """Check this rule against the bag before any output is written.

        Called once, before the output directory is created, so raising here
        costs nothing and leaves no half-written bag behind. `source` is the
        resolved bag directory; open a filtered reader on it (see
        `bagio.open_reader(topics=...)`) if the check needs message content.

        Raise ValueError to abort the run.
        """

    def prepare(self, topic: str, msg: Any, stamp: int) -> None:
        """Called for every selected message during the pre-pass, if needed."""

    def prepare_done(self) -> None:
        """Called once after the pre-pass finishes."""

    def apply(self, topic: str, msg: Any, stamp: int) -> Any | None:
        """Return the edited message, or None to drop it from the output."""
        return msg

    def finish(self) -> None:
        """Called once after the rewrite pass, before reporting."""

    def plan_topics(self, topics: dict[str, Any]) -> dict[str, Any]:
        """Rewrite the writer's topic table (drop/rename/add). Default: unchanged."""
        return topics

    def route(self, topic: str) -> str | None:
        """Map a source topic to its output name; None drops it. Default: unchanged.

        Called once per topic (not per message), after `plan_topics`.
        """
        return topic

    def emit(self, ctx: "EmitContext") -> Iterable[tuple[str, Any, int]]:
        """Extra (topic, message, stamp) tuples to append to the output bag."""
        return ()

    # ---- reporting --------------------------------------------------------

    def describe(self, msg: Any) -> str:
        """Short one-line rendering of a message, used for dry-run samples."""
        return repr(msg)[:200]

    def record_sample(self, before: str, after: str) -> None:
        if self.sample is None:
            self.sample = (before, after)

    def note(self, text: str) -> None:
        """Record a warning once, however many messages trigger it."""
        if text not in self.notes:
            self.notes.append(text)


class EmitContext:
    """Bag-level facts a transform may need when synthesizing messages."""

    def __init__(self, start_time: int, end_time: int, existing_topics: set[str]):
        self.start_time = start_time
        self.end_time = end_time
        self.existing_topics = existing_topics


def _as_list(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):  # legacy {topic: type} form from the old scripts
        return list(value)
    return list(value)


# Import the concrete transforms so importing this package populates REGISTRY.
from . import camera_info, frame_id, stamp_offset, tf_static, topic_filter  # noqa: E402,F401
