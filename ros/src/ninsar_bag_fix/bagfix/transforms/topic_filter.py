"""Drop or rename topics in the output bag.

This transform works at the topic table level, so the affected messages are
never deserialized - dropped topics cost nothing and renames are a raw copy.
"""

from __future__ import annotations

import fnmatch

from ..bagio import rename_topic_metadata
from . import Transform, register


@register("topic_filter")
class TopicFilter(Transform):
    """Options:

    drop:   list of topic name globs to remove from the output
    rename: mapping of old topic name -> new topic name (exact names)
    keep:   list of globs; when set, every topic NOT matching is dropped
    """

    def configure(self) -> None:
        self.drop = self.options.get("drop") or []
        self.keep = self.options.get("keep") or []
        self.rename = self.options.get("rename") or {}
        if not (self.drop or self.keep or self.rename):
            raise ValueError("topic_filter needs at least one of 'drop', 'keep', 'rename'")
        if isinstance(self.drop, str):
            self.drop = [self.drop]
        if isinstance(self.keep, str):
            self.keep = [self.keep]
        self.dropped: set[str] = set()
        self.renamed: dict[str, str] = {}

    def selects(self, topic, type_name):
        return False  # purely topic-level; never touches message payloads

    def plan_topics(self, topics):
        planned = {}
        for name, meta in topics.items():
            if self._is_dropped(name):
                self.dropped.add(name)
                continue
            new_name = self.rename.get(name)
            if new_name:
                if new_name in topics or new_name in planned:
                    raise ValueError(
                        f"topic_filter: cannot rename '{name}' to '{new_name}', "
                        "that topic already exists"
                    )
                self.renamed[name] = new_name
                planned[new_name] = rename_topic_metadata(meta, new_name)
            else:
                planned[name] = meta

        for name in self.drop:
            if not any(fnmatch.fnmatchcase(t, name) for t in topics):
                self.note(f"drop pattern '{name}' matched no topic")
        for name in self.rename:
            if name not in topics:
                self.note(f"rename source '{name}' is not in the bag")

        self.count = len(self.dropped) + len(self.renamed)
        self.record_sample(
            f"{len(topics)} topics",
            f"{len(planned)} topics "
            f"(dropped {sorted(self.dropped)}, renamed {self.renamed})",
        )
        return planned

    def _is_dropped(self, topic: str) -> bool:
        if any(fnmatch.fnmatchcase(topic, p) for p in self.drop):
            return True
        if self.keep and not any(fnmatch.fnmatchcase(topic, p) for p in self.keep):
            return True
        return False

    # ---- consulted by the pipeline for each message -----------------------

    def route(self, topic: str) -> str | None:
        """None means 'drop this message'; otherwise the output topic name."""
        if topic in self.dropped:
            return None
        return self.renamed.get(topic, topic)
