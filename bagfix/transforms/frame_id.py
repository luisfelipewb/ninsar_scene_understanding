"""Rewrite header.frame_id on the selected topics."""

from __future__ import annotations

from . import Transform, register


@register("frame_id")
class FrameId(Transform):
    """Set `header.frame_id` to a fixed value.

    Options:
        topics:   list of topic name globs
        frame_id: the frame to write
    """

    def configure(self) -> None:
        self.frame_id = self.options.get("frame_id")
        if not self.frame_id:
            raise ValueError("frame_id transform needs a 'frame_id' option")
        if not self.patterns:
            raise ValueError("frame_id transform needs a 'topics' option")

    def apply(self, topic, msg, stamp):
        header = getattr(msg, "header", None)
        if header is None:
            self.note(f"{topic}: message has no header, skipped")
            return msg
        self.record_sample(f"frame_id={header.frame_id!r}", f"frame_id={self.frame_id!r}")
        header.frame_id = self.frame_id
        self.count += 1
        return msg
