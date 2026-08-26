"""Shift header stamps that drifted away from the bag receive time.

Legacy fix, ported from fix_lidar_stamps.py - only very old recordings need it,
so this transform is disabled unless the config says otherwise.

modes:
    auto          estimate a constant offset = median(receive - header) over the
                  selected messages in a first pass, then add it to every stamp
    fixed         add `offset_ns` from the config
    receive_time  overwrite the header stamp with the bag receive timestamp
"""

from __future__ import annotations

import statistics

from . import Transform, register

NS = 1_000_000_000


@register("stamp_offset")
class StampOffset(Transform):
    """Options: topics, mode (auto|fixed|receive_time), offset_ns, enabled."""

    def configure(self) -> None:
        self.mode = self.options.get("mode", "auto")
        if self.mode not in ("auto", "fixed", "receive_time"):
            raise ValueError(
                f"stamp_offset: mode must be auto|fixed|receive_time, got {self.mode!r}"
            )
        if self.enabled and not self.patterns:
            raise ValueError("stamp_offset transform needs a 'topics' option")
        self.offset_ns = int(self.options.get("offset_ns") or 0)
        if self.mode == "fixed" and not self.offset_ns:
            raise ValueError("stamp_offset: mode 'fixed' needs a non-zero 'offset_ns'")
        self.needs_prepare = self.mode == "auto"
        self._diffs: list[int] = []

    def prepare(self, topic, msg, stamp):
        header = getattr(msg, "header", None)
        if header is None:
            return
        self._diffs.append(stamp - _stamp_ns(header))

    def prepare_done(self) -> None:
        if not self.needs_prepare:
            return
        if not self._diffs:
            self.note("no messages matched, offset left at 0")
            return
        self.offset_ns = int(round(statistics.median(self._diffs)))
        self.note(f"estimated offset {self.offset_ns} ns (~{self.offset_ns / NS:.3f} s)")

    def apply(self, topic, msg, stamp):
        header = getattr(msg, "header", None)
        if header is None:
            self.note(f"{topic}: message has no header, skipped")
            return msg

        before = _stamp_ns(header)
        after = stamp if self.mode == "receive_time" else before + self.offset_ns
        header.stamp.sec, header.stamp.nanosec = divmod(after, NS)

        self.record_sample(f"stamp={before / NS:.6f}", f"stamp={after / NS:.6f}")
        self.count += 1
        return msg

    def describe(self, msg) -> str:
        return f"stamp={_stamp_ns(msg.header) / NS:.6f}"


def _stamp_ns(header) -> int:
    return header.stamp.sec * NS + header.stamp.nanosec
