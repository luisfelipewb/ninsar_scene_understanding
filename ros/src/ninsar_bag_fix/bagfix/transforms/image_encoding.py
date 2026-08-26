"""Relabel the `encoding` field of sensor_msgs/Image.

Written for the Visionary-B depth stream, which is recorded as `mono16` while
every consumer of a depth image expects `16UC1`. `depth_image_proc` rejects the
frames outright:

    [ERROR] [PointCloudXyzrgbNode]: Depth image has unsupported encoding [mono16]

`mono16` and `16UC1` describe the *same* buffer - one channel, 16-bit unsigned -
so nothing about the pixels changes here, only the label.

What does change is the **meaning**. `mono16` carries no unit; `16UC1` is
defined by the depth pipeline to be **millimetres** (as `32FC1` is metres).
Relabelling therefore asserts a unit that the bytes cannot prove. Get it wrong
and every point comes out scaled by a constant, silently. Confirm the unit
against something external - a known distance, a calibrated lidar - before
trusting the output.

To keep the harmless half honest, `preflight` verifies the buffer really is
laid out the way the target encoding requires (`step == width * bytes/pixel`,
`len(data) == height * step`, and the same bytes/pixel on both sides) and
refuses the run otherwise, before any output is written.
"""

from __future__ import annotations

import re

import rclpy.serialization as ser

from ..bagio import message_class, open_reader
from . import Transform, register

IMAGE_TYPE = "sensor_msgs/msg/Image"

#: bytes per pixel for the named encodings in sensor_msgs/image_encodings.hpp
_NAMED_BYTES = {
    "mono8": 1, "mono16": 2,
    "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4,
    "rgb16": 6, "bgr16": 6, "rgba16": 8, "bgra16": 8,
    "yuv422": 2, "yuv422_yuy2": 2, "uyvy": 2, "yuyv": 2,
}

#: the generic OpenCV-style form, e.g. 16UC1, 32FC3
_TYPE_RE = re.compile(r"^(8U|8S|16U|16S|32S|32F|64F)C(\d+)$")
_DEPTH_BYTES = {"8U": 1, "8S": 1, "16U": 2, "16S": 2, "32S": 4, "32F": 4, "64F": 8}


def bytes_per_pixel(encoding: str) -> int | None:
    """Bytes per pixel of an image encoding, or None if it is not recognised."""
    if encoding in _NAMED_BYTES:
        return _NAMED_BYTES[encoding]
    match = _TYPE_RE.match(encoding)
    if match:
        return _DEPTH_BYTES[match.group(1)] * int(match.group(2))
    if encoding.startswith("bayer_"):
        return 2 if encoding.endswith("16") else 1
    return None


@register("image_encoding")
class ImageEncoding(Transform):
    """Rewrite `Image.encoding`, leaving the pixel buffer untouched.

    Options:
        topics: list of topic name globs
        to:     the encoding to write (required)
        from:   only rewrite messages currently in this encoding (optional, but
                recommended - it is what makes a re-run a no-op rather than a
                second, wrong relabel)
    """

    accepts = (IMAGE_TYPE,)

    def configure(self) -> None:
        if not self.patterns:
            raise ValueError("image_encoding transform needs a 'topics' option")
        self.to_encoding = self.options.get("to")
        if not self.to_encoding:
            raise ValueError("image_encoding transform needs a 'to' option")
        self.from_encoding = self.options.get("from")

        self.target_bytes = bytes_per_pixel(self.to_encoding)
        if self.target_bytes is None:
            raise ValueError(
                f"'to: {self.to_encoding}' is not a known image encoding "
                f"(expected one of {', '.join(sorted(_NAMED_BYTES))}, or a form "
                f"like 16UC1)"
            )
        if self.from_encoding:
            source_bytes = bytes_per_pixel(self.from_encoding)
            if source_bytes is None:
                raise ValueError(
                    f"'from: {self.from_encoding}' is not a known image encoding"
                )
            if source_bytes != self.target_bytes:
                raise ValueError(
                    f"'{self.from_encoding}' is {source_bytes} bytes/pixel and "
                    f"'{self.to_encoding}' is {self.target_bytes}: relabelling "
                    "cannot reinterpret a buffer, it only renames it"
                )

    # ---- preflight --------------------------------------------------------

    def preflight(self, source, types) -> None:
        """Check the first frame of each selected topic before anything is written.

        Reads only the selected topics, so this is an index lookup rather than a
        pass over the bag.
        """
        targets = [t for t, type_name in types.items() if self.selects(t, type_name)]
        if not targets:
            return

        reader = open_reader(source, topics=targets)
        cls = message_class(IMAGE_TYPE)
        seen: dict[str, object] = {}
        while reader.has_next() and len(seen) < len(targets):
            topic, data, _ = reader.read_next()
            if topic not in seen:
                seen[topic] = ser.deserialize_message(data, cls)

        for topic, msg in seen.items():
            self._check(topic, msg)

    def _check(self, topic: str, msg) -> None:
        if msg.encoding == self.to_encoding:
            raise ValueError(
                f"{topic} is already '{self.to_encoding}'; drop this rule (or set "
                "'enabled: false') rather than relabelling it a second time"
            )
        if self.from_encoding and msg.encoding != self.from_encoding:
            raise ValueError(
                f"{topic} is '{msg.encoding}', not the '{self.from_encoding}' this "
                "rule expects; check the topic globs with 'bagfix inspect'"
            )

        source_bytes = bytes_per_pixel(msg.encoding)
        if source_bytes is None:
            raise ValueError(
                f"{topic} has unrecognised encoding '{msg.encoding}', so its "
                f"layout cannot be checked against '{self.to_encoding}'"
            )
        if source_bytes != self.target_bytes:
            raise ValueError(
                f"{topic} is '{msg.encoding}' ({source_bytes} bytes/pixel) and "
                f"'{self.to_encoding}' is {self.target_bytes}: relabelling would "
                "misread the buffer"
            )

        expected_step = msg.width * self.target_bytes
        if msg.step != expected_step:
            raise ValueError(
                f"{topic}: step is {msg.step}, but '{self.to_encoding}' at "
                f"width {msg.width} requires {expected_step}; this buffer is not "
                "laid out the way the target encoding expects"
            )
        if len(msg.data) != msg.height * msg.step:
            raise ValueError(
                f"{topic}: data is {len(msg.data)} bytes, expected "
                f"{msg.height * msg.step} (height {msg.height} x step {msg.step})"
            )
        if msg.is_bigendian:
            self.note(
                f"{topic}: is_bigendian is set; consumers that assume host byte "
                "order will misread this depth image"
            )

    # ---- rewrite ----------------------------------------------------------

    def apply(self, topic, msg, stamp):
        if self.from_encoding and msg.encoding != self.from_encoding:
            self.note(
                f"{topic}: encoding is '{msg.encoding}', not "
                f"'{self.from_encoding}', left unchanged"
            )
            return msg
        self.record_sample(self.describe(msg), f"encoding='{self.to_encoding}'")
        msg.encoding = self.to_encoding
        self.count += 1
        return msg

    def describe(self, msg) -> str:
        return f"encoding='{msg.encoding}'"
