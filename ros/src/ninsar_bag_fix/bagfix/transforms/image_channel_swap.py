"""Reorder the colour channels of sensor_msgs/Image, and relabel to match.

Written for the Visionary-B RGB stream, which is recorded as `bgr8` with the
bytes genuinely laid out B, G, R - so every consumer that trusts the header
renders red and blue the wrong way round.

This is the sibling of `image_encoding` and the *opposite* operation. That one
renames a buffer it never touches; this one rewrites the pixels and then renames
them to match. Which one you want depends on the bag:

- pixels already RGB, header wrongly says `bgr8` -> `image_encoding` (relabel),
- pixels really BGR, and you want RGB out -> this transform (swap).

Getting that backwards produces a bag that looks repaired and is not, so the
choice has to be made against the actual image - a known-coloured object in the
scene - not against the header.

Only the 8-bit colour encodings are accepted (`bgr8 <-> rgb8`,
`bgra8 <-> rgba8`), where the swap is exactly "exchange channel 0 and channel 2,
leave alpha alone". Packed formats like `yuv422` or a bayer pattern have no such
channel to exchange and are refused rather than mangled.

Unlike a relabel, this operation is its own inverse, so a second run would
silently undo the first. `from:` is therefore required, and `preflight` refuses
a topic that is already in the target encoding - before any output is written.
"""

from __future__ import annotations

import rclpy.serialization as ser

from ..bagio import message_class, open_reader
from . import Transform, register
from .image_encoding import IMAGE_TYPE, bytes_per_pixel

#: the swaps that are well defined: exchange channel 0 and 2, keep any alpha
_SWAPS = {
    ("bgr8", "rgb8"),
    ("rgb8", "bgr8"),
    ("bgra8", "rgba8"),
    ("rgba8", "bgra8"),
}


@register("image_channel_swap")
class ImageChannelSwap(Transform):
    """Swap the red and blue channels of an image, and relabel it to match.

    Options:
        topics: list of topic name globs
        from:   the encoding the bytes are currently in (required)
        to:     the encoding to produce (required)

    Supported pairs: bgr8 <-> rgb8, bgra8 <-> rgba8.
    """

    accepts = (IMAGE_TYPE,)

    def configure(self) -> None:
        if not self.patterns:
            raise ValueError("image_channel_swap transform needs a 'topics' option")
        self.from_encoding = self.options.get("from")
        self.to_encoding = self.options.get("to")
        if not self.from_encoding or not self.to_encoding:
            raise ValueError(
                "image_channel_swap transform needs both a 'from' and a 'to' "
                "option; the swap is its own inverse, so a run without 'from' "
                "could not tell a fix from an undo"
            )
        if (self.from_encoding, self.to_encoding) not in _SWAPS:
            supported = ", ".join(f"{a} -> {b}" for a, b in sorted(_SWAPS))
            raise ValueError(
                f"'{self.from_encoding} -> {self.to_encoding}' is not a channel "
                f"swap; supported: {supported}. To rename a buffer without "
                "touching it, use the image_encoding transform instead"
            )
        self.pixel_bytes = bytes_per_pixel(self.to_encoding)

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
                f"{topic} is already '{self.to_encoding}'; a channel swap is its "
                "own inverse, so running this rule again would undo it. Drop the "
                "rule (or set 'enabled: false')"
            )
        if msg.encoding != self.from_encoding:
            raise ValueError(
                f"{topic} is '{msg.encoding}', not the '{self.from_encoding}' this "
                "rule expects; check the topic globs with 'bagfix inspect'"
            )

        expected_step = msg.width * self.pixel_bytes
        if msg.step != expected_step:
            raise ValueError(
                f"{topic}: step is {msg.step}, but '{self.to_encoding}' at "
                f"width {msg.width} requires {expected_step}; this buffer is not "
                "laid out the way the encoding claims, so the channels cannot be "
                "picked out of it"
            )
        if len(msg.data) != msg.height * msg.step:
            raise ValueError(
                f"{topic}: data is {len(msg.data)} bytes, expected "
                f"{msg.height * msg.step} (height {msg.height} x step {msg.step})"
            )

    # ---- rewrite ----------------------------------------------------------

    def apply(self, topic, msg, stamp):
        if msg.encoding != self.from_encoding:
            self.note(
                f"{topic}: encoding is '{msg.encoding}', not "
                f"'{self.from_encoding}', left unchanged"
            )
            return msg

        before = self.describe(msg)
        buf = bytearray(msg.data)
        stride = self.pixel_bytes
        # the right-hand side is evaluated first, so this is a true swap; any
        # alpha byte at index 3 is left where it is
        buf[0::stride], buf[2::stride] = buf[2::stride], buf[0::stride]
        msg.data = bytes(buf)
        msg.encoding = self.to_encoding

        self.record_sample(before, self.describe(msg))
        self.count += 1
        return msg

    def describe(self, msg) -> str:
        head = ", ".join(str(b) for b in bytes(msg.data[: self.pixel_bytes]))
        return f"encoding='{msg.encoding}' first pixel=[{head}]"
