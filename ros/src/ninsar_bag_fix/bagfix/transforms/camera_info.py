"""Repair broken CameraInfo calibration.

Ported from the original fix_camera_info.py:
  * K: focal lengths forced positive (sign bug); principal point kept, or
       defaulted to the image centre if it looks degenerate; fx/fy defaulted
       from a plausible FOV only if they are missing entirely.
  * R: set to the 3x3 identity (required for a monocular camera).
  * P: rebuilt from the corrected K with a zero translation column.
Each repair is individually switchable from the config.
"""

from __future__ import annotations

import math

from . import Transform, register

IDENTITY_3X3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


@register("camera_info")
class CameraInfoFix(Transform):
    """Options:

    topics:                 list of topic name globs
    force_positive_focal:   abs() fx/fy (default true)
    fix_principal_point:    reset cx/cy to image centre when out of frame (default true)
    identity_rectification: R := identity (default true)
    rebuild_p:              P := [K | 0] (default true)
    fallback_hfov_deg:      used only when fx/fy are still 0 (default 90.0)
    """

    accepts = ("sensor_msgs/msg/CameraInfo",)

    def configure(self) -> None:
        if not self.patterns:
            raise ValueError("camera_info transform needs a 'topics' option")
        self.force_positive_focal = self.options.get("force_positive_focal", True)
        self.fix_principal_point = self.options.get("fix_principal_point", True)
        self.identity_rectification = self.options.get("identity_rectification", True)
        self.rebuild_p = self.options.get("rebuild_p", True)
        self.fallback_hfov_deg = float(self.options.get("fallback_hfov_deg", 90.0))

    def apply(self, topic, msg, stamp):
        before = self.describe(msg)
        w, h = msg.width, msg.height
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]

        # --- focal lengths ---
        if self.force_positive_focal:
            fx, fy = abs(fx), abs(fy)
        # If still zero/missing, guess from a plausible horizontal FOV.
        if (fx == 0.0 or fy == 0.0) and w:
            guessed = (w / 2.0) / math.tan(math.radians(self.fallback_hfov_deg) / 2.0)
            fx = guessed if fx == 0.0 else fx
            fy = guessed if fy == 0.0 else fy
            self.note(f"{topic}: focal length missing, guessed from "
                      f"{self.fallback_hfov_deg} deg HFOV")

        # --- principal point ---
        # Keep the given value if it's inside the image, else default to centre.
        if self.fix_principal_point:
            if not (0.0 < cx < w):
                cx = w / 2.0
            if not (0.0 < cy < h):
                cy = h / 2.0

        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        if self.identity_rectification:
            msg.r = list(IDENTITY_3X3)
        if self.rebuild_p:
            msg.p = [fx, 0.0, cx, 0.0,
                     0.0, fy, cy, 0.0,
                     0.0, 0.0, 1.0, 0.0]

        self.record_sample(before, self.describe(msg))
        self.count += 1
        return msg

    def describe(self, msg) -> str:
        return f"K={[round(v, 3) for v in msg.k]} R={[round(v, 3) for v in msg.r]} " \
               f"P={[round(v, 3) for v in msg.p]}"
