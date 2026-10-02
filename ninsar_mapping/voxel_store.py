"""
Pure numpy side of the voxel query node (no ROS graph needed; covered by test/test_voxel_store.py).

The Voxeland server publishes its map on bonxai_point_cloud_centers as XYZRGB voxel centres. In
the dense RGBSemantics mode we run, the colour of a voxel is the palette colour of its most likely
class, or the camera RGB when the most likely class is the last one ("unknown"):
RGBSemantics::toColor and SemanticMap::indexToHexColor in voxeland_map. So the snapshot gives the
top class of every voxel, and full distributions are fetched only for the voxels a query selects.

Boxes follow spark_dsg's BoundingBox (dimensions, world_P_center, world_R_center): AABB is R = I,
the RAABB of scripts/make_isaac_farm_dsg.py is a yaw rotation, and any rotation (OBB) works.
"""

import re

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

# SemanticMap::color_palette, voxeland_map/src/semantics.cpp:6 (0xRRGGBB)
VOXELAND_PALETTE = (
    0xFAD4E0, 0x9DBBE3, 0xBFE3DF, 0xB59CD9, 0xFFF5CC, 0xFFD9BD, 0xEE9D94, 0xF7ADCF,
    0xE6194B, 0x3CB44B, 0xFFE119, 0x4363D8, 0xF58231, 0x911EB4, 0x42D4F4, 0xF032E6,
    0xBFEF45, 0xFABED4, 0x469990, 0xDCBEFF, 0x9A6324, 0xFFFAC8,
)
INDEX0_GREY = 0xBCBCBC  # indexToHexColor(0)


# --------------------------------------------------------------------------------------
# Colours
# --------------------------------------------------------------------------------------

def class_colors(n_classes):
    """0xRRGGBB of classes 0 .. n_classes - 2 (the last class, "unknown", shows the camera RGB)."""
    return np.array([INDEX0_GREY if i == 0 else VOXELAND_PALETTE[i % len(VOXELAND_PALETTE)]
                     for i in range(n_classes - 1)], dtype=np.uint32)


def unpack_rgb(rgb):
    """PCL-packed rgb (float32 or uint32) -> 0xRRGGBB as uint32."""
    return np.ascontiguousarray(rgb).view(np.uint32) & np.uint32(0xFFFFFF)


def pack_rgb(rgb_u32):
    """0xRRGGBB -> PCL-packed float32 rgb (alpha 255)."""
    return (np.asarray(rgb_u32, dtype=np.uint32) | np.uint32(0xFF000000)).view(np.float32)


def decode_top_class(rgb_u32, n_classes):
    """Most likely class index of each voxel from its colour; anything else -> n_classes - 1.

    A camera colour equal to a class colour reads as that class. Rare, and the fetched
    distribution of a selected voxel is exact anyway.
    """
    top = np.full(rgb_u32.shape, n_classes - 1, dtype=np.int16)
    # reversed: if the palette wraps (more classes than colours), the lower index wins
    for i, color in reversed(list(enumerate(class_colors(n_classes)))):
        top[rgb_u32 == color] = i
    return top


# --------------------------------------------------------------------------------------
# Boxes
# --------------------------------------------------------------------------------------

def yaw_matrix(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def matrix_to_quaternion(R):
    """Rotation matrix -> (x, y, z, w)."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


class OrientedBox:
    """Box with centre, full side lengths and orientation R (box axes in world)."""

    def __init__(self, center, dimensions, rotation=None):
        self.center = np.asarray(center, dtype=np.float64).reshape(3)
        self.dimensions = np.asarray(dimensions, dtype=np.float64).reshape(3)
        self.rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=np.float64)

    @classmethod
    def from_yaw(cls, center, dimensions, yaw=0.0):
        return cls(center, dimensions, yaw_matrix(yaw))

    def corners(self):
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        return self.center + (signs * self.dimensions / 2) @ self.rotation.T

    def contains(self, xyz):
        """Mask of points inside (boundary included)."""
        xyz = np.asarray(xyz)
        corners = self.corners()
        lo, hi = corners.min(axis=0), corners.max(axis=0)
        mask = np.all((xyz >= lo) & (xyz <= hi), axis=1)  # world bounds first: cheap prefilter
        idx = np.flatnonzero(mask)
        local = (xyz[idx] - self.center) @ self.rotation  # R^T (p - c), row-wise
        mask[idx] = np.all(np.abs(local) <= self.dimensions / 2 + 1e-6, axis=1)
        return mask


# --------------------------------------------------------------------------------------
# Snapshot
# --------------------------------------------------------------------------------------

class VoxelStore:
    """Occupied voxel centres and their top class, from one bonxai_point_cloud_centers message."""

    def __init__(self, xyz, top_class, header=None):
        self.xyz = xyz
        self.top_class = top_class
        self.header = header

    def __len__(self):
        return self.xyz.shape[0]

    def select(self, box=None, class_ids=()):
        """Indices of voxels inside box (None = whole map) whose top class is in class_ids (empty = any)."""
        mask = np.ones(len(self), dtype=bool) if box is None else box.contains(self.xyz)
        if len(class_ids):
            mask &= np.isin(self.top_class, np.asarray(class_ids, dtype=self.top_class.dtype))
        return np.flatnonzero(mask)


def cloud2_to_arrays(msg):
    """PointCloud2 with x, y, z, rgb -> (xyz float32 (M, 3), 0xRRGGBB uint32 (M,))."""
    n = msg.width * msg.height
    pts = np.frombuffer(msg.data, dtype=point_cloud2.dtype_from_fields(msg.fields, msg.point_step),
                        count=n)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1).astype(np.float32)
    return xyz, unpack_rgb(pts["rgb"].astype(np.float32))


def store_from_cloud(msg, n_classes):
    xyz, rgb = cloud2_to_arrays(msg)
    return VoxelStore(xyz, decode_top_class(rgb, n_classes), msg.header)


# --------------------------------------------------------------------------------------
# Distributions and result
# --------------------------------------------------------------------------------------

def distributions_to_array(distributions, class_names):
    """voxeland_msgs/ClassDistribution[] -> (N, C) float32, columns in class_names order."""
    probs = np.empty((len(distributions), len(class_names)), dtype=np.float32)
    for i, dist in enumerate(distributions):
        hyps = dist.probabilities
        if [h.class_id for h in hyps] != class_names:
            raise ValueError(f"classes {[h.class_id for h in hyps]} differ from {class_names}")
        probs[i] = [h.score for h in hyps]
    return probs


def keep_by_prob(probs, class_ids, min_prob):
    """Mask of rows whose max probability over class_ids (empty = all classes) is >= min_prob."""
    cols = probs[:, list(class_ids)] if len(class_ids) else probs
    if cols.shape[0] == 0 or cols.shape[1] == 0:
        return np.ones(cols.shape[0], dtype=bool)
    return cols.max(axis=1) >= min_prob


def prob_field_names(class_names):
    """Point field name per class: p_<name>, with anything but [A-Za-z0-9_] turned into _."""
    return ["p_" + re.sub(r"\W", "_", name) for name in class_names]


def build_result_cloud(header, xyz, rgb_u32, probs, class_names):
    """PointCloud2 with x, y, z, rgb and one float32 p_<class> field per class."""
    names = ["x", "y", "z", "rgb"] + prob_field_names(class_names)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(names)]
    out = np.empty(xyz.shape[0], dtype=np.dtype([(n, "<f4") for n in names]))
    out["x"], out["y"], out["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    out["rgb"] = pack_rgb(rgb_u32)
    for k, n in enumerate(names[4:]):
        out[n] = probs[:, k]
    cloud = point_cloud2.create_cloud(header, fields, out)
    cloud.is_dense = True
    return cloud
