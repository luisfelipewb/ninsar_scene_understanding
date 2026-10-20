"""Checks the numpy side of the voxel query node: colour decoding, boxes, selection, clouds."""

import numpy as np
import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from vision_msgs.msg import ObjectHypothesis
from voxeland_msgs.msg import ClassDistribution

from ninsar_mapping.voxel_store import (
    INDEX0_GREY, VOXELAND_PALETTE, OrientedBox, VoxelStore, build_result_cloud, class_colors,
    cloud2_to_arrays, decode_top_class, distributions_to_array, keep_by_prob, matrix_to_quaternion,
    pack_rgb, prob_field_names, store_from_cloud, unpack_rgb, yaw_matrix,
)

# ninsar_bringup config/isaac/semantic_cloud_adapter.yaml categories as the server sees them ("unknown" last)
CLASSES = ["ground", "tree", "grapevine", "lemon", "cabbage", "fence", "building", "obstacle", "unknown"]
N = len(CLASSES)


def test_class_colors_follow_voxeland():
    colors = class_colors(N)
    assert colors.shape == (N - 1,)  # unknown shows the camera RGB, no class colour
    assert colors[0] == INDEX0_GREY
    assert list(colors[1:]) == list(VOXELAND_PALETTE[1:N - 1])


def test_rgb_pack_round_trip():
    rgb = np.array([0x000000, 0xBCBCBC, 0xE6194B, 0xFFFFFF], dtype=np.uint32)
    packed = pack_rgb(rgb)
    assert packed.dtype == np.float32
    assert np.array_equal(unpack_rgb(packed), rgb)


def test_decode_top_class():
    colors = class_colors(N)
    rgb = np.array([colors[0], colors[2], colors[N - 2], 0x123456, 0xBCBCBD], dtype=np.uint32)
    assert decode_top_class(rgb, N).tolist() == [0, 2, N - 2, N - 1, N - 1]


def test_decode_lower_index_wins_when_palette_wraps():
    n = len(VOXELAND_PALETTE) + 3  # class len(palette) + 1 reuses palette[1]
    rgb = np.array([VOXELAND_PALETTE[1]], dtype=np.uint32)
    assert decode_top_class(rgb, n).tolist() == [1]


def test_axis_aligned_box():
    box = OrientedBox([1.0, 2.0, 3.0], [2.0, 4.0, 6.0])
    pts = np.array([[1, 2, 3], [2, 4, 6], [0, 0, 0], [2.01, 2, 3], [1, 2, 6.01]], dtype=np.float32)
    assert box.contains(pts).tolist() == [True, True, True, False, False]


def test_yaw_box():
    # 4 x 1 x 1 box along the x axis, turned 45 degrees: its long axis is the x = y diagonal
    box = OrientedBox.from_yaw([0.0, 0.0, 0.0], [4.0, 1.0, 1.0], np.pi / 4)
    d = np.sqrt(0.5)
    pts = np.array([
        [0.0, 0.0, 0.0],      # centre
        [1.9 * d, 1.9 * d, 0.0],  # along the long axis, inside
        [1.9, 0.0, 0.0],      # inside the unrotated box, outside the rotated one
        [-0.4 * d, 0.4 * d, 0.0],  # across, inside
        [-0.6 * d, 0.6 * d, 0.0],  # across, outside
    ])
    assert box.contains(pts).tolist() == [True, True, False, True, False]
    corners = box.corners()
    assert box.contains(corners).all()
    assert not box.contains(corners * 1.01).any()


def test_box_rotation_quaternion():
    for yaw in (0.0, 0.3, -2.0, np.pi):
        x, y, z, w = matrix_to_quaternion(yaw_matrix(yaw))
        assert np.allclose(np.abs([x, y, z, w]), np.abs([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)]))


def test_box_matches_spark_dsg():
    dsg = pytest.importorskip("spark_dsg")
    center, dims, yaw = np.float32([1.0, -2.0, 0.5]), np.float32([3.0, 1.0, 2.0]), 0.7
    ref = dsg.BoundingBox(dims, center, yaw)
    pts = np.random.default_rng(0).uniform(-3, 4, size=(500, 3)).astype(np.float32)
    ours = OrientedBox.from_yaw(center, dims, yaw).contains(pts)
    assert ours.tolist() == [bool(ref.contains(p)) for p in pts]


def store():
    xyz = np.array([[0, 0, 0], [1, 0, 0], [5, 5, 0], [0.5, 0.5, 0.5]], dtype=np.float32)
    return VoxelStore(xyz, np.array([0, 2, 2, 8], dtype=np.int16))


def test_select():
    s = store()
    box = OrientedBox([0.5, 0.0, 0.0], [2.0, 2.0, 2.0])
    assert s.select().tolist() == [0, 1, 2, 3]
    assert s.select(class_ids=[2]).tolist() == [1, 2]
    assert s.select(box).tolist() == [0, 1, 3]
    assert s.select(box, [2, 8]).tolist() == [1, 3]


def test_cloud_round_trip():
    s = store()
    probs = np.random.default_rng(1).random((len(s), N)).astype(np.float32)
    rgb = class_colors(N)[[0, 2, 2, 3]]
    cloud = build_result_cloud(Header(frame_id="world"), s.xyz, rgb, probs, CLASSES)
    assert [f.name for f in cloud.fields] == ["x", "y", "z", "rgb"] + prob_field_names(CLASSES)
    xyz, rgb_back = cloud2_to_arrays(cloud)
    assert np.array_equal(xyz, s.xyz)
    assert np.array_equal(rgb_back, rgb)
    pts = point_cloud2.read_points(cloud, field_names=["p_grapevine"])
    assert np.allclose(pts["p_grapevine"], probs[:, 2])
    assert store_from_cloud(cloud, N).top_class.tolist() == [0, 2, 2, 3]


def test_prob_field_names_sanitized():
    assert prob_field_names(["ground", "fruit tree", "a-b"]) == ["p_ground", "p_fruit_tree", "p_a_b"]


def distribution(scores, names=CLASSES):
    return ClassDistribution(probabilities=[ObjectHypothesis(class_id=n, score=float(s))
                                            for n, s in zip(names, scores)])


def test_distributions_to_array():
    rows = np.eye(N)[:3]
    probs = distributions_to_array([distribution(r) for r in rows], CLASSES)
    assert probs.shape == (3, N) and np.allclose(probs, rows)
    with pytest.raises(ValueError):
        distributions_to_array([distribution(rows[0], CLASSES[::-1])], CLASSES)


def test_keep_by_prob():
    probs = np.array([[0.7, 0.2, 0.1], [0.1, 0.3, 0.6]], dtype=np.float32)
    assert keep_by_prob(probs, [1], 0.25).tolist() == [False, True]
    assert keep_by_prob(probs, [], 0.65).tolist() == [True, False]
    assert keep_by_prob(probs, [0, 1], 0.0).tolist() == [True, True]
    assert keep_by_prob(probs[:0], [1], 0.5).tolist() == []
