"""Checks the cloud_in contract (docs/CLOUD_IN_SPEC.md) on the adapter's encoding functions."""

import json

import numpy as np
import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from ninsar_mapping.semantic_cloud_adapter import (
    OUTPUT_DTYPE, OUTPUT_FIELDS, UNKNOWN, build_instances, check_table, confidence_table,
    encode_confidence, encode_ground_truth, ground_truth_table, parse_labels_json,
    select_points, server_categories,
)

CATEGORIES = server_categories(["ground", "tree", "lemon"])
# Isaac-style table: 0/1 are background ids, 7 is a class we do not map
NAMES = {0: "BACKGROUND", 1: "UNLABELLED", 2: "lemon", 3: "ground", 6: "tree", 7: "grapevine"}


def labels_image():
    rng = np.random.default_rng(0)
    img = rng.choice([0, 1, 2, 3, 6, 7, 42, -1], size=(6, 8)).astype(np.int32)  # 42: not in table
    img[0, 0] = 99999  # far outside the table
    return img


def assert_contract(table, ids, categories):
    check_table(table, categories)
    instances = build_instances(table)
    assert [d.id for d in instances] == [str(i) for i in range(len(instances))]
    assert all(r.hypothesis.class_id in categories for d in instances for r in d.results)
    assert ids.dtype == np.int32
    assert ids.min() >= 0 and ids.max() < len(instances)


def test_categories_unknown_last_once():
    assert CATEGORIES == ["ground", "tree", "lemon", UNKNOWN]
    assert server_categories(["a", UNKNOWN, "b"]) == ["a", "b", UNKNOWN]


def test_ground_truth_encoding():
    table = ground_truth_table(CATEGORIES)
    assert table[0] == [(UNKNOWN, 0.05)]
    assert table[1:] == [[("ground", 1.0)], [("tree", 1.0)], [("lemon", 1.0)]]
    labels = labels_image()
    ids = encode_ground_truth(labels, NAMES, CATEGORIES)
    assert_contract(table, ids, CATEGORIES)
    expected = {0: 0, 1: 0, 2: 3, 3: 1, 6: 2, 7: 0, 42: 0, -1: 0, 99999: 0}
    for label, inst in expected.items():
        assert (ids[labels == label] == inst).all(), label


def test_confidence_encoding():
    n_bins = 4
    table = confidence_table(CATEGORIES, n_bins)
    assert len(table) == 1 + 3 * n_bins
    labels = np.array([[3, 3, 6, 0]], dtype=np.int32)
    conf = np.array([[0.0, 0.99, 0.6, 0.9]], dtype=np.float32)
    ids = encode_confidence(labels, conf, NAMES, CATEGORIES, n_bins)
    assert_contract(table, ids, CATEGORIES)
    assert ids.tolist() == [[1, 4, 7, 0]]  # ground bin 0, ground bin 3, tree bin 2, unknown
    assert table[4] == [("ground", 0.875), (UNKNOWN, 0.125)]
    nan_ids = encode_confidence(labels, np.full_like(conf, np.nan), NAMES, CATEGORIES, n_bins)
    assert nan_ids.min() >= 0 and nan_ids.max() < len(table)


def test_check_table_rejects_unknown_class():
    with pytest.raises(ValueError):
        check_table([[(UNKNOWN, 0.05)], [("person", 1.0)]], CATEGORIES)


def test_select_points_drops_non_finite():
    xyz = np.array([[1, 0, 0], [np.nan, 0, 0], [np.inf, 0, 0], [0.05, 0, 0], [50, 0, 0]], float)
    assert select_points(xyz, 0.1, 10.0, False).tolist() == [True, False, False, False, True]
    assert select_points(xyz, 0.1, 10.0, True).tolist() == [True, False, False, False, False]


def test_output_cloud_field_set():
    out = np.zeros(3, dtype=OUTPUT_DTYPE)
    cloud = point_cloud2.create_cloud(Header(), OUTPUT_FIELDS, out)
    assert [f.name for f in cloud.fields] == ["x", "y", "z", "rgb", "instance_id"]
    assert cloud.point_step == OUTPUT_DTYPE.itemsize == 20


def test_parse_labels_json():
    text = json.dumps({"0": {"class": "BACKGROUND"}, "3": {"class": "ground"},
                       "time_stamp": {"sec": 7, "nanosec": 5}})
    stamp, table = parse_labels_json(text)
    assert stamp == (7, 5)
    assert table == {0: "BACKGROUND", 3: "ground"}
