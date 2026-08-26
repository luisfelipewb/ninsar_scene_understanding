#!/usr/bin/env python3
"""End-to-end test on a synthetic bag - no external data needed.

    source /opt/ros/jazzy/setup.bash
    python3 -m pytest tests/test_roundtrip.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import rclpy.serialization as ser

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bagfix import bagio, config as config_mod, pipeline  # noqa: E402

NS = 1_000_000_000
CAMERA_INFO = "sensor_msgs/msg/CameraInfo"
IMAGE = "sensor_msgs/msg/Image"
TF_MESSAGE = "tf2_msgs/msg/TFMessage"

CONFIG = """
robot: test
output: {root: out, suffix: _fixed}
pipeline:
  - transform: frame_id
    topics: ["/cam/*"]
    frame_id: fixed_optical_frame
  - transform: camera_info
    topics: ["/cam/camera_info"]
  - transform: tf_static
    add:
      - {parent: base_link, child: fixed_optical_frame, xyz: [1.0, 2.0, 3.0], rpy: [0, 0, 0]}
  - transform: topic_filter
    drop: ["/junk"]
    rename: {/cam/rgb: /camera/image_raw}
"""


def _write_source_bag(path: Path) -> None:
    writer = bagio.open_writer(path)
    for name, type_name, transient in (
        ("/cam/rgb", IMAGE, False),
        ("/cam/camera_info", CAMERA_INFO, False),
        ("/junk", IMAGE, False),
        ("/other", IMAGE, False),
    ):
        writer.create_topic(bagio.make_topic_metadata(name, type_name, transient))

    image = bagio.message_class(IMAGE)()
    image.header.frame_id = "wrong_frame"
    image.width, image.height = 640, 480

    info = bagio.message_class(CAMERA_INFO)()
    info.header.frame_id = "wrong_frame"
    info.width, info.height = 640, 480
    info.k = [-500.0, 0.0, 9999.0, 0.0, -500.0, 0.0, 0.0, 0.0, 1.0]
    info.r = [0.0] * 9
    info.p = [0.0] * 12

    for i in range(3):
        stamp = (100 + i) * NS
        writer.write("/cam/rgb", ser.serialize_message(image), stamp)
        writer.write("/cam/camera_info", ser.serialize_message(info), stamp)
        writer.write("/junk", ser.serialize_message(image), stamp)
        writer.write("/other", ser.serialize_message(image), stamp)
    writer.close()


def _read_all(path: Path):
    reader = bagio.open_reader(path)
    types = bagio.topic_types(reader)
    messages = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        messages.append((topic, data, stamp))
    return types, messages


@pytest.fixture
def bags(tmp_path):
    source = tmp_path / "2026-07-09-16-39-34" / "bag"
    _write_source_bag(source)
    cfg_path = tmp_path / "test.yaml"
    cfg_path.write_text(CONFIG)
    cfg = config_mod.load(cfg_path)
    out = tmp_path / "bag_fixed"
    result = pipeline.run(source, out, cfg)
    return source, out, result


def test_topics_dropped_and_renamed(bags):
    _, out, _ = bags
    types, _ = _read_all(out)
    assert "/junk" not in types
    assert "/cam/rgb" not in types
    assert types["/camera/image_raw"] == IMAGE
    assert "/other" in types


def test_frame_ids_rewritten(bags):
    _, out, _ = bags
    types, messages = _read_all(out)
    for topic, data, _ in messages:
        if topic in ("/camera/image_raw", "/cam/camera_info"):
            msg = ser.deserialize_message(data, bagio.message_class(types[topic]))
            assert msg.header.frame_id == "fixed_optical_frame"


def test_untouched_topic_is_byte_identical(bags):
    source, out, _ = bags
    _, src_msgs = _read_all(source)
    _, out_msgs = _read_all(out)
    src_other = [(d, t) for topic, d, t in src_msgs if topic == "/other"]
    out_other = [(d, t) for topic, d, t in out_msgs if topic == "/other"]
    assert src_other == out_other


def test_camera_info_repaired(bags):
    _, out, _ = bags
    types, messages = _read_all(out)
    infos = [d for topic, d, _ in messages if topic == "/cam/camera_info"]
    msg = ser.deserialize_message(infos[0], bagio.message_class(CAMERA_INFO))
    assert msg.k[0] == 500.0 and msg.k[4] == 500.0      # sign bug corrected
    assert msg.k[2] == 320.0 and msg.k[5] == 240.0      # degenerate cx/cy -> centre
    assert list(msg.r) == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert list(msg.p) == [500.0, 0.0, 320.0, 0.0,
                           0.0, 500.0, 240.0, 0.0,
                           0.0, 0.0, 1.0, 0.0]


def test_tf_static_synthesized(bags):
    _, out, _ = bags
    types, messages = _read_all(out)
    assert types["/tf_static"] == TF_MESSAGE
    tf_msgs = [d for topic, d, _ in messages if topic == "/tf_static"]
    assert len(tf_msgs) == 1
    msg = ser.deserialize_message(tf_msgs[0], bagio.message_class(TF_MESSAGE))
    tf = msg.transforms[0]
    assert (tf.header.frame_id, tf.child_frame_id) == ("base_link", "fixed_optical_frame")
    assert (tf.transform.translation.x, tf.transform.translation.y,
            tf.transform.translation.z) == (1.0, 2.0, 3.0)
    assert tf.transform.rotation.w == pytest.approx(1.0)


def test_counts(bags):
    _, _, result = bags
    assert result.total_messages == 12
    assert result.dropped == 3            # /junk
    assert result.emitted == 1            # synthesized /tf_static
    assert result.deserialized == 6       # /cam/rgb + /cam/camera_info only


def test_session_name_and_output_layout(tmp_path):
    """Recordings are <session>/bag/bag_N.mcap; the session names the output."""
    session_dir = tmp_path / "test_acqui"
    _write_source_bag(session_dir / "bag")

    # the session folder, the bag folder and a file inside it all resolve alike
    for arg in (session_dir, session_dir / "bag",
                next((session_dir / "bag").glob("*.mcap"))):
        assert bagio.session_name(arg) == "test_acqui"
        assert bagio.resolve_bag(arg) == session_dir / "bag"

    cfg_path = tmp_path / "c.yaml"
    cfg_path.write_text(CONFIG)
    cfg = config_mod.load(cfg_path)
    out = pipeline.default_output("test_acqui", cfg, tmp_path / "out")
    assert out == tmp_path / "out" / "test_acqui_fixed" / "bag"

    pipeline.run(session_dir, out, cfg)
    # rosbag2 names split files after the bag directory, keeping the convention
    assert (out / "metadata.yaml").is_file()
    assert sorted(p.name for p in out.glob("*.mcap")) == ["bag_0.mcap"]


def test_missing_bag_is_reported(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError, match="no rosbag2 found"):
        bagio.resolve_bag(tmp_path / "empty")
    with pytest.raises(FileNotFoundError, match="not found"):
        bagio.resolve_bag(tmp_path / "nope")


def _tf_static_bag(tmp_path, pairs):
    """A bag whose /tf_static carries the given (parent, child) transforms."""
    source = tmp_path / "session" / "bag"
    writer = bagio.open_writer(source)
    writer.create_topic(bagio.make_topic_metadata("/tf_static", TF_MESSAGE, True))
    msg = bagio.message_class(TF_MESSAGE)()
    for parent, child in pairs:
        tf_stamped = bagio.message_class("geometry_msgs/msg/TransformStamped")()
        tf_stamped.header.frame_id = parent
        tf_stamped.child_frame_id = child
        tf_stamped.transform.rotation.w = 1.0
        msg.transforms.append(tf_stamped)
    writer.write("/tf_static", ser.serialize_message(msg), 100 * NS)
    writer.close()
    return source


def _tf_config(tmp_path, rule_yaml):
    cfg_path = tmp_path / "tf.yaml"
    cfg_path.write_text("robot: t\npipeline:\n  - transform: tf_static\n" + rule_yaml)
    return config_mod.load(cfg_path)


def _tf_pairs(bag):
    """Every (parent, child) on /tf_static in a bag, plus its topic table."""
    types, messages = _read_all(bag)
    found = []
    for topic, data, _ in messages:
        if topic == "/tf_static":
            msg = ser.deserialize_message(data, bagio.message_class(TF_MESSAGE))
            found += [(t.header.frame_id, t.child_frame_id) for t in msg.transforms]
    return found, types


def _run_tf_rule(tmp_path, pairs, rule_yaml):
    source = _tf_static_bag(tmp_path, pairs)
    out = tmp_path / "out" / "bag"
    pipeline.run(source, out, _tf_config(tmp_path, rule_yaml))
    return _tf_pairs(out)


def test_tf_static_remove_edit_and_add_in_one_rule(tmp_path):
    """The three lists apply in order: remove, then edit, then add."""
    source = _tf_static_bag(
        tmp_path, [("base_link", "old_lidar"), ("base_link", "imu")])
    out = tmp_path / "out" / "bag"
    pipeline.run(source, out, _tf_config(
        tmp_path,
        "    remove:\n"
        "      - {child: old_lidar}\n"
        "    edit:\n"
        "      - {parent: base_link, child: imu, xyz: [0.1, 0.2, 0.3], rpy: [0, 0, 0]}\n"
        "    add:\n"
        "      - {parent: base_link, child: lidar, xyz: [0.3, 0, 1.4], rpy: [0, 0, 0]}\n",
    ))

    found, _ = _tf_pairs(out)
    assert found == [("base_link", "imu"), ("base_link", "lidar")]

    # the edited pose really changed (the fixture writes an all-zero translation)
    _, messages = _read_all(out)
    msg = ser.deserialize_message(
        next(d for t, d, _ in messages if t == "/tf_static"),
        bagio.message_class(TF_MESSAGE))
    imu = next(t for t in msg.transforms if t.child_frame_id == "imu")
    assert (imu.transform.translation.x, imu.transform.translation.y,
            imu.transform.translation.z) == pytest.approx((0.1, 0.2, 0.3))


def test_tf_static_remove_accepts_globs_and_a_lone_child(tmp_path):
    found, _ = _run_tf_rule(
        tmp_path,
        [("base_link", "cam_left"), ("base_link", "cam_right"), ("base_link", "imu"),
         ("odom", "junk")],
        "    remove:\n"
        "      - {parent: base_link, child: 'cam_*'}\n"
        "      - {child: junk}\n",          # any parent
    )
    assert found == [("base_link", "imu")]


def test_tf_static_drops_a_message_emptied_by_removal(tmp_path):
    found, types = _run_tf_rule(
        tmp_path,
        [("base_link", "junk_a"), ("base_link", "junk_b")],
        "    remove:\n      - {parent: base_link}\n",
    )
    assert found == []
    # the topic stays declared, it just carries nothing now
    assert "/tf_static" in types


def test_tf_static_remove_only_does_not_create_the_topic(tmp_path):
    """A remove-only rule has nothing to put in a bag that has no /tf_static."""
    source = tmp_path / "session" / "bag"
    _write_source_bag(source)  # this fixture has no /tf_static
    out = tmp_path / "out" / "bag"
    pipeline.run(source, out,
                 _tf_config(tmp_path, "    remove:\n      - {child: anything}\n"))

    types, _ = _read_all(out)
    assert "/tf_static" not in types


# ---- preflight: the rule must match the bag, and fail before writing -------


def test_tf_static_edit_of_a_missing_pair_fails_before_writing(tmp_path):
    source = _tf_static_bag(tmp_path, [("base_link", "imu")])
    out = tmp_path / "out" / "bag"
    config = _tf_config(
        tmp_path,
        "    edit:\n"
        "      - {parent: base_link, child: typo_frame, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )

    with pytest.raises(ValueError, match="does not have"):
        pipeline.run(source, out, config)
    # the whole point: nothing was written
    assert not out.exists()
    assert not out.parent.exists()


def test_tf_static_add_of_an_existing_pair_fails_before_writing(tmp_path):
    source = _tf_static_bag(tmp_path, [("base_link", "imu")])
    out = tmp_path / "out" / "bag"
    config = _tf_config(
        tmp_path,
        "    add:\n"
        "      - {parent: base_link, child: imu, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )

    with pytest.raises(ValueError, match="already has"):
        pipeline.run(source, out, config)
    assert not out.exists()


def test_tf_static_edit_needs_the_topic_to_exist(tmp_path):
    """Editing a bag with no /tf_static at all is still an error."""
    source = tmp_path / "session" / "bag"
    _write_source_bag(source)
    config = _tf_config(
        tmp_path,
        "    edit:\n"
        "      - {parent: base_link, child: lidar, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )
    with pytest.raises(ValueError, match="does not have"):
        pipeline.run(source, tmp_path / "out" / "bag", config)


def test_tf_static_preflight_also_runs_on_a_dry_run(tmp_path):
    """A dry run is how you check a config, so it must catch this too."""
    source = _tf_static_bag(tmp_path, [("base_link", "imu")])
    config = _tf_config(
        tmp_path,
        "    edit:\n"
        "      - {parent: base_link, child: typo_frame, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )
    with pytest.raises(ValueError, match="does not have"):
        pipeline.run(source, tmp_path / "out" / "bag", config, dry_run=True)


# ---- config-time rejections ------------------------------------------------


def test_tf_static_rejects_globs_in_edit_and_add(tmp_path):
    """A glob would give several frames one pose; it belongs in 'remove'."""
    for listing in ("edit", "add"):
        config = _tf_config(
            tmp_path,
            f"    {listing}:\n"
            "      - {parent: base_link, child: 'cam_*', xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
        )
        with pytest.raises(ValueError, match="exact frame names"):
            pipeline.build_transforms(config)


def test_tf_static_rejects_contradictory_rules(tmp_path):
    config = _tf_config(
        tmp_path,
        "    remove:\n      - {parent: base_link, child: lidar}\n"
        "    edit:\n"
        "      - {parent: base_link, child: lidar, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )
    with pytest.raises(ValueError, match="both 'edit' and 'remove'"):
        pipeline.build_transforms(config)


def test_tf_static_rejects_the_same_pair_in_edit_and_add(tmp_path):
    config = _tf_config(
        tmp_path,
        "    edit:\n"
        "      - {parent: base_link, child: lidar, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n"
        "    add:\n"
        "      - {parent: base_link, child: lidar, xyz: [1, 0, 0], rpy: [0, 0, 0]}\n",
    )
    with pytest.raises(ValueError, match="both 'edit' and 'add'"):
        pipeline.build_transforms(config)


def test_tf_static_rejects_a_removal_that_names_nothing(tmp_path):
    config = _tf_config(tmp_path, "    remove:\n      - {}\n")
    with pytest.raises(ValueError, match="needs 'parent', 'child', or both"):
        pipeline.build_transforms(config)


def test_tf_static_rejects_the_retired_keys(tmp_path):
    """An old config must fail loudly, not quietly leave the bag unrepaired."""
    mode_config = _tf_config(
        tmp_path,
        "    mode: patch\n"
        "    edit:\n"
        "      - {parent: a, child: b, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )
    with pytest.raises(ValueError, match="'mode' is gone"):
        pipeline.build_transforms(mode_config)

    transforms_config = _tf_config(
        tmp_path,
        "    transforms:\n"
        "      - {parent: a, child: b, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n",
    )
    with pytest.raises(ValueError, match="'transforms' is gone"):
        pipeline.build_transforms(transforms_config)


def test_tf_static_warns_when_a_removal_matches_nothing(tmp_path):
    source = _tf_static_bag(tmp_path, [("base_link", "wheel")])
    config = _tf_config(tmp_path, "    remove:\n      - {child: not_here}\n")
    result = pipeline.run(source, tmp_path / "out" / "bag", config)

    notes = " ".join(result.transforms[0].notes)
    assert "no transform matched remove" in notes


def test_stamp_offset_auto(tmp_path):
    """The legacy lidar fix: header stamps 2s behind the receive time."""
    source = tmp_path / "lidar"
    writer = bagio.open_writer(source)
    writer.create_topic(bagio.make_topic_metadata("/lidar/points", IMAGE))
    msg = bagio.message_class(IMAGE)()
    for i in range(5):
        msg.header.stamp.sec = 100 + i
        writer.write("/lidar/points", ser.serialize_message(msg), (102 + i) * NS)
    writer.close()

    cfg_path = tmp_path / "lidar.yaml"
    cfg_path.write_text(
        "robot: t\npipeline:\n"
        "  - transform: stamp_offset\n    topics: ['/lidar/*']\n    mode: auto\n"
    )
    out = tmp_path / "lidar_fixed"
    pipeline.run(source, out, config_mod.load(cfg_path))

    types, messages = _read_all(out)
    stamps = [
        ser.deserialize_message(d, bagio.message_class(IMAGE)).header.stamp.sec
        for _, d, _ in messages
    ]
    assert stamps == [102, 103, 104, 105, 106]


def test_per_bag_override_and_set(tmp_path):
    cfg_path = tmp_path / "c.yaml"
    cfg_path.write_text(
        "robot: t\n"
        "pipeline:\n"
        "  - transform: frame_id\n    topics: ['/a']\n    frame_id: one\n"
        "bags:\n  2026-01-01:\n    frame_id: {frame_id: two}\n"
    )
    assert config_mod.load(cfg_path).pipeline[0].options["frame_id"] == "one"
    assert config_mod.load(cfg_path, "2026-01-01").pipeline[0].options["frame_id"] == "two"
    cfg = config_mod.load(cfg_path, sets=["frame_id.frame_id=three"])
    assert cfg.pipeline[0].options["frame_id"] == "three"


def test_bad_config_is_rejected(tmp_path):
    cfg_path = tmp_path / "c.yaml"
    cfg_path.write_text("robot: t\npipeline:\n  - transform: frame_id\n    topics: ['/a']\n")
    with pytest.raises(ValueError, match="frame_id"):
        pipeline.build_transforms(config_mod.load(cfg_path))
