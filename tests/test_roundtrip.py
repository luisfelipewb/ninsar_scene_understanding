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
    mode: append
    transforms:
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
