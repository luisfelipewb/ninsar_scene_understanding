"""Thin helpers around rosbag2_py: opening bags, storage autodetection, topic metadata."""

from __future__ import annotations

import os
import shutil
from pathlib import Path

import yaml
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageFilter,
    StorageOptions,
    TopicMetadata,
)
from rosbag2_py._storage import QoS
from rosidl_runtime_py.utilities import get_message

SERIALIZATION_FORMAT = "cdr"

#: recordings are laid out as <session name>/bag/bag_0.mcap, so the directory
#: rosbag2 opens is always called this and carries no identifying name itself
BAG_SUBDIR = "bag"


def detect_storage_id(uri: str | Path) -> str:
    """Return the storage plugin for a bag directory ('mcap' or 'sqlite3').

    Prefers the storage_identifier recorded in metadata.yaml, falling back to
    sniffing the file extensions inside the directory.
    """
    path = Path(uri)
    if not path.exists():
        raise FileNotFoundError(f"bag not found: {uri}")
    meta = path / "metadata.yaml"
    if meta.is_file():
        try:
            info = yaml.safe_load(meta.read_text())["rosbag2_bagfile_information"]
            storage_id = info.get("storage_identifier")
            if storage_id:
                return storage_id
        except (KeyError, TypeError, yaml.YAMLError):
            pass  # fall through to sniffing

    if path.is_dir():
        suffixes = {p.suffix for p in path.iterdir()}
        if ".mcap" in suffixes:
            return "mcap"
        if ".db3" in suffixes:
            return "sqlite3"
    elif path.suffix == ".mcap":
        return "mcap"
    elif path.suffix == ".db3":
        return "sqlite3"

    raise ValueError(f"cannot determine bag storage format for '{uri}'")


def open_reader(uri: str | Path, storage_id: str | None = None,
                topics: list[str] | None = None) -> SequentialReader:
    """Open a bag for reading.

    `topics` restricts the reader to those topics. The storage layer skips the
    rest, so scanning one small topic (a preflight check on `/tf_static`, say)
    costs an index lookup rather than a pass over every message in the bag.
    """
    uri = str(resolve_bag(uri))
    storage_id = storage_id or detect_storage_id(uri)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=uri, storage_id=storage_id),
        ConverterOptions(SERIALIZATION_FORMAT, SERIALIZATION_FORMAT),
    )
    if topics:
        reader.set_filter(StorageFilter(topics=list(topics)))
    return reader


def open_writer(uri: str | Path, storage_id: str = "mcap",
                max_bagfile_size: int = 0) -> SequentialWriter:
    """Open a bag for writing. `max_bagfile_size` of 0 means never split.

    rosbag2 names split files after the bag directory, so writing to
    ``<session>_fixed/bag`` reproduces the ``bag_0.mcap``, ``bag_1.mcap`` ...
    convention of the source recordings.
    """
    options = StorageOptions(uri=str(uri), storage_id=storage_id)
    if max_bagfile_size:
        options.max_bagfile_size = max_bagfile_size
    writer = SequentialWriter()
    writer.open(options, ConverterOptions(SERIALIZATION_FORMAT, SERIALIZATION_FORMAT))
    return writer


def source_split_size(bag_dir: str | Path) -> int:
    """Infer the split size of a recording from its existing storage files.

    Returns 0 when the bag was not split, so the output is not split either.
    """
    files = [p for p in Path(bag_dir).iterdir() if p.suffix in (".mcap", ".db3")]
    if len(files) < 2:
        return 0
    return max(p.stat().st_size for p in files)


def prepare_output_dir(uri: str | Path, force: bool = False) -> Path:
    """Ensure the output bag directory does not exist yet."""
    path = Path(uri)
    if path.exists():
        if not force:
            raise FileExistsError(
                f"output '{path}' already exists (use --force to overwrite)"
            )
        shutil.rmtree(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def topic_types(reader: SequentialReader) -> dict[str, str]:
    """Map topic name -> message type string, as recorded in the bag."""
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


_msg_class_cache: dict[str, type] = {}


def message_class(type_name: str) -> type:
    """Resolve (and cache) the Python class for a ROS message type string."""
    cls = _msg_class_cache.get(type_name)
    if cls is None:
        cls = get_message(type_name)
        _msg_class_cache[type_name] = cls
    return cls


def make_topic_metadata(name: str, type_name: str, transient_local: bool = False):
    """Build a TopicMetadata for a topic that does not exist in the source bag."""
    qos = QoS(1)
    if transient_local:
        qos = qos.transient_local().reliable().keep_last(1)
    return TopicMetadata(
        id=0,
        name=name,
        type=type_name,
        serialization_format=SERIALIZATION_FORMAT,
        offered_qos_profiles=[qos],
    )


def rename_topic_metadata(meta, new_name: str):
    """Copy a TopicMetadata, changing only its name (QoS/hash preserved)."""
    return TopicMetadata(
        id=meta.id,
        name=new_name,
        type=meta.type,
        serialization_format=meta.serialization_format,
        offered_qos_profiles=meta.offered_qos_profiles,
        type_description_hash=meta.type_description_hash,
    )


def session_name(path: str | Path) -> str:
    """The identifying folder name for a recording.

    Recordings look like ``2026-07-09-16-39-34/bag/bag_0.mcap``: the directory
    rosbag2 opens is always called ``bag``, so the name that identifies the
    recording is the closest ancestor that is called something else. Used for
    output naming and to key per-bag config overrides.
    """
    path = Path(os.path.normpath(os.path.abspath(str(path))))
    if path.is_file():
        # a loose file only identifies the recording through its own name unless
        # it sits in the usual <session>/bag/ layout
        if path.parent.name != BAG_SUBDIR:
            return path.stem
        path = path.parent
    while path.name == BAG_SUBDIR and path.parent != path:
        path = path.parent
    return path.name


def resolve_bag(path: str | Path) -> Path:
    """Find the rosbag2 directory to open, given a session or bag directory.

    Accepts the session folder (``2026-07-09-16-39-34``), the ``bag`` folder
    inside it, or any directory holding a ``metadata.yaml``.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"not found: {path}")
    if path.is_file():
        path = path.parent

    if (path / "metadata.yaml").is_file():
        return path
    if (path / BAG_SUBDIR / "metadata.yaml").is_file():
        return path / BAG_SUBDIR

    # last resort: a single sub-directory that looks like a bag
    candidates = sorted(
        child for child in path.iterdir()
        if child.is_dir() and (child / "metadata.yaml").is_file()
    )
    if len(candidates) == 1:
        return candidates[0]
    if candidates:
        raise ValueError(
            f"{path} holds several bags ({', '.join(c.name for c in candidates)}); "
            "point at one of them"
        )
    raise FileNotFoundError(
        f"no rosbag2 found in {path} (expected {path}/metadata.yaml or "
        f"{path}/{BAG_SUBDIR}/metadata.yaml)"
    )
