"""convert_ros1 calls the rosbags-convert CLI; a stub on PATH stands in for it."""

import os
import stat
import sys

import pytest

from bagfix import convert_ros1


def _stub(tmp_path, body):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    exe = bin_dir / convert_ros1.CONVERT_CLI
    exe.write_text(f"#!{sys.executable}\n{body}\n")
    exe.chmod(exe.stat().st_mode | stat.S_IEXEC)
    return bin_dir


def test_calls_cli_with_src_dst_and_storage(tmp_path, monkeypatch):
    log = tmp_path / "args.txt"
    bin_dir = _stub(tmp_path, f"import sys, pathlib\npathlib.Path({str(log)!r}).write_text(' '.join(sys.argv[1:]))")
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    src = tmp_path / "run.bag"
    src.write_bytes(b"")
    dest = tmp_path / "out" / "run_ros2" / "bag"

    assert convert_ros1.convert(src, dest, storage="mcap") == dest
    assert log.read_text() == f"--src {src} --dst {dest} --dst-storage mcap"


def test_cli_failure_is_reported(tmp_path, monkeypatch):
    bin_dir = _stub(tmp_path, "import sys\nsys.stderr.write('bad bag')\nsys.exit(2)")
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    src = tmp_path / "run.bag"
    src.write_bytes(b"")

    with pytest.raises(convert_ros1.ConversionError, match="bad bag"):
        convert_ros1.convert(src, tmp_path / "dest")


def test_missing_cli_explains_install(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", str(tmp_path))
    src = tmp_path / "run.bag"
    src.write_bytes(b"")

    with pytest.raises(convert_ros1.ConversionError, match="pipx install rosbags"):
        convert_ros1.convert(src, tmp_path / "dest")
