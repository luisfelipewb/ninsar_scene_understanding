"""Shared modules for run_scene.py and the tools.

Every module here is importable under both interpreters -- Isaac Sim's python.sh
and the system ROS 2 python -- because none depends on omni/pxr at import time.
terrain_height.build_lut() is the one exception and imports pxr lazily.
"""
