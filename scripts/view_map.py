#!/usr/bin/env python3
"""View a Voxeland map export (voxeland_pointcloud.ply) with Open3D.

Usage:
  python3 tools/view_map.py voxeland_pointcloud.ply            # points
  python3 tools/view_map.py voxeland_pointcloud.ply --voxels   # render as 0.05 m cubes
  python3 tools/view_map.py voxeland_pointcloud.ply --save map.ply   # also write a binary copy (~4x smaller)
"""
import argparse
import numpy as np
import open3d as o3d

ap = argparse.ArgumentParser()
ap.add_argument("ply")
ap.add_argument("--voxels", action="store_true", help="draw cubes instead of points")
ap.add_argument("--voxel-size", type=float, default=0.05, help="server 'resolution' used when mapping")
ap.add_argument("--save", help="write a binary PLY/PCD copy of the cloud")
args = ap.parse_args()

pcd = o3d.io.read_point_cloud(args.ply)
pts = np.asarray(pcd.points)
print(f"{len(pts)} voxels, bounds min={pts.min(0).round(2)} max={pts.max(0).round(2)}, colors={pcd.has_colors()}")

if args.save:
    o3d.io.write_point_cloud(args.save, pcd, write_ascii=False, compressed=True)
    print(f"wrote {args.save}")

geom = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=args.voxel_size) if args.voxels else pcd
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0, origin=pts.min(0))
o3d.visualization.draw_geometries([geom, frame], window_name=args.ply, width=1280, height=800)
