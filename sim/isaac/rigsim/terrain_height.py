#!/usr/bin/env python3
"""Sample the ground height of a scene's terrain at an (x, y) world position.

Montoldre's terrain is /root/Ground/Mesh_011, an ordinary UsdGeom.Mesh with no
collider on it -- nothing in that scene has one -- so there is nothing to
raycast against without authoring physics first. It does not need it: the mesh
is a 2.5D heightfield, 761 x 767 vertices on a complete rectilinear lattice
with every quad split into two triangles. No (x, y) carries two vertices and no
triangle is vertical, so height is a plain function of (x, y) and a query is a
grid lookup plus a barycentric interpolation.

build_lut() bakes that mesh into an .npz and needs pxr, so it runs under Isaac
Sim's python.sh. TerrainHeight reads the .npz and needs only numpy, which is
what lets tools/bag_to_waypoints.py use it under the ROS 2 interpreter. The
bake is driven by tools/bake_heightmap.py:

    ~/isaacsim/python.sh tools/bake_heightmap.py \
        --usd assets/Montoldre_V3_grouped_clean.usdc --out assets/montoldre_heightmap.npz

Two details of the mesh the lookup has to respect, both checked rather than
assumed. The grid spacing is not quite uniform -- it is ~1.969 m over most of
the sheet but drifts to ~1.83 m in bands -- so cells are found by bisection on
the real axis, not by dividing by a step. And the quad diagonal flips from cell
to cell, so plain bilinear interpolation misses the actual surface by up to a
centimetre; recording each cell's diagonal and evaluating the triangle the
point really falls in brings that to 25 um, which is the float32 precision of
the source points.
"""

import numpy as np

# The terrain in every Montoldre variant in assets/ -- the plain one, the
# backup and the collected-with-cabbages copy carry the same mesh, vertex for
# vertex, so one baked heightmap serves all three.
MESH_PATH = "/root/Ground/Mesh_011"

# Coordinates closer than this along one axis are the same lattice line, split
# only by float32 rounding. Far below the ~1.83 m minimum spacing, far above
# the ~2 cm the rounding actually costs.
AXIS_TOLERANCE = 0.5


def build_lut(usd_path, out_npz, mesh_path=MESH_PATH):
    """Bake a terrain mesh into a heightmap .npz. Needs pxr; run under python.sh.

    Raises if the mesh is not the clean heightfield the lookup assumes, rather
    than baking something subtly wrong: a scene whose ground has overhangs or
    holes needs a raycast, not this.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(usd_path)
    prim = stage.GetPrimAtPath(mesh_path)
    if not prim:
        raise ValueError(f"{usd_path}: no prim at {mesh_path}")

    mesh = UsdGeom.Mesh(prim)
    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    if not np.all(counts == 3):
        raise ValueError(f"{mesh_path}: expected a triangulated mesh, got face sizes {np.unique(counts)}")
    triangles = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)

    # The mesh sits under a translated Xform -- for Montoldre a -283 m drop that
    # is what puts the ground at z = 0 -- so the points alone are not world space.
    matrix = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim), dtype=np.float64)
    world = np.hstack([points, np.ones((len(points), 1))]) @ matrix
    world = world[:, :3] / world[:, 3:4]

    axis_x, axis_y = _axis(world[:, 0]), _axis(world[:, 1])
    if len(axis_x) * len(axis_y) != len(world):
        raise ValueError(
            f"{mesh_path}: a {len(axis_x)} x {len(axis_y)} lattice cannot hold {len(world)} "
            f"vertices, so this mesh is not a regular grid"
        )

    column = _snap(world[:, 0], axis_x)
    row = _snap(world[:, 1], axis_y)
    heights = np.full((len(axis_y), len(axis_x)), np.nan)
    heights[row, column] = world[:, 2]
    if np.isnan(heights).any():
        raise ValueError(
            f"{mesh_path}: {int(np.isnan(heights).sum())} lattice sites have no vertex, "
            f"so the grid has holes"
        )

    diagonals = _diagonals(triangles, column, row, len(axis_x), len(axis_y))
    np.savez_compressed(out_npz, heights=heights, axis_x=axis_x, axis_y=axis_y, diagonals=diagonals)
    return out_npz


def _axis(values, tolerance=AXIS_TOLERANCE):
    """The distinct coordinates along one axis, merging float32 near-duplicates."""
    ordered = np.sort(values)
    edges = np.concatenate([[0], np.flatnonzero(np.diff(ordered) > tolerance) + 1, [len(ordered)]])
    return np.array([ordered[edges[i]:edges[i + 1]].mean() for i in range(len(edges) - 1)])


def _snap(values, axis):
    """Index of the nearest entry of `axis` for each value."""
    return np.clip(np.searchsorted((axis[1:] + axis[:-1]) / 2, values), 0, len(axis) - 1)


def _diagonals(triangles, column, row, nx, ny):
    """Per-cell split direction: 1 where the diagonal runs (i+1,j)-(i,j+1), else 0.

    Each triangle covers one cell and omits exactly one of its four corners,
    which is what names the diagonal. Montoldre's splits are mixed -- 460,886
    cells one way, 121,274 the other -- so there is no single orientation to
    hard-code.
    """
    corners = (row[triangles] - row[triangles].min(axis=1)[:, None]) * 2 + (
        column[triangles] - column[triangles].min(axis=1)[:, None]
    )
    seen = np.zeros((len(triangles), 4), dtype=bool)
    seen[np.arange(len(triangles))[:, None], corners] = True
    absent = np.argmin(seen, axis=1)

    diagonals = np.full((ny - 1, nx - 1), -1, dtype=np.int8)
    diagonals[row[triangles].min(axis=1), column[triangles].min(axis=1)] = (absent == 0) | (absent == 3)
    if (diagonals < 0).any():
        raise ValueError(f"{int((diagonals < 0).sum())} grid cells are covered by no triangle")
    return diagonals


class TerrainHeight:
    """Ground height over a baked heightmap. numpy only, so either interpreter.

    Calling it is vectorised and costs a bisection per query -- a few million a
    second -- so sampling a whole trajectory is free next to building it.
    """

    def __init__(self, npz_path):
        data = np.load(npz_path)
        self.heights = data["heights"]
        self.axis_x = data["axis_x"]
        self.axis_y = data["axis_y"]
        self.diagonals = data["diagonals"]

    @property
    def bounds(self):
        """(x_min, x_max, y_min, y_max) of the terrain footprint, in metres."""
        return (self.axis_x[0], self.axis_x[-1], self.axis_y[0], self.axis_y[-1])

    def outside(self, x, y):
        """Mask of the query points that fall beyond the footprint."""
        x, y = np.atleast_1d(x), np.atleast_1d(y)
        return (
            (x < self.axis_x[0]) | (x > self.axis_x[-1]) | (y < self.axis_y[0]) | (y > self.axis_y[-1])
        )

    def __call__(self, x, y, clamp=True):
        """Ground height at (x, y). Scalars in, scalar out; arrays in, array out.

        Beyond the footprint the query is held at the nearest edge, which keeps
        a path that strays off the sheet continuous; clamp=False returns NaN
        there instead, for callers that would rather hear about it.
        """
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        scalar = x.ndim == 0 and y.ndim == 0
        x, y = np.atleast_1d(x), np.atleast_1d(y)

        i = np.clip(np.searchsorted(self.axis_x, x) - 1, 0, len(self.axis_x) - 2)
        j = np.clip(np.searchsorted(self.axis_y, y) - 1, 0, len(self.axis_y) - 2)
        # Position within the cell. Off the sheet these leave [0, 1], and
        # clamping them is what holds the edge value.
        u = (x - self.axis_x[i]) / (self.axis_x[i + 1] - self.axis_x[i])
        v = (y - self.axis_y[j]) / (self.axis_y[j + 1] - self.axis_y[j])
        u, v = np.clip(u, 0.0, 1.0), np.clip(v, 0.0, 1.0)

        z00 = self.heights[j, i]
        z10 = self.heights[j, i + 1]
        z01 = self.heights[j + 1, i]
        z11 = self.heights[j + 1, i + 1]

        # Each cell is two triangles; which two depends on its diagonal, and
        # which of the pair the point is in follows from u and v. Both branches
        # are just the plane through the triangle's three corners.
        main = np.where(
            u >= v,
            z00 + (z10 - z00) * u + (z11 - z10) * v,
            z00 + (z01 - z00) * v + (z11 - z01) * u,
        )
        anti = np.where(
            u + v <= 1.0,
            z00 + (z10 - z00) * u + (z01 - z00) * v,
            z11 + (z01 - z11) * (1.0 - u) + (z10 - z11) * (1.0 - v),
        )
        z = np.where(self.diagonals[j, i] == 1, anti, main)

        if not clamp:
            z = np.where(self.outside(x, y), np.nan, z)
        return float(z[0]) if scalar else z

