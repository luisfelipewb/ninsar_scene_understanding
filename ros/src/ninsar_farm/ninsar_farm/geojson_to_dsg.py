#!/usr/bin/env python3
"""Build the farm scene graph (docs/FARM_DSG_SPEC.md) from a farm GeoJSON prior
(docs/FARM_GEOJSON_SPEC.md), as written by the farm editor.

The file is validated first (farm_geojson.check); a file with errors is refused. Tree
parents, overlay membership, ordering and measures come from farm_geojson.FarmIndex; this
script turns them into nodes and edges and derives the navigation waypoints.

Usage:
  ros2 run ninsar_farm geojson_to_dsg FILE.geojson [-o OUT] [--set KEY=VALUE ...]
      # -> data/dsg/<site>_dsg.json; exit 1 if the file is refused or the graph breaks a spec rule
  ros2 launch hydra_visualizer static_visualizer.launch.yaml \
    scene_graph:=$PWD/data/dsg/<site>_dsg.json \
    visualizer_config_path:=$(ros2 pkg prefix --share ninsar_bringup)/config/farm_dsg_viz.yaml
"""
import argparse
import hashlib
import itertools
import math
import os
import sys

import numpy as np
import shapely
import spark_dsg as dsg
from shapely.geometry import LineString, Point, box

from ninsar_farm import farm_dsg as fd
from ninsar_farm import farm_geojson as fg
from ninsar_farm.paths import data_dir

ROW_OBJECTS = {"crop", "row_support"}
BLOCKER_CATEGORIES = {"obstacle", "movable", "vegetation", "structure", "equipment"}
NOT_BLOCKERS = {"gate"}
STRAIGHT_ROW_M = 0.5  # lanes assume straight rows; warn beyond this deviation


class ImportRefused(Exception):
    def __init__(self, issues):
        self.issues = issues
        n = sum(i.level == "error" for i in issues)
        super().__init__(f"the file has {n} validation errors")


def hex_rgb(color):
    c = color.lstrip("#")
    return tuple(int(c[i:i + 2], 16) for i in (0, 2, 4))


def round_coords(coords, nd=3):
    if isinstance(coords, (int, float)):
        return round(float(coords), nd)
    return [round_coords(c, nd) for c in coords]


def enu_geometry(geom):
    """Shapely geometry (ENU) -> GeoJSON geometry dict, coordinates rounded to 1 mm."""
    m = shapely.geometry.mapping(geom)
    return {"type": m["type"], "coordinates": round_coords(m["coordinates"])}


def oriented_box(geom, size):
    """(centre xy, (long, short), yaw) of the minimum rotated rectangle of a geometry.

    A Point gets the class default box, axis-aligned. A short side below the default's smaller
    horizontal size (a straight line, a thin polygon) is raised to it."""
    if geom.geom_type == "Point":
        return (geom.x, geom.y), (size[0], size[1]), 0.0
    rect = geom.minimum_rotated_rectangle
    if rect.geom_type == "Polygon":
        c = np.asarray(rect.exterior.coords)[:3]
        e1, e2 = c[1] - c[0], c[2] - c[1]
        if np.linalg.norm(e1) < np.linalg.norm(e2):
            e1, e2 = e2, e1
        long_, short = float(np.linalg.norm(e1)), float(np.linalg.norm(e2))
        centre = rect.centroid
    else:  # collinear geometry: the rectangle degenerates to a segment
        c = np.asarray(rect.coords)
        e1 = c[-1] - c[0]
        long_, short = float(np.linalg.norm(e1)), 0.0
        centre = rect.interpolate(0.5, normalized=True)
    yaw = math.atan2(e1[1], e1[0]) % math.pi
    return (centre.x, centre.y), (long_, max(short, min(size[0], size[1]))), yaw


def footprint(geom, size):
    """Planar footprint of an object: its geometry, or the default box around a Point."""
    if geom.geom_type != "Point":
        return geom
    return box(geom.x - size[0] / 2, geom.y - size[1] / 2, geom.x + size[0] / 2, geom.y + size[1] / 2)


class Importer:
    def __init__(self, fc, idx, schema, source_path=None):
        self.fc, self.idx, self.schema, self.source_path = fc, idx, schema, source_path
        self.nodes, self.d, self.th = idx.nodes, idx.derived, idx.th
        self.f = fd.FarmGraph()
        self.G = self.f.G
        self.warnings = []
        self.order = sorted(self.nodes, key=lambda fid: fg.sort_key({"id": fid, "properties": self.nodes[fid].props}))
        self.wp = {}  # waypoint uid -> {"xy", "kind"}
        self._wp_count = itertools.count()
        self.road_wps, self.lanes, self.headlands = [], {}, {}  # lanes[parcel][lane_index] = [uids]
        self.nav_edges = []  # (a, b, relation, extra metadata), linked last with the blocked flags
        self._nav_pairs = set()
        self.wp_overlays = {}  # waypoint uid -> restricted-area overlay uids
        self.no_entry = set()  # waypoints in a no_entry area

    def warn(self, fid, msg):
        self.warnings.append(fg.Issue(fid, "warning", "import", msg))

    def run(self):
        self.add_nodes()
        self.add_tree_and_overlays()
        self.add_spatial()
        self.add_roads()
        self.add_lanes()
        self.add_restricted_waypoints()
        self.add_entrances()
        self.add_access()
        self.link_navigation()
        self.add_metadata()
        return self.G

    # --- nodes ---------------------------------------------------------------------------

    def size(self, kind):
        return self.schema.default_size_m.get(kind)

    def color(self, n):
        if n.kind in self.schema.crops and n.layer in ("row", "object"):
            return hex_rgb(self.schema.crops[n.kind]["color"])
        return hex_rgb(n.spec.get("color", "#000000"))

    def node_metadata(self, n):
        m = {"uid": n.id, "kind": n.kind, "source": "prior", "status": "expected", "modified": False,
             "confidence": 1.0}
        m.update({k: v for k, v in n.props.items() if k not in ("layer", "kind", "name")})
        d = self.d[n.id]
        m.update({k: d[k] for k in n.spec.get("derived", []) if k in d and k != "members"})  # members: edges
        if n.geom.geom_type != "Point":
            m["geometry"] = enu_geometry(n.geom)
        return m

    def add_nodes(self):
        th = self.th
        for fid in self.order:
            n = self.nodes[fid]
            if n.layer == "navigation":
                continue  # roads become waypoints
            prefix, num = fg.parse_id(fid)
            sym = dsg.NodeSymbol(prefix, num)
            m = self.node_metadata(n)
            xy = self.d[fid]["position"]
            name = n.props.get("name") or fid
            if n.layer == "object":
                size = self.size(n.kind)
                centre, dims, yaw = oriented_box(n.geom, size)
                if n.category == "crop":
                    row = self.nodes[self.d[fid]["parent"]]
                    if "variety" not in m and row.props.get("variety"):
                        m["variety"] = row.props["variety"]
                z = size[2] / 2
                self.f.add(sym, fd.OBJECTS, cls=dsg.ObjectNodeAttributes, name=name, position=(*xy, z),
                           color=self.color(n), box=((*dims, size[2]), yaw, (*centre, z)),
                           label=self.schema.label_ids[n.kind], metadata=m)
            elif n.layer == "row":
                size = self.schema.default_size_m[n.kind]
                centre, dims, yaw = oriented_box(n.geom, size)
                m["crop"] = n.kind
                if "plant_spacing_m" not in m:
                    spacing = self.plant_spacing(n)
                    if spacing:
                        m["plant_spacing_m"] = spacing
                z = size[2] / 2
                self.f.add(sym, fd.ROWS, name=name, position=(*xy, z), color=self.color(n),
                           box=((dims[0] + size[0], max(dims[1], size[1]), size[2]), yaw, (*centre, z)),
                           label=self.schema.label_ids[n.kind], metadata=m)
            else:  # farm, overlay, area: flat boxes
                if n.layer == "area" and n.kind == "parcel" and "row_spacing_m" not in m and fid in self.idx.spacing:
                    m["row_spacing_m"] = round(self.idx.spacing[fid], 3)
                h = th["flat_height_m"]
                centre, dims, yaw = oriented_box(n.geom, (0.0, 0.0, h))
                layer, partition = {"farm": (fd.FARM, fd.P0), "overlay": (fd.FARM, fd.OVERLAYS),
                                    "area": (fd.AREAS, fd.P0)}[n.layer]
                self.f.add(sym, layer, partition, name=name, position=(*xy, h / 2), color=self.color(n),
                           box=((*dims, h), yaw, (*centre, h / 2)), metadata=m)

    def plant_spacing(self, row):
        plants = [self.nodes[k] for k in self.d[row.id].get("children", []) if self.nodes[k].category == "crop"]
        along = sorted(row.geom.project(p.geom) for p in plants)
        return round(float(np.median(np.diff(along))), 3) if len(along) > 1 else None

    # --- tree, overlays, spatial ---------------------------------------------------------

    def children(self, fid, layer=None, category=None):
        kids = [self.nodes[k] for k in self.d[fid].get("children", [])]
        return [k for k in kids if (layer is None or k.layer == layer) and (category is None or k.category == category)]

    def add_tree_and_overlays(self):
        for fid in self.order:
            parent = self.d[fid].get("parent")
            if parent and self.nodes[fid].layer != "navigation":
                self.f.link(parent, fid, "contains")
        for z in self.idx.by_layer["overlay"]:
            for m in self.d[z.id]["members"]:
                self.f.link(m, z.id, "member_of")

    def rows_of(self, parcel_id):
        return sorted(self.children(parcel_id, "row"), key=lambda r: self.d[r.id]["row_index"])

    def add_spatial(self):
        th = self.th
        for r in sorted(self.idx.by_layer["row"], key=lambda r: fg.parse_id(r.id)[1]):
            plants = sorted(self.children(r.id, "object", "crop"), key=lambda p: self.d[p.id]["row_index"])
            for a, b in zip(plants, plants[1:]):
                self.f.link(a.id, b.id, "next_in_row")
        for a in sorted(self.idx.by_layer["area"], key=lambda a: fg.parse_id(a.id)[1]):
            rows = self.rows_of(a.id)
            for r0, r1 in zip(rows, rows[1:]):
                self.f.link(r0.id, r1.id, "adjacent_row")

        areas = sorted(self.idx.by_layer["area"], key=lambda a: fg.parse_id(a.id)[1])
        for a, b in itertools.combinations(areas, 2):
            gap = a.geom.distance(b.geom)
            if gap <= th["adjacent_area_m"]:
                self.f.link(a.id, b.id, "adjacent_area", gap_m=round(gap, 2))

        objects = [self.nodes[fid] for fid in self.order if self.nodes[fid].layer == "object"]
        self.footprints = {o.id: footprint(o.geom, self.size(o.kind)) for o in objects}
        geoms = [self.footprints[o.id] for o in objects]
        if len(geoms) < 2:
            return
        i, j = shapely.STRtree(geoms).query(geoms, predicate="dwithin", distance=th["near_m"])
        for a, b in sorted({(min(p, q), max(p, q)) for p, q in zip(i.tolist(), j.tolist()) if p != q}):
            oa, ob = objects[a], objects[b]
            if oa.category in ROW_OBJECTS and ob.category in ROW_OBJECTS and \
                    self.d[oa.id]["parent"] == self.d[ob.id]["parent"]:
                continue
            self.f.link(oa.id, ob.id, "near")

    # --- navigation ----------------------------------------------------------------------

    def waypoint(self, xy, kind, **extra):
        u = f"w{next(self._wp_count)}"
        meta = {"uid": u, "kind": kind, "source": "prior", "status": "expected", "modified": False,
                "confidence": 1.0, **extra}
        xy = (round(float(xy[0]), 3), round(float(xy[1]), 3))
        self.f.add(dsg.NodeSymbol("w", int(u[1:])), fd.OBJECTS, fd.NAV, position=(*xy, 0.0),
                   color=fd.NAV_COLORS[kind], metadata=meta)
        self.wp[u] = {"xy": np.array(xy), "kind": kind}
        return u

    def nav(self, a, b, rel, **extra):
        pair = frozenset((a, b))
        if a != b and pair not in self._nav_pairs:
            self._nav_pairs.add(pair)
            self.nav_edges.append((a, b, rel, extra))

    def add_roads(self):
        th = self.th
        roads = sorted(self.idx.by_layer["navigation"], key=lambda r: fg.parse_id(r.id)[1])
        for r in roads:
            line = r.geom
            keys = {0.0, line.length}
            acc = 0.0
            c = np.asarray(line.coords)
            for seg in np.hypot(*np.diff(c, axis=0).T):
                acc += float(seg)
                keys.add(min(acc, line.length))
            for o in roads:
                if o is r:
                    continue
                for p in shapely.get_parts(line.intersection(o.geom)):
                    if p.geom_type == "Point":
                        keys.add(line.project(p))
                for end in (Point(o.geom.coords[0]), Point(o.geom.coords[-1])):
                    if line.distance(end) <= th["junction_m"]:
                        keys.add(line.project(end))
            keys = sorted(keys)
            dists = [keys[0]]
            for d0, d1 in zip(keys, keys[1:]):
                if d1 - d0 < 1e-6:
                    continue
                n = max(1, math.ceil((d1 - d0) / th["road_step_m"] - 1e-9))
                dists += [d0 + (d1 - d0) * k / n for k in range(1, n + 1)]
            prev = None
            extra = {"road": r.id, **({"width_m": r.props["width_m"]} if "width_m" in r.props else {})}
            for s in dists:
                p = line.interpolate(s)
                xy = np.array([p.x, p.y])
                near = [w for w in self.road_wps if np.linalg.norm(self.wp[w]["xy"] - xy) <= th["junction_m"]]
                w = near[0] if near else self.waypoint(xy, "road", **extra)
                if not near:
                    self.road_wps.append(w)
                if prev is not None:
                    self.nav(prev, w, "road")
                prev = w

    def add_lanes(self):
        th = self.th
        for parcel in sorted(self.idx.by_layer["area"], key=lambda a: fg.parse_id(a.id)[1]):
            rows = self.rows_of(parcel.id)
            if parcel.kind != "parcel" or not rows:
                continue
            h = math.radians(self.d[parcel.id]["heading_deg"])
            u = np.array([math.cos(h), math.sin(h)])
            c0 = np.asarray(rows[0].geom.coords)
            if (c0[-1] - c0[0]) @ u < 0:  # point u along the rows, so "start" is the row-start side
                u = -u
            nrm = np.array([-u[1], u[0]])
            spacing = self.idx.spacing[parcel.id]
            offsets, spans = [], []
            for r in rows:
                c = np.asarray(r.geom.coords)
                chord = LineString([c[0], c[-1]])
                dev = max(chord.distance(Point(p)) for p in c)
                if dev > STRAIGHT_ROW_M:
                    self.warn(r.id, f"row bends {dev:.2f} m off its chord; lanes assume straight rows")
                offsets.append(float(np.asarray(r.geom.centroid.coords[0]) @ nrm))
                spans.append(((c @ u).min(), (c @ u).max()))
            sgn = 1.0 if len(rows) == 1 or offsets[-1] >= offsets[0] else -1.0
            lane_offsets = ([offsets[0] - sgn * spacing / 2]
                            + [(a + b) / 2 for a, b in zip(offsets, offsets[1:])]
                            + [offsets[-1] + sgn * spacing / 2])
            ext = parcel.props["headland_m"] / 2 if parcel.props.get("headland_m") else th["row_end_m"]
            zone = self.idx.zone(parcel)
            lanes = {}
            for i, off in enumerate(lane_offsets):
                beside = spans[max(i - 1, 0):i + 1]
                lo, hi = min(s[0] for s in beside), max(s[1] for s in beside)
                full = LineString([(lo - ext) * u + off * nrm, (hi + ext) * u + off * nrm])
                parts = [g for g in shapely.get_parts(full.intersection(zone)) if g.geom_type == "LineString"]
                core = LineString([lo * u + off * nrm, hi * u + off * nrm])
                if not parts:
                    self.warn(parcel.id, f"lane {i} lies outside the parcel; skipped")
                    continue
                part = max(parts, key=lambda g: g.intersection(core.buffer(1e-6)).length)
                pc = np.asarray(part.coords)
                if (pc[-1] - pc[0]) @ u < 0:
                    part = part.reverse()
                n = max(1, math.ceil(part.length / th["lane_step_m"] - 1e-9))
                wps = []
                for k in range(n + 1):
                    p = part.interpolate(part.length * k / n)
                    end = k in (0, n)
                    extra = {"parcel": parcel.id, "lane_index": i}
                    if end:
                        extra["side"] = "start" if k == 0 else "end"
                    wps.append(self.waypoint((p.x, p.y), "headland" if end else "lane", **extra))
                for a, b in zip(wps, wps[1:]):
                    self.nav(a, b, "lane")
                lanes[i] = wps
            for i in sorted(lanes):
                if i + 1 in lanes:
                    self.nav(lanes[i][0], lanes[i + 1][0], "headland")
                    self.nav(lanes[i][-1], lanes[i + 1][-1], "headland")
            self.lanes[parcel.id] = lanes
            self.headlands[parcel.id] = [w for i in sorted(lanes) for w in (lanes[i][0], lanes[i][-1])]

    def add_restricted_waypoints(self):
        if not self.wp:
            return
        wps = list(self.wp)
        tree = shapely.STRtree([Point(self.wp[w]["xy"]) for w in wps])
        restricted = sorted((z for z in self.idx.by_layer["overlay"] if z.kind == "restricted_area"),
                            key=lambda z: fg.parse_id(z.id)[1])
        for z in restricted:
            for i in sorted(tree.query(self.idx.zone(z), predicate="covers").tolist()):
                self.f.link(wps[i], z.id, "member_of")
                self.wp_overlays.setdefault(wps[i], []).append(z.id)
        self.no_entry = {w for w, zs in self.wp_overlays.items()
                         if any(self.nodes[z].props.get("restriction") == "no_entry" for z in zs)}

    def nearest(self, xy, candidates):
        cands = [w for w in candidates if w not in self.no_entry]
        if not cands:
            return None, math.inf
        dist = np.linalg.norm(np.array([self.wp[w]["xy"] for w in cands]) - np.asarray(xy), axis=1)
        k = int(np.argmin(dist))
        return cands[k], float(dist[k])

    def add_entrances(self):
        """Headland -> road. Each parcel gets its nearest unblocked pair (or, failing that, its
        nearest pair), plus every headland waypoint within `entrance_m` of a road waypoint."""
        th = self.th
        if not self.road_wps:
            if self.headlands:
                self.warn(None, "no roads: parcels are not connected to each other")
            return
        roads = [w for w in self.road_wps if w not in self.no_entry]
        road_xy = np.array([self.wp[w]["xy"] for w in roads]) if roads else np.zeros((0, 2))
        for pid, heads in self.headlands.items():
            heads = [h for h in heads if h not in self.no_entry]
            if not heads or not roads:
                self.warn(pid, "no entrance: every headland or road waypoint is in a no_entry area")
                continue
            dist = np.linalg.norm(np.array([self.wp[h]["xy"] for h in heads])[:, None] - road_xy[None], axis=2)
            for i, h in enumerate(heads):
                j = int(np.argmin(dist[i]))
                if dist[i, j] <= th["entrance_m"]:
                    self.nav(h, roads[j], "entrance")
            pairs = sorted(np.ndindex(dist.shape), key=lambda ij: dist[ij])
            best = next((ij for ij in pairs if not self.blocked_by(heads[ij[0]], roads[ij[1]], "entrance")), pairs[0])
            h, w, length = heads[best[0]], roads[best[1]], float(dist[best])
            if frozenset((h, w)) in self._nav_pairs:
                continue
            self.nav(h, w, "entrance")
            hits = self.blocked_by(h, w, "entrance")
            self.warn(pid, f"entrance {h} -> {w} is {length:.1f} m long (entrance_m = {th['entrance_m']})"
                           + (f" and blocked by {', '.join(hits)}" if hits else ""))

    def parcel_of(self, fid):
        return next((p for p in [fid] + self.idx.lineage(fid) if p in self.lanes), None)

    def add_access(self):
        th = self.th
        outer = self.road_wps + [w for heads in self.headlands.values() for w in heads]
        for fid in self.order:
            o = self.nodes[fid]
            if o.layer != "object":
                continue
            xy = self.d[fid]["position"]
            parent = self.d[fid]["parent"]
            if self.nodes[parent].layer == "row":
                lanes = self.lanes.get(self.d[parent]["parent"], {})
                i = self.d[parent]["row_index"]
                for lane in (lanes.get(i), lanes.get(i + 1)):
                    w, _ = self.nearest(xy, lane or [])
                    if w is not None:
                        self.f.link(fid, w, "access")
                continue
            pid = self.parcel_of(fid)
            cands = outer + ([w for lane in self.lanes[pid].values() for w in lane] if pid else [])
            w, dist = self.nearest(xy, cands)
            if o.category == "service_point" and w is None:
                w, dist = self.nearest(xy, list(self.wp))
            if w is None:
                if o.category == "service_point":
                    self.warn(fid, "service point without a reachable waypoint")
                continue
            if dist <= th["access_m"]:
                self.f.link(fid, w, "access")
            elif o.category == "service_point":
                self.f.link(fid, w, "access")
                self.warn(fid, f"nearest waypoint {w} is {dist:.1f} m away (access_m = {th['access_m']})")

    def blocked_by(self, a, b, rel):
        """Uids of the objects (and, for an entrance, water bodies and no_entry overlays) that
        lie within `clearance_m` of the straight segment a-b."""
        if not hasattr(self, "_blockers"):
            self._blockers = [o for o in self.idx.by_layer["object"]
                              if o.category in BLOCKER_CATEGORIES and o.kind not in NOT_BLOCKERS]
            geoms = [self.footprints[o.id] for o in self._blockers]
            self._blocker_tree = shapely.STRtree(geoms) if geoms else None
            self._no_go = [n for n in self.idx.by_layer["area"] if n.kind == "water_body"] + \
                [z for z in self.idx.by_layer["overlay"] if z.props.get("restriction") == "no_entry"]
        seg = LineString([self.wp[a]["xy"], self.wp[b]["xy"]])
        hits = []
        if self._blocker_tree is not None:
            hits = [self._blockers[i].id for i in
                    self._blocker_tree.query(seg.buffer(self.th["clearance_m"]), predicate="intersects")]
        if rel == "entrance":
            hits += [n.id for n in self._no_go if seg.intersects(n.geom)]
        return sorted(set(hits), key=fg.parse_id)

    def link_navigation(self):
        for a, b, rel, extra in self.nav_edges:
            hits = self.blocked_by(a, b, rel)
            if hits:
                extra = {**extra, "blocked": True, "blocked_by": hits}
            self.f.link(a, b, rel, **extra)

    # --- graph metadata ------------------------------------------------------------------

    def add_metadata(self):
        sch = self.schema
        objects = {i: c for c, i in sorted(sch.label_ids.items(), key=lambda kv: kv[1])}
        crops = {i: c for i, c in objects.items() if c in sch.crops}
        self.G.set_labelspace(dsg.Labelspace(objects), fd.OBJECTS, 0)
        self.G.set_labelspace(dsg.Labelspace(crops), fd.ROWS, 0)
        if not self.fc.get("origin"):
            self.warn(None, "no origin in the file; the farm centroid is used")
        prior = {"farm_geojson": self.fc.get("farm_geojson"), "schema_version": sch.version}
        if self.source_path:
            with open(self.source_path, "rb") as fh:
                prior.update(file=os.path.basename(self.source_path), sha256=hashlib.sha256(fh.read()).hexdigest())
        self.G.metadata.add({
            "spec_version": fd.DSG_SPEC_VERSION, "site": self.fc.get("site", ""),
            "origin": self.idx.frame.origin(), "frame": "map", "units": "m",
            "thresholds": dict(sorted(self.th.items())),
            "labelspaces": {str(fd.OBJECTS): {str(i): c for i, c in objects.items()},
                            str(fd.ROWS): {str(i): c for i, c in crops.items()}},
            "prior": prior,
        })


def build(fc, schema=None, thresholds=None, source_path=None):
    """(graph, issues). Raises ImportRefused when the file has validation errors."""
    schema = schema or fg.default_schema()
    issues, idx = fg.check(fc, schema, thresholds)
    if any(i.level == "error" for i in issues):
        raise ImportRefused(issues)
    imp = Importer(fc, idx, schema, source_path)
    return imp.run(), issues + imp.warnings


# --- CLI --------------------------------------------------------------------------------

def report(G, out=sys.stdout):
    fd.summary(G, out)
    services = sorted((n.id.str() for n in G.get_layer(fd.OBJECTS).nodes if n.id.category == "s"),
                      key=fg.parse_id)
    if not services:
        return
    start = services[0]
    out.write(f"== routes from {start} to the first plant of each parcel\n")
    for a in sorted(G.get_layer(fd.AREAS).nodes, key=lambda n: n.id.category_id):
        rows = sorted(fd.children(G, a.id), key=lambda r: fd.meta(G.get_node(fd.symbol(r)).attributes).get("row_index", 0))
        plants = [c for r in rows[:1] for c in fd.children(G, r)
                  if fd.meta(G.get_node(fd.symbol(c)).attributes).get("category") == "crop"
                  and fd.meta(G.get_node(fd.symbol(c)).attributes).get("row_index") == 0]
        if not plants:
            continue
        length, path = fd.route(G, start, plants[0])
        free, _ = fd.route(G, start, plants[0], avoid_blocked=False, avoid_no_entry=False)
        detour = f" ({free:.1f} m ignoring blocked edges and no_entry)" if path and free < length - 1e-6 else ""
        out.write(f"  {a.id.str()} {plants[0]}: " + (f"{length:.1f} m over {len(path) - 2} waypoints{detour}\n"
                                                    if path else "unreachable\n"))


def parse_set(items):
    out = {}
    for item in items:
        key, _, value = item.partition("=")
        out[key] = float(value)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="farm GeoJSON file")
    ap.add_argument("-o", "--output", help="graph file: .json (readable) or any other extension (binary); "
                                           "default data/dsg/<site>_dsg.json")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override a threshold of config/farm_schema.yaml, e.g. --set near_m=0.5")
    ap.add_argument("-q", "--quiet", action="store_true", help="skip the report")
    args = ap.parse_args(argv)

    fc = fg.load(args.file)
    try:
        G, issues = build(fc, thresholds=parse_set(args.set), source_path=args.file)
    except ImportRefused as e:
        for i in e.issues:
            print(f"{i.level:7s} {i.rule:6s} {i.id or '-':8s} {i.message}")
        print(f"refused: {e}", file=sys.stderr)
        return 1
    for i in issues:
        print(f"{i.level:7s} {i.rule:6s} {i.id or '-':8s} {i.message}")
    stem = fc.get("site") or os.path.splitext(os.path.basename(args.file))[0]
    output = args.output or str(data_dir() / "dsg" / f"{stem}_dsg.json")
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    G.save(output, include_mesh=False)
    G = dsg.DynamicSceneGraph.load(output)  # check and report on the saved file, proving it round-trips
    violations = fd.verify(G)
    if not args.quiet:
        report(G)
    fd.print_violations(violations)
    n_err = sum(v.level == "error" for v in violations)
    print(f"{G.num_nodes()} nodes, {G.num_edges()} edges -> {output}"
          + (f"; {n_err} spec violations" if n_err else ""))
    return 1 if n_err else 0


if __name__ == "__main__":
    sys.exit(main())
