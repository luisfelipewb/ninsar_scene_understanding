#!/usr/bin/env python3
"""Farm GeoJSON profile (docs/specs/FARM_GEOJSON_SPEC.md): load/save, ids, the local ENU frame,
derived structure (§6), validation (§5) and the row/plant/post generators.

Shared by the farm editor (ninsar_farm/editor/serve.py) and the DSG importer. All geometry
work runs in the local ENU frame, in metres, with shapely; the file stays in WGS84.

Usage:
  ros2 run ninsar_farm farm_geojson validate FILE   # exit 1 on errors
  ros2 run ninsar_farm farm_geojson tree FILE [--full]
"""
import argparse
import copy
import json
import math
import os
import re
import shutil
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass

import numpy as np
import shapely
import shapely.validation
import yaml
from ament_index_python.packages import get_package_share_directory
from shapely.geometry import LineString, Point, shape
from shapely.ops import substring

PROFILE_VERSION = "0.1"
SCHEMA_PATH = os.path.join(get_package_share_directory("ninsar_farm"), "config", "farm_schema.yaml")
LAYER_ORDER = ("farm", "overlay", "area", "row", "object", "navigation")
COMMON_KEYS = {"layer", "category", "kind", "name", "description"}
ID_RE = re.compile(r"^([A-Za-z])(\d+)$")
COORD_DECIMALS = 8  # about 1 mm
LENGTH_EPS_M = 0.01  # generators: 8-decimal rounding moves each vertex by up to ~0.6 mm


# --- schema catalog ---------------------------------------------------------------------

class Schema:
    """config/farm_schema.yaml with the `crops` placeholder expanded."""

    def __init__(self, path=SCHEMA_PATH):
        with open(path) as f:
            raw = yaml.safe_load(f)
        self.version = raw["version"]
        self.crops = dict(raw["crops"])
        self.thresholds = dict(raw["thresholds"])
        self.reserved = set(raw["reserved"])
        self.default_size_m = {**raw["default_size_m"],
                               **{name: c["default_size_m"] for name, c in self.crops.items()}}
        self.workflow = raw.get("workflow", [])
        self.type_list = []
        self.types = {}  # (layer, category, kind) -> type entry
        for entry in raw["types"]:
            kinds = list(self.crops) if entry["kinds"] == "crops" else list(entry["kinds"])
            spec = {"category": None, "attributes": [], "parents": [], "derived": [], **entry, "kinds": kinds}
            self.type_list.append(spec)
            for kind in kinds:
                self.types[(spec["layer"], spec["category"], kind)] = spec
        self.label_ids = dict(raw.get("label_ids", {}))
        classes = {k for t in self.type_list if t["layer"] == "object" for k in t["kinds"]}
        missing = sorted(classes - set(self.label_ids))
        dup = [i for i, c in Counter(self.label_ids.values()).items() if c > 1]
        if missing or dup:
            raise ValueError(f"{path}: label_ids missing for {missing}, duplicated ids {dup}")

    def type_of(self, props):
        return self.types.get((props.get("layer"), props.get("category"), props.get("kind")))

    def entry(self, layer, category=None, kind=None):
        """Type entry for a workflow button: by kind when given, else the first of the layer/category."""
        return next((t for t in self.type_list if t["layer"] == layer and t["category"] == category
                     and (kind is None or kind in t["kinds"])), None)

    def to_json(self):
        return {"version": self.version, "crops": self.crops, "thresholds": self.thresholds,
                "reserved": sorted(self.reserved), "default_size_m": self.default_size_m,
                "label_ids": self.label_ids, "types": self.type_list, "workflow": self.workflow}


_DEFAULT_SCHEMA = None


def default_schema():
    global _DEFAULT_SCHEMA
    if _DEFAULT_SCHEMA is None:
        _DEFAULT_SCHEMA = Schema()
    return _DEFAULT_SCHEMA


# --- local frame (§3) -------------------------------------------------------------------

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
EARTH_R = 6371000.0


def geodetic_to_ecef(lat, lon, alt):
    lat, lon = np.radians(lat), np.radians(lon)
    n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
    return np.stack([(n + alt) * np.cos(lat) * np.cos(lon),
                     (n + alt) * np.cos(lat) * np.sin(lon),
                     (n * (1 - WGS84_E2) + alt) * np.sin(lat)], axis=-1)


def ecef_to_geodetic(xyz):
    x, y, z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1 - WGS84_E2))
    for _ in range(6):
        n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
        alt = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1 - WGS84_E2 * n / (n + alt)))
    return np.degrees(lat), np.degrees(lon), alt


class Frame:
    """Local ENU tangent frame (`map`) at an origin on the WGS84 ellipsoid."""

    def __init__(self, lat, lon, alt=0.0):
        self.lat, self.lon, self.alt = float(lat), float(lon), float(alt)
        self.ecef0 = geodetic_to_ecef(self.lat, self.lon, self.alt)
        sl, cl = math.sin(math.radians(self.lat)), math.cos(math.radians(self.lat))
        so, co = math.sin(math.radians(self.lon)), math.cos(math.radians(self.lon))
        self.R = np.array([[-so, co, 0.0], [-sl * co, -sl * so, cl], [cl * co, cl * so, sl]])  # rows: E, N, U

    @classmethod
    def from_origin(cls, origin):
        return cls(origin["lat"], origin["lon"], origin.get("alt", 0.0))

    def origin(self):
        return {"lat": self.lat, "lon": self.lon, "alt": self.alt}

    def enu(self, lonlat):
        """(N, 2) [lon, lat] -> (N, 2) [x, y] metres, for points at the origin altitude."""
        lonlat = np.asarray(lonlat, dtype=float).reshape(-1, 2)
        ecef = geodetic_to_ecef(lonlat[:, 1], lonlat[:, 0], np.full(len(lonlat), self.alt))
        return ((ecef - self.ecef0) @ self.R.T)[:, :2]

    def wgs84(self, xy):
        """(N, 2) [x, y] metres -> (N, 2) [lon, lat], inverse of enu()."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        z = -(xy[:, 0] ** 2 + xy[:, 1] ** 2) / (2 * EARTH_R)  # stay on the origin altitude
        ecef = self.ecef0 + np.column_stack([xy, z]) @ self.R
        lat, lon, _ = ecef_to_geodetic(ecef)
        return np.column_stack([lon, lat])


def to_enu(geometry, frame):
    """GeoJSON geometry dict -> 2D shapely geometry in ENU metres."""
    return shapely.transform(shape(geometry), frame.enu)


def to_geojson(geom, frame):
    """Shapely geometry in ENU metres -> GeoJSON geometry dict in WGS84."""
    return round_geometry(shapely.geometry.mapping(shapely.transform(geom, frame.wgs84)))


def round_coords(coords):
    if isinstance(coords, (int, float)):
        return round(float(coords), COORD_DECIMALS)
    return [round_coords(c) for c in coords]


def round_geometry(geometry):
    if not geometry or "coordinates" not in geometry:
        return geometry
    return {"type": geometry["type"], "coordinates": round_coords(geometry["coordinates"])}


def heading_deg(dx, dy, period=360.0):
    """Angle counter-clockwise from east, in [0, period) and rounded to 0.01° (§3)."""
    return round(math.degrees(math.atan2(dy, dx)) % period, 2) % period


# --- file io (§2) -----------------------------------------------------------------------

def parse_id(fid):
    m = ID_RE.match(fid) if isinstance(fid, str) else None
    return (m.group(1), int(m.group(2))) if m else None


def new_collection(site=""):
    return {"type": "FeatureCollection", "farm_geojson": PROFILE_VERSION, "site": site,
            "next_id": {}, "features": []}


def used_counters(fc):
    used = {}
    for f in fc.get("features", []):
        parsed = parse_id(f.get("id"))
        if parsed:
            used[parsed[0]] = max(used.get(parsed[0], 0), parsed[1] + 1)
    return used


def normalize(fc):
    """Fill missing collection members (§2) in place. Existing values are kept as they are."""
    fc["type"] = "FeatureCollection"
    fc.setdefault("farm_geojson", PROFILE_VERSION)
    fc.setdefault("site", "")
    fc.setdefault("features", [])
    if "next_id" not in fc:
        fc["next_id"] = used_counters(fc)
    return fc


def farm_feature(fc):
    return next((f for f in fc.get("features", [])
                 if (f.get("properties") or {}).get("layer") == "farm"), None)


def ensure_origin(fc):
    """Set `origin` to the farm centroid when it is missing (it is never moved afterwards)."""
    if fc.get("origin") is None:
        farm = farm_feature(fc)
        try:
            c = shape(farm["geometry"]).centroid
            fc["origin"] = {"lat": round(c.y, COORD_DECIMALS), "lon": round(c.x, COORD_DECIMALS), "alt": 0.0}
        except Exception:  # no usable farm polygon yet
            pass
    return fc


def frame_of(fc):
    """Frame at `origin`, else at the farm centroid, else at the first coordinate found."""
    if fc.get("origin"):
        return Frame.from_origin(fc["origin"])
    probe = ensure_origin({"features": fc.get("features", [])})
    if probe.get("origin"):
        return Frame.from_origin(probe["origin"])
    for f in fc.get("features", []):
        try:
            p = shape(f["geometry"]).representative_point()
            return Frame(p.y, p.x)
        except Exception:
            continue
    return Frame(0.0, 0.0)


def sort_key(f):
    layer = (f.get("properties") or {}).get("layer")
    parsed = parse_id(f.get("id")) or ("~", 0)
    return (LAYER_ORDER.index(layer) if layer in LAYER_ORDER else len(LAYER_ORDER), parsed[0], parsed[1])


def dumps(fc):
    """One feature per line: diff-friendly and still compact for thousands of plants."""
    head = {"type": fc["type"], **{k: v for k, v in fc.items() if k not in ("type", "features")}}
    lines = ["{"] + [f"  {json.dumps(k)}: {json.dumps(v, ensure_ascii=False)}," for k, v in head.items()]
    feats = [f"    {json.dumps(f, ensure_ascii=False)}" for f in fc["features"]]
    lines += ['  "features": [', ",\n".join(feats), "  ]", "}"] if feats else ['  "features": []', "}"]
    return "\n".join(lines) + "\n"


def load(path):
    with open(path) as f:
        return normalize(json.load(f))


def save(path, fc):
    """Normalize, round, sort and write atomically. The previous file is kept as `<path>.bak`."""
    fc = normalize(copy.deepcopy(fc))
    ensure_origin(fc)
    for f in fc["features"]:
        f["geometry"] = round_geometry(f.get("geometry"))
    fc["features"].sort(key=sort_key)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if os.path.exists(path):
        shutil.copy2(path, path + ".bak")
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(dumps(fc))
    os.replace(tmp, path)
    return fc


def new_id(fc, prefix):
    """Next free id for `prefix`; increments `next_id` so ids are never reused (§4.1)."""
    counters = fc.setdefault("next_id", {})
    n = max(counters.get(prefix, 0), used_counters(fc).get(prefix, 0))
    counters[prefix] = n + 1
    return f"{prefix}{n}"


# --- parsed features --------------------------------------------------------------------

@dataclass
class Issue:
    id: object  # feature id, or None for collection-level issues
    level: str  # "error" | "warning"
    rule: str
    message: str


@dataclass
class Node:
    id: str
    props: dict
    spec: dict
    geom: object  # shapely, ENU metres

    @property
    def layer(self):
        return self.props["layer"]

    @property
    def kind(self):
        return self.props["kind"]

    @property
    def category(self):
        return self.props.get("category")

    def label(self):
        name = self.props.get("name")
        return f"{self.id} {self.kind}" + (f' "{name}"' if name else "")


def _geometry_issue(geometry, spec):
    """Problem with a GeoJSON geometry in WGS84 (G3 type, G4 validity), or None."""
    gtype = (geometry or {}).get("type")
    if gtype not in spec["geometry"]:
        return "G3", f"geometry {gtype} not allowed (allowed: {', '.join(spec['geometry'])})"
    try:
        g = shape(geometry)
    except Exception as e:
        return "G4", f"malformed geometry: {e}"
    if g.is_empty:
        return "G4", "empty geometry"
    xy = shapely.get_coordinates(g)
    if not np.isfinite(xy).all() or (np.abs(xy[:, 0]) > 180).any() or (np.abs(xy[:, 1]) > 90).any():
        return "G4", "coordinates are not [lon, lat] in degrees"
    if gtype == "LineString" and len({tuple(c) for c in xy}) < 2:
        return "G4", "line needs at least two distinct vertices"
    return None


def parse_nodes(fc, schema, frame, issues=None):
    """Features with a usable id, type and geometry, as ENU Nodes. Problems go to `issues`."""
    def report(fid, rule, msg):
        if issues is not None:
            issues.append(Issue(fid, "error", rule, msg))

    nodes = {}
    for i, f in enumerate(fc.get("features", [])):
        fid = f.get("id")
        if not isinstance(fid, str) or not fid:
            report(None, "G2", f"feature #{i} has no string id")
            continue
        if fid in nodes:
            report(fid, "G2", "duplicate id")
            continue
        props = f.get("properties") or {}
        spec = schema.type_of(props)
        if spec is None:
            what = "/".join(str(props.get(k)) for k in ("layer", "category", "kind"))
            report(fid, "G3", f"unknown type {what}")
            continue
        problem = _geometry_issue(f.get("geometry"), spec)
        if problem:
            report(fid, *problem)
            continue
        geom = to_enu(f["geometry"], frame)
        if geom.geom_type == "Polygon" and not geom.is_valid:
            report(fid, "G4", f"invalid polygon: {shapely.validation.explain_validity(geom)}")
            continue
        if geom.geom_type == "LineString" and not geom.is_simple:
            report(fid, "G4", "line crosses itself")
            continue
        nodes[fid] = Node(fid, props, spec, geom)
    return nodes


# --- derived structure (§6) -------------------------------------------------------------

def _extend(line, d):
    """LineString lengthened by `d` metres at both ends, along its end segments."""
    c = np.asarray(line.coords, dtype=float)
    u0, u1 = c[0] - c[1], c[-1] - c[-2]
    c[0] += u0 / np.linalg.norm(u0) * d
    c[-1] += u1 / np.linalg.norm(u1) * d
    return LineString(c)


def _circular_mean_mod180(degs):
    a = np.radians(np.asarray(degs) * 2.0)
    return round(math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()) / 2.0) % 180.0, 2) % 180.0


class FarmIndex:
    """Parsed features plus everything derived from them (§6), in the ENU frame.

    `derived[id]` holds, as applicable: parent, children, overlays, members, roads, row_index,
    n_plants, n_rows, crops, length_m, heading_deg, area_m2, position.
    """

    def __init__(self, fc, schema=None, thresholds=None, issues=None):
        self.schema = schema or default_schema()
        self.th = {**self.schema.thresholds, **(thresholds or {})}
        self.frame = frame_of(fc)
        self.nodes = parse_nodes(fc, self.schema, self.frame, issues)
        self.derived = {fid: {} for fid in self.nodes}
        self._zones = {}
        self.strips = {}
        self.spacing = {}  # area id -> row spacing used for its strips (file value or median)
        by_layer = defaultdict(list)
        for n in self.nodes.values():
            by_layer[n.layer].append(n)
        self.by_layer = by_layer
        farms = by_layer["farm"]
        self.farm = farms[0] if len(farms) == 1 else None
        self._derive()

    # containment with tolerance (T3)
    def zone(self, n):
        if n.id not in self._zones:
            g = n.geom.buffer(self.th["contain_tol_m"])
            shapely.prepare(g)
            self._zones[n.id] = g
        return self._zones[n.id]

    def inside(self, outer, inner_geom):
        return self.zone(outer).covers(inner_geom)

    def _place(self, geom, area_tree, areas):
        """Lowest of area / farm whose footprint contains `geom`."""
        for i in area_tree.query(geom, predicate="covered_by"):
            return areas[i].id
        return self.farm.id if self.farm and self.inside(self.farm, geom) else None

    def _derive(self):
        d, th = self.derived, self.th
        areas, rows, objects = self.by_layer["area"], self.by_layer["row"], self.by_layer["object"]
        for n in self.nodes.values():
            g = n.geom
            p = g if g.geom_type == "Point" else (
                g.interpolate(0.5, normalized=True) if g.geom_type == "LineString" else g.centroid)
            d[n.id]["position"] = [round(p.x, 3), round(p.y, 3)]
        for n in self.by_layer["farm"] + areas:
            d[n.id]["area_m2"] = round(n.geom.area, 1)

        area_tree = shapely.STRtree([self.zone(a) for a in areas])
        for a in areas:
            d[a.id]["parent"] = self.farm.id if self.farm and self.inside(self.farm, a.geom) else None

        # rows: parent area, order and strips
        rows_by_area = defaultdict(list)
        for r in rows:
            c = np.asarray(r.geom.coords)
            d[r.id].update(parent=self._place(r.geom, area_tree, areas), length_m=round(r.geom.length, 3),
                           heading_deg=heading_deg(*(c[-1] - c[0])))
            rows_by_area[d[r.id]["parent"]].append(r)
        for aid, group in rows_by_area.items():
            heading = _circular_mean_mod180([d[r.id]["heading_deg"] for r in group])
            normal = np.array([-math.sin(math.radians(heading)), math.cos(math.radians(heading))])
            offsets = {r.id: float(np.asarray(r.geom.centroid.coords[0]) @ normal) for r in group}
            ordered = sorted(group, key=lambda r: offsets[r.id])
            for i, r in enumerate(ordered):
                d[r.id]["row_index"] = i
            spacing = self.nodes[aid].props.get("row_spacing_m") if aid in self.nodes else None
            if not isinstance(spacing, (int, float)) or spacing <= 0:
                gaps = np.diff([offsets[r.id] for r in ordered])
                spacing = float(np.median(gaps)) if len(gaps) else None
            half = spacing / 2 if spacing and spacing > 0 else th["row_strip_m"]
            self.spacing[aid] = 2 * half
            for r in group:
                self.strips[r.id] = _extend(r.geom, th["row_end_m"]).buffer(half, cap_style="flat")
            if aid in self.nodes and self.nodes[aid].kind == "parcel":
                d[aid].update(heading_deg=heading, n_rows=len(group),
                              crops=sorted({r.kind for r in group}))

        # objects: row strip (nearest axis wins), else area, else farm (T4)
        strip_rows = [self.nodes[rid] for rid in self.strips]
        strip_tree = shapely.STRtree([self.strips[r.id] for r in strip_rows])
        for o in objects:
            hits = [strip_rows[i] for i in strip_tree.query(o.geom, predicate="covered_by")]
            if hits:
                ref = o.geom if o.geom.geom_type == "Point" else o.geom.centroid
                d[o.id]["parent"] = min(hits, key=lambda r: r.geom.distance(ref)).id
            else:
                d[o.id]["parent"] = self._place(o.geom, area_tree, areas)

        children = defaultdict(list)
        for fid, v in d.items():
            if v.get("parent"):
                children[v["parent"]].append(fid)
        for pid, kids in children.items():
            d[pid]["children"] = sorted(kids, key=lambda k: sort_key({"id": k, "properties": self.nodes[k].props}))
        for r in rows:
            plants = [self.nodes[k] for k in d[r.id].get("children", []) if self.nodes[k].category == "crop"]
            plants.sort(key=lambda p: r.geom.project(p.geom))
            for i, p in enumerate(plants):
                d[p.id]["row_index"] = i
            d[r.id]["n_plants"] = len(plants)
        self._derive_overlays()

    def container(self, n):
        """Footprint that holds all of a node's tree descendants."""
        return self.strips.get(n.id, n.geom)

    def _derive_overlays(self):
        d = self.derived
        roads = self.by_layer["navigation"]
        top = d[self.farm.id].get("children", []) if self.farm else []
        direct = defaultdict(list)
        for z in self.by_layer["overlay"]:
            zone, members = self.zone(z), []

            def collect(fid):
                n = self.nodes[fid]
                if zone.covers(n.geom):
                    members.append(fid)
                elif zone.intersects(self.container(n)):
                    for kid in d[fid].get("children", []):
                        collect(kid)

            for fid in top:
                collect(fid)
            d[z.id]["members"] = members
            d[z.id]["roads"] = [r.id for r in roads if zone.intersects(r.geom)]
            for m in members:
                direct[m].append(z.id)

        def inherit(fid, above):
            mine = sorted(set(above) | set(direct.get(fid, [])))
            if mine:
                d[fid]["overlays"] = mine
            for kid in d[fid].get("children", []):
                inherit(kid, mine)

        for fid in top:
            inherit(fid, [])

    def lineage(self, fid):
        out = []
        while (fid := self.derived.get(fid, {}).get("parent")):
            out.append(fid)
        return out


def derive(fc, schema=None, thresholds=None):
    return FarmIndex(fc, schema, thresholds).derived


# --- validation (§5) --------------------------------------------------------------------

def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _attribute_issues(fid, props, spec, schema):
    out = []
    for key in sorted(set(props) & schema.reserved):
        out.append(Issue(fid, "error", "G6", f"reserved key '{key}' (derived on import)"))
    for a in spec["attributes"]:
        name, v = a["name"], props.get(a["name"])
        if v is None or v == "":
            if a.get("required"):
                out.append(Issue(fid, "error", "G5", f"missing required '{name}'"))
            continue
        t, bad = a["type"], None
        if t in ("str", "uid_ref") and not isinstance(v, str):
            bad = "must be a string"
        elif t == "enum" and v not in a["values"]:
            bad = f"'{v}' not in {a['values']}"
        elif t == "int" and not (isinstance(v, int) and not isinstance(v, bool)):
            bad = "must be an integer"
        elif t == "float" and not _is_number(v):
            bad = "must be a number"
        elif a.get("unit") == "m" and _is_number(v) and v <= 0:
            bad = "must be a positive length"
        if bad:
            out.append(Issue(fid, "error", "G5", f"'{name}' {bad}"))
    return out


def _intersecting_pairs(geoms):
    """Index pairs (i < j) of intersecting geometries."""
    if len(geoms) < 2:
        return []
    i, j = shapely.STRtree(geoms).query(np.array(geoms, dtype=object), predicate="intersects")
    return [(a, b) for a, b in zip(i.tolist(), j.tolist()) if a < b]


def _overlap_issues(nodes, tol, rule, what):
    out = []
    for i, j in _intersecting_pairs([n.geom for n in nodes]):
        inter = nodes[i].geom.intersection(nodes[j].geom)
        if not inter.buffer(-tol / 2).is_empty:
            out.append(Issue(nodes[j].id, "error", rule, f"{what} overlaps {nodes[i].id} ({inter.area:.2f} m²)"))
    return out


def _parent_tag(idx, pid):
    n = idx.nodes[pid]
    return {"farm": {"farm"}, "row": {"row"}}.get(n.layer, {"area", n.kind})


def check(fc, schema=None, thresholds=None):
    """Validate `fc` (§5). Returns (issues, FarmIndex)."""
    schema = schema or default_schema()
    issues = []
    idx = FarmIndex(fc, schema, thresholds, issues)
    nodes, d, th = idx.nodes, idx.derived, idx.th
    error = lambda fid, rule, msg: issues.append(Issue(fid, "error", rule, msg))  # noqa: E731
    warn = lambda fid, rule, msg: issues.append(Issue(fid, "warning", rule, msg))  # noqa: E731

    # G1
    n_farms = sum(1 for f in fc.get("features", []) if (f.get("properties") or {}).get("layer") == "farm")
    if n_farms != 1:
        error(None, "G1", f"expected exactly one farm feature, found {n_farms}")

    # G2 (format, prefix, counter), G5, G6
    next_id = fc.get("next_id")
    for n in nodes.values():
        parsed = parse_id(n.id)
        if not parsed:
            error(n.id, "G2", "id must be <prefix><number>, e.g. A3")
        elif parsed[0] != n.spec["prefix"]:
            error(n.id, "G2", f"id prefix must be '{n.spec['prefix']}' for a {n.kind}")
        elif next_id is not None and parsed[1] >= next_id.get(parsed[0], 0):
            error(n.id, "G2", f"id counter not below next_id['{parsed[0]}'] = {next_id.get(parsed[0], 0)}")
        issues.extend(_attribute_issues(n.id, n.props, n.spec, schema))

    # G7
    if idx.farm:
        for n in nodes.values():
            if n is not idx.farm and not idx.inside(idx.farm, n.geom):
                error(n.id, "G7", f"{n.kind} is not inside the farm boundary")
    outside = lambda n: d[n.id].get("parent") is None  # noqa: E731  already reported by G7

    # G8
    issues.extend(_overlap_issues(idx.by_layer["area"], th["overlap_tol_m"], "G8", "area"))

    # G9, G12
    for r in idx.by_layer["row"]:
        pid = d[r.id]["parent"]
        if outside(r):
            continue
        if nodes[pid].layer != "area":
            error(r.id, "G9", "row is not inside one area")
        elif nodes[pid].kind != "parcel":
            error(r.id, "G9", f"row lies in {nodes[pid].kind} {pid}, not in a parcel")
    rows = idx.by_layer["row"]
    for i, j in _intersecting_pairs([r.geom for r in rows]):
        if d[rows[i].id]["parent"] == d[rows[j].id]["parent"]:
            error(rows[j].id, "G12", f"row crosses {rows[i].id}")

    # G10, G11
    for n in idx.by_layer["area"] + idx.by_layer["object"]:
        pid = d[n.id]["parent"]
        if outside(n):
            continue
        if n.category in ("crop", "row_support"):
            if nodes[pid].layer != "row":
                error(n.id, "G10", f"{n.kind} is not on a row (nearest container: {pid})")
            elif n.category == "crop" and nodes[pid].kind != n.kind:
                error(n.id, "G10", f"{n.kind} on a {nodes[pid].kind} row {pid} (one crop per row)")
        elif not _parent_tag(idx, pid) & set(n.spec["parents"]):
            error(n.id, "G11", f"a {n.kind} cannot be inside {nodes[pid].kind} {pid} "
                               f"(allowed: {', '.join(n.spec['parents'])})")

    # G13
    groups = defaultdict(list)
    for z in idx.by_layer["overlay"]:
        excl = z.spec.get("exclusive")
        if excl is True:
            groups[(z.kind,)].append(z)
        elif isinstance(excl, dict):
            groups[(z.kind, z.props.get(excl["per"]))].append(z)
    for key, group in groups.items():
        what = key[0] if len(key) == 1 else f"{key[0]} ({key[1]})"
        issues.extend(_overlap_issues(group, th["overlap_tol_m"], "G13", f"exclusive {what}"))

    # G14 (every uid_ref attribute)
    all_ids = {f.get("id"): f for f in fc.get("features", [])}
    for n in nodes.values():
        for a in n.spec["attributes"]:
            ref = n.props.get(a["name"])
            if a["type"] != "uid_ref" or not isinstance(ref, str) or not ref:
                continue
            target = all_ids.get(ref)
            if target is None:
                error(n.id, "G14", f"'{a['name']}' refers to missing id {ref}")
            elif a.get("ref_kind") and (target.get("properties") or {}).get("kind") not in a["ref_kind"]:
                error(n.id, "G14", f"'{a['name']}' {ref} is not a {' / '.join(a['ref_kind'])}")

    # G15
    for z in idx.by_layer["overlay"]:
        if not d[z.id]["members"] and not d[z.id]["roads"]:
            warn(z.id, "G15", "overlay contains no tree node and no road")

    # G16
    roads = idx.by_layer["navigation"]
    for s in (o for o in idx.by_layer["object"] if o.category == "service_point"):
        dist = min((r.geom.distance(s.geom) for r in roads), default=math.inf)
        if dist > th["access_m"]:
            warn(s.id, "G16", f"no road within {th['access_m']} m (nearest: {dist:.1f} m)")

    order = {"error": 0, "warning": 1}
    issues.sort(key=lambda i: (order[i.level], i.rule, str(i.id)))
    return issues, idx


def validate(fc, schema=None, thresholds=None):
    return check(fc, schema, thresholds)[0]


# --- generators (§5 of FARM_EDITOR.md) --------------------------------------------------

def edge_heading(polygon, edge=None):
    """Heading of exterior edge `edge` (index), or of the longest edge."""
    c = np.asarray(polygon.exterior.coords)
    seg = c[1:] - c[:-1]
    i = int(np.argmax(np.hypot(seg[:, 0], seg[:, 1]))) if edge is None else int(edge) % len(seg)
    return heading_deg(*seg[i])


def generate_rows(parcel, heading, row_spacing_m, headland_m=0.0, first_offset_m=None, count=None,
                  min_length_m=1.0):
    """Parallel row axes across `parcel` (ENU Polygon), running along `heading` degrees.

    Rows start `first_offset_m` (default: half a spacing) from the parcel side and stop the
    same distance from the opposite side. Each axis is clipped to the parcel and trimmed by
    `headland_m` at both ends; a concave parcel can split one axis into several rows.
    """
    h = math.radians(heading)
    u, nrm = np.array([math.cos(h), math.sin(h)]), np.array([-math.sin(h), math.cos(h)])
    pts = np.asarray(parcel.exterior.coords)
    along, across = pts @ u, pts @ nrm
    first = row_spacing_m / 2 if first_offset_m is None else first_offset_m
    lo, hi = along.min() - 1.0, along.max() + 1.0
    rows, k = [], 0
    while (o := across.min() + first + k * row_spacing_m) <= across.max() - first + LENGTH_EPS_M:
        k += 1
        axis = LineString([lo * u + o * nrm, hi * u + o * nrm])
        parts = [g for g in shapely.get_parts(axis.intersection(parcel)) if g.geom_type == "LineString"]
        for part in sorted(parts, key=lambda g: np.asarray(g.coords[0]) @ u):
            if part.length - 2 * headland_m < min_length_m:
                continue
            seg = substring(part, headland_m, part.length - headland_m)
            c = np.asarray(seg.coords)
            rows.append(seg if (c[-1] - c[0]) @ u >= 0 else seg.reverse())
            if count is not None and len(rows) >= count:
                return rows
    return rows


def generate_plants(row, plant_spacing_m, start_offset_m=0.0, end_margin_m=0.0):
    """One Point every `plant_spacing_m` along the row axis."""
    dists = np.arange(start_offset_m, row.length - end_margin_m + LENGTH_EPS_M, plant_spacing_m)
    return [row.interpolate(float(s)) for s in dists]


def generate_posts(row, post_spacing_m, end_anchors=True, anchor_offset_m=1.0):
    """(posts, anchors): posts at both row ends and evenly between, at most `post_spacing_m`
    apart; anchors `anchor_offset_m` beyond the ends."""
    n = max(1, math.ceil(row.length / post_spacing_m - LENGTH_EPS_M))
    posts = [row.interpolate(float(s)) for s in np.linspace(0.0, row.length, n + 1)]
    anchors = []
    if end_anchors:
        c = np.asarray(row.coords)
        for end, nxt in ((c[0], c[1]), (c[-1], c[-2])):
            v = end - nxt
            anchors.append(Point(end + v / np.linalg.norm(v) * anchor_offset_m))
    return posts, anchors


def _feature(geom, frame, **props):
    return {"type": "Feature", "properties": props, "geometry": to_geojson(geom, frame)}


MAX_GENERATED = 200_000  # features per generator call


def _positive(params, key, default=None):
    v = params.get(key, default)
    if not _is_number(v) or v <= 0:
        raise ValueError(f"'{key}' must be a positive number of metres")
    return float(v)


def _non_negative(params, key, default=0.0):
    v = params.get(key)
    v = default if v is None or v == "" else v
    if not _is_number(v) or v < 0:
        raise ValueError(f"'{key}' must be zero or more metres")
    return float(v)


def _row_supports(idx, row_ids, kinds=None):
    """Crop and row-support objects on the given rows (what a replace removes)."""
    return [k for rid in row_ids for k in idx.derived[rid].get("children", [])
            if idx.nodes[k].category in ("crop", "row_support") and (kinds is None or idx.nodes[k].kind in kinds)]


def _target_rows(idx, ids):
    """Row ids for a mix of row and parcel ids (a parcel stands for its rows, in row order)."""
    rows = []
    for fid in ids:
        n = idx.nodes.get(fid)
        if n is None:
            raise KeyError(f"unknown or invalid feature {fid}")
        if n.layer == "row":
            rows.append(fid)
        elif n.layer == "area":
            kids = [k for k in idx.derived[fid].get("children", []) if idx.nodes[k].layer == "row"]
            rows += sorted(kids, key=lambda k: idx.derived[k]["row_index"])
        else:
            raise ValueError(f"{fid} is a {n.kind}, not a row or parcel")
    return list(dict.fromkeys(rows))


def make_rows(fc, parcel_id, params, schema=None):
    """Row features (no ids yet) for a parcel.

    params: crop, row_spacing_m, headland_m, first_offset_m, count, min_length_m, variety, and
    the heading as heading_deg, or edge (exterior edge index; default the longest), plus
    reverse (run the other way). Returns {features, updates, existing, stats}; `existing`
    is what a replace removes: the parcel's rows and the plants and supports on them.
    """
    idx = FarmIndex(fc, schema)
    parcel = idx.nodes.get(parcel_id)
    if parcel is None or parcel.layer != "area":
        raise KeyError(f"{parcel_id} is not a valid area")
    p = params
    if p.get("crop") not in idx.schema.crops:
        raise ValueError(f"unknown crop {p.get('crop')!r}")
    spacing = _positive(p, "row_spacing_m")
    headland = _non_negative(p, "headland_m")
    first = None if p.get("first_offset_m") in (None, "") else _non_negative(p, "first_offset_m")
    count = None if p.get("count") in (None, "") else int(p["count"])
    width = np.ptp(np.asarray(parcel.geom.exterior.coords), axis=0).max()
    if width / spacing > 20_000:
        raise ValueError(f"row spacing {spacing} m gives too many rows for this parcel")
    if p.get("heading_deg") not in (None, ""):
        heading = float(p["heading_deg"])
    else:
        heading = edge_heading(parcel.geom, p.get("edge"))
    if p.get("reverse"):
        heading = (heading + 180.0) % 360.0
    lines = generate_rows(parcel.geom, heading, spacing, headland, first, count, p.get("min_length_m", 1.0))
    props = {"layer": "row", "kind": p["crop"]}
    if p.get("variety"):
        props["variety"] = p["variety"]
    feats = [_feature(g, idx.frame, **props) for g in lines]
    old_rows = [k for k in idx.derived[parcel_id].get("children", []) if idx.nodes[k].layer == "row"]
    lengths = [g.length for g in lines]
    stats = {"count": len(lines), "heading_deg": round(heading, 2),
             "length_min_m": round(min(lengths), 2) if lengths else 0,
             "length_max_m": round(max(lengths), 2) if lengths else 0,
             "existing_rows": len(old_rows)}
    existing = old_rows + _row_supports(idx, old_rows)
    stats["existing_objects"] = len(existing) - len(old_rows)
    update = {"row_spacing_m": spacing, "headland_m": headland}
    return {"features": feats, "updates": {parcel_id: update}, "existing": existing, "stats": stats}


def make_plants(fc, target_ids, params, schema=None):
    """Crop features (no ids yet) along rows, or along all rows of the given parcels.

    params: plant_spacing_m, start_offset_m, end_margin_m. Returns {features, updates,
    existing, stats}; `existing` holds the plants already on those rows.
    """
    idx = FarmIndex(fc, schema)
    rows = _target_rows(idx, target_ids)
    spacing = _positive(params, "plant_spacing_m")
    start, margin = _non_negative(params, "start_offset_m"), _non_negative(params, "end_margin_m")
    if sum(idx.nodes[r].geom.length for r in rows) / spacing > MAX_GENERATED:
        raise ValueError(f"plant spacing {spacing} m gives more than {MAX_GENERATED} plants")
    feats, updates = [], {}
    for rid in rows:
        row = idx.nodes[rid]
        for g in generate_plants(row.geom, spacing, start, margin):
            feats.append(_feature(g, idx.frame, layer="object", category="crop", kind=row.kind))
        updates[rid] = {"plant_spacing_m": spacing}
    existing = _row_supports(idx, rows, {k for k in idx.schema.crops})
    stats = {"count": len(feats), "rows": len(rows), "existing_objects": len(existing)}
    return {"features": feats, "updates": updates, "existing": existing, "stats": stats}


def make_posts(fc, target_ids, params, schema=None):
    """Post and end-anchor features (no ids yet) along rows, or along all rows of parcels.

    params: post_spacing_m, end_anchors (bool), anchor_offset_m. Returns {features,
    updates, existing, stats}; `existing` holds the posts and anchors already on those rows.
    """
    idx = FarmIndex(fc, schema)
    rows = _target_rows(idx, target_ids)
    spacing = _positive(params, "post_spacing_m")
    offset = _non_negative(params, "anchor_offset_m", 1.0)
    if sum(idx.nodes[r].geom.length for r in rows) / spacing > MAX_GENERATED:
        raise ValueError(f"post spacing {spacing} m gives more than {MAX_GENERATED} posts")
    feats = []
    for rid in rows:
        posts, anchors = generate_posts(idx.nodes[rid].geom, spacing, bool(params.get("end_anchors", True)), offset)
        for kind, pts in (("post", posts), ("end_anchor", anchors)):
            feats += [_feature(g, idx.frame, layer="object", category="row_support", kind=kind) for g in pts]
    existing = _row_supports(idx, rows, {"post", "end_anchor"})
    stats = {"count": len(feats), "rows": len(rows), "existing_objects": len(existing)}
    return {"features": feats, "updates": {}, "existing": existing, "stats": stats}


def make_offset_rows(fc, row_id, params, schema=None):
    """Copies of a row, `distance_m` apart (positive: to the left of the row direction).

    params: distance_m, count. Copies keep the row's crop, variety and plant spacing.
    Returns {features, updates, existing, stats} like the other generators.
    """
    idx = FarmIndex(fc, schema)
    row = idx.nodes.get(row_id)
    if row is None or row.layer != "row":
        raise KeyError(f"{row_id} is not a valid row")
    d = params.get("distance_m")
    if not _is_number(d) or d == 0:
        raise ValueError("'distance_m' must be a non-zero number of metres")
    count = int(params.get("count") or 1)
    if not 1 <= count <= 5000:
        raise ValueError("'count' must be between 1 and 5000")
    props = {k: v for k, v in row.props.items() if k in ("layer", "kind", "variety", "plant_spacing_m")}
    feats = [_feature(row.geom.offset_curve(d * k), idx.frame, **props) for k in range(1, count + 1)]
    return {"features": feats, "updates": {}, "existing": [], "stats": {"count": count, "span_m": round(abs(d) * count, 2)}}


# --- CLI --------------------------------------------------------------------------------

def _summary(idx, fid):
    n, d = idx.nodes[fid], idx.derived[fid]
    bits = []
    if "area_m2" in d:
        bits.append(f"{d['area_m2'] / 1e4:.2f} ha" if d["area_m2"] >= 1e4 else f"{d['area_m2']:.0f} m²")
    if n.layer == "area" and n.kind == "parcel" and d.get("n_rows"):
        bits.append(f"{d['n_rows']} rows ({', '.join(d['crops'])}), heading {d['heading_deg']:.1f}°")
    if n.layer == "row":
        bits.append(f"#{d.get('row_index')} {d['n_plants']} plants, {d['length_m']:.1f} m")
    if d.get("overlays"):
        bits.append("in " + ", ".join(d["overlays"]))
    return n.label() + ("  " + "; ".join(bits) if bits else "")


def print_tree(idx, full=False, out=sys.stdout):
    def walk(fid, prefix, last):
        out.write(f"{prefix}{'└─ ' if last else '├─ '}{_summary(idx, fid)}\n")
        kids = idx.derived[fid].get("children", [])
        if not full and idx.nodes[fid].layer == "row":
            counts = Counter(idx.nodes[k].kind for k in kids)
            kids, extra = [], [f"{c} {k}" for k, c in sorted(counts.items())]
            if extra:
                out.write(f"{prefix}{'   ' if last else '│  '}└─ {', '.join(extra)}\n")
        for i, k in enumerate(kids):
            walk(k, prefix + ("   " if last else "│  "), i == len(kids) - 1)

    if idx.farm:
        out.write(_summary(idx, idx.farm.id) + "\n")
        kids = idx.derived[idx.farm.id].get("children", [])
        for i, k in enumerate(kids):
            walk(k, "", i == len(kids) - 1)
    orphans = [fid for fid, n in idx.nodes.items()
               if n.layer in ("area", "row", "object") and idx.derived[fid].get("parent") is None]
    if orphans:
        out.write(f"outside the farm: {', '.join(orphans)}\n")
    for z in idx.by_layer["overlay"]:
        d = idx.derived[z.id]
        out.write(f"overlay {z.label()}: members {', '.join(d['members']) or '-'}"
                  + (f"; roads {', '.join(d['roads'])}" if d["roads"] else "") + "\n")
    for r in idx.by_layer["navigation"]:
        out.write(f"road {r.label()}: {r.geom.length:.1f} m\n")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate", help="check the rules of FARM_GEOJSON_SPEC.md §5")
    v.add_argument("file")
    v.add_argument("--json", action="store_true", help="print issues as JSON")
    t = sub.add_parser("tree", help="print the derived hierarchy")
    t.add_argument("file")
    t.add_argument("--full", action="store_true", help="list every object instead of counts per row")
    args = ap.parse_args(argv)

    fc = load(args.file)
    issues, idx = check(fc)
    if args.cmd == "validate":
        if args.json:
            print(json.dumps([asdict(i) for i in issues], indent=1))
        else:
            for i in issues:
                print(f"{i.level:7s} {i.rule:4s} {i.id or '-':8s} {i.message}")
            n_err = sum(i.level == "error" for i in issues)
            print(f"{len(idx.nodes)} features, {n_err} errors, {len(issues) - n_err} warnings")
        return 1 if any(i.level == "error" for i in issues) else 0
    print_tree(idx, args.full)
    return 0


if __name__ == "__main__":
    sys.exit(main())
