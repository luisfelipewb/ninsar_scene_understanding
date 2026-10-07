// Leaflet map: one canvas renderer for every feature (fast with thousands of plants),
// one toggleable group per catalog type, Geoman for drawing and vertex editing.
/* global L */
import { byId, colorOf, emit, LAYER_ORDER, on, state, typeOf, updateFeature } from './state.js';

let map;
let renderer;
const layers = new Map();  // feature id -> Leaflet layer
const groups = new Map();  // type key -> L.LayerGroup
let control;
let editing = null;        // {id, layer, handle?, onEdit?}
let drawCallback = null;
let pickCallback = null;
let clickHook = null;      // gets the latlng of every map click (generator edge picking)
let preview = null;        // L.LayerGroup with generator previews
let editingSuspended = false;  // no vertex editing while a generator dialog is open

const SNAP = { snappable: true, snapDistance: 12, allowSelfIntersection: false };
const GEOM_RANK = { Polygon: 0, MultiPolygon: 0, LineString: 1, MultiLineString: 1, Point: 2, MultiPoint: 2 };

// --- styles ---------------------------------------------------------------------------

function baseStyle(f) {
  const p = f.properties || {};
  const color = colorOf(p);
  const t = typeOf(p);
  if (!t) return { color: '#888888', weight: 2, dashArray: '2 4', fillOpacity: 0.1, radius: 5 };
  if (f.geometry.type === 'Point') {
    const radius = p.category === 'crop' ? 3 : p.category === 'row_support' ? 2.5 : 5;
    return { radius, color: '#222222', weight: 1, fillColor: color, fillOpacity: 0.95 };
  }
  switch (p.layer) {
    case 'farm': return { color: '#000000', weight: 3, fill: false };
    case 'overlay': return { color, weight: 2, dashArray: '6 4', fill: false };
    case 'area': return { color, weight: 2, fillColor: color, fillOpacity: 0.18 };
    case 'row': return { color, weight: 2 };
    case 'navigation': return { color, weight: 5, opacity: 0.85 };
    default: return { color, weight: 2, fillColor: color, fillOpacity: 0.45 };
  }
}

const ISSUE_COLOR = { error: '#e8202a', warning: '#ff9800' };

function styleFor(f) {
  const s = { dashArray: null, lineCap: 'round', ...baseStyle(f) };  // setStyle merges: reset what other states set
  if (!state.selection.has(f.id)) {
    const issue = state.issuesById.get(f.id);
    if (!issue) return s;
    const color = ISSUE_COLOR[issue.level];
    return f.geometry.type === 'Point'
      ? { ...s, color, weight: 2.5, radius: s.radius + 1.5 }
      : { ...s, color, weight: (s.weight || 2) + 2, dashArray: '1 6', lineCap: 'round' };  // dotted: unlike any type style
  }
  return f.geometry.type === 'Point'
    ? { ...s, color: '#ffd700', weight: 3, radius: s.radius + 2 }
    : { ...s, color: '#ffd700', weight: (s.weight || 2) + 2 };
}

const rank = (f) => LAYER_ORDER.indexOf(f.properties?.layer ?? 'object') * 3 + (GEOM_RANK[f.geometry.type] ?? 2);

// --- layers ---------------------------------------------------------------------------

const toLatLng = (c) => [c[1], c[0]];

function makeLayer(f) {
  const g = f.geometry;
  const style = { ...styleFor(f), renderer };
  let layer;
  if (g.type === 'Point') {
    layer = L.circleMarker(toLatLng(g.coordinates), { ...style, pmIgnore: true, snapIgnore: true });
  } else if (g.type === 'LineString') {
    layer = L.polyline(g.coordinates.map(toLatLng), style);
  } else if (g.type === 'Polygon') {
    layer = L.polygon(g.coordinates.map((ring) => ring.map(toLatLng)), style);
  } else {  // Multi* from an import, until it is classified
    layer = L.geoJSON(g, { style, pmIgnore: true, pointToLayer: (_, ll) => L.circleMarker(ll, style) });
  }
  layer.featureId = f.id;
  layer.on('click', (e) => {
    if (drawCallback) return;  // let the click reach the map: Geoman places vertices on map clicks
    L.DomEvent.stopPropagation(e);
    if (clickHook) { clickHook(e.latlng); return; }
    const id = layer.featureId;
    if (pickCallback) { const cb = pickCallback; setPick(null); cb(id); return; }
    emit('featureClick', { id, shift: e.originalEvent.shiftKey });
  });
  if (f.properties?.layer === 'overlay' && g.type === 'Polygon') {  // light fill that does not catch clicks
    layer.tint = L.polygon(layer.getLatLngs(), {
      renderer, stroke: false, fillColor: style.color, fillOpacity: 0.08, interactive: false,
      pmIgnore: true, snapIgnore: true,
    });
  }
  return layer;
}

function groupKey(f) {
  const t = typeOf(f.properties);
  return t ? state.schema.types.indexOf(t) : 'unclassified';
}

function addLayer(f) {
  const layer = makeLayer(f);
  layers.set(f.id, layer);
  const group = groups.get(groupKey(f));
  if (layer.tint) group.addLayer(layer.tint);
  group.addLayer(layer);
}

function removeLayer(id) {
  const layer = layers.get(id);
  if (!layer) return;
  if (editing?.id === id) stopEditing();
  groups.forEach((g) => { g.removeLayer(layer); if (layer.tint) g.removeLayer(layer.tint); });
  layers.delete(id);
}

// canvas draws (and hit-tests) in insertion order: keep points on top of rows on top of areas
function reorder() {
  const sorted = [...state.fc.features].sort((a, b) => rank(a) - rank(b));
  for (const f of sorted) {
    const layer = layers.get(f.id);
    if (layer && map.hasLayer(layer) && layer.bringToFront) {
      layer.tint?.bringToFront();
      layer.bringToFront();
    }
  }
}

function makeGroups() {
  groups.forEach((g) => map.removeLayer(g));
  groups.clear();
  if (control) map.removeControl(control);
  const overlays = {};
  state.schema.types.forEach((t, i) => {
    const g = L.layerGroup().addTo(map);
    groups.set(i, g);
    if (!t.hidden) overlays[`<span class="swatch" style="background:${t.color}"></span>${t.label}`] = g;
  });
  const unclassified = L.layerGroup().addTo(map);
  groups.set('unclassified', unclassified);
  overlays['<span class="swatch" style="background:#888"></span>Unclassified'] = unclassified;
  control = L.control.layers(baseLayers, overlays, { collapsed: true, position: 'topright' }).addTo(map);
}

export function render() {
  stopEditing();
  layers.clear();
  makeGroups();
  const sorted = [...state.fc.features].sort((a, b) => rank(a) - rank(b));
  sorted.forEach(addLayer);
}

function restyle(id) {
  const f = byId(id);
  const layer = layers.get(id);
  if (!f || !layer) return;
  const s = styleFor(f);
  layer.setStyle?.(s);
  if (s.radius && layer.setRadius) layer.setRadius(s.radius);
}

// --- selection and editing --------------------------------------------------------------

function stopEditing() {
  if (!editing) return;
  const { layer, handle, onEdit } = editing;
  if (handle) map.removeLayer(handle);
  if (onEdit) { layer.off('pm:edit', onEdit); layer.pm?.disable(); }
  editing = null;
}

function startEditing(id) {
  const layer = layers.get(id);
  if (!layer || !map.hasLayer(layer)) return;
  if (layer instanceof L.CircleMarker) {
    const handle = L.marker(layer.getLatLng(), {
      draggable: true, pmIgnore: true, keyboard: false,
      icon: L.divIcon({ className: 'drag-handle', iconSize: [16, 16] }),
    }).addTo(map);
    handle.on('drag', () => layer.setLatLng(handle.getLatLng()));
    handle.on('dragend', () => updateFeature(id, { geometry: layer.toGeoJSON(false).geometry }, 'map'));
    editing = { id, layer, handle };
  } else if (layer.pm) {
    const onEdit = () => {
      layer.tint?.setLatLngs(layer.getLatLngs());
      updateFeature(layer.featureId, { geometry: layer.toGeoJSON(false).geometry }, 'map');
    };
    layer.pm.enable(SNAP);
    layer.on('pm:edit', onEdit);
    editing = { id, layer, onEdit };
  }
}

let previous = new Set();
on('selection', () => {
  stopEditing();
  for (const id of new Set([...previous, ...state.selection])) restyle(id);
  previous = new Set(state.selection);
  if (state.selection.size === 1 && !drawCallback && !editingSuspended) startEditing([...state.selection][0]);
});

export function suspendEditing(suspend) {
  editingSuspended = suspend;
  if (suspend) stopEditing();
  else if (state.selection.size === 1) startEditing([...state.selection][0]);
}

on('changed', ({ added = [], updated = [], removed = [], renamed = {}, source }) => {
  removed.forEach(removeLayer);
  for (const [from, to] of Object.entries(renamed)) {
    const layer = layers.get(from);
    layers.delete(from);
    if (layer) { layer.featureId = to; layers.set(to, layer); }
    if (editing?.id === from) editing.id = to;
  }
  let needOrder = false;
  for (const id of added) {
    addLayer(byId(id));
    needOrder ||= byId(id).geometry.type !== 'Point';
  }
  for (const id of updated) {
    if (source === 'map' || source === 'props') { restyle(id); continue; }
    const wasEditing = editing?.id === id;
    removeLayer(id);
    addLayer(byId(id));
    needOrder = true;
    if (wasEditing) startEditing(id);
  }
  if (needOrder) reorder();
});

on('loaded', render);
on('report', ({ touched }) => touched.forEach(restyle));

// --- drawing and picking ----------------------------------------------------------------

const SHAPES = { Point: 'CircleMarker', LineString: 'Line', Polygon: 'Polygon' };

export function startDraw(geometry, { continueDrawing = false, color = '#3388ff' } = {}, onCreate) {
  stopDraw();
  stopEditing();
  drawCallback = onCreate;
  map.pm.enableDraw(SHAPES[geometry], {
    ...SNAP, continueDrawing,
    templineStyle: { color }, hintlineStyle: { color, dashArray: [5, 5] },
    pathOptions: { color, fillColor: color, radius: 5 },
  });
  emit('mode', `Drawing ${geometry === 'Point' ? 'points' : geometry === 'LineString' ? 'a line' : 'a polygon'}`
    + (geometry === 'Point' ? ': click to place, Esc to stop' : ': click the last point again to finish, Esc to cancel'));
}

export function stopDraw() {
  if (map.pm.globalDrawModeEnabled()) map.pm.disableDraw();
  drawCallback = null;
  emit('mode', '');
}

export function setPick(callback, message = 'Click a feature on the map (Esc to cancel)') {
  pickCallback = callback;
  map.getContainer().classList.toggle('picking', !!callback);
  emit('mode', callback ? message : '');
}

export const isBusy = () => !!drawCallback || !!pickCallback || !!clickHook;

export function setClickHook(callback) {
  clickHook = callback;
  map.getContainer().classList.toggle('picking', !!callback);
}

// --- generator preview ------------------------------------------------------------------

const PREVIEW = '#ff6d00';

export function setPreview(features = [], highlight = null) {
  preview ||= L.layerGroup().addTo(map);
  preview.clearLayers();
  const opts = { renderer, interactive: false, pmIgnore: true, snapIgnore: true };
  preview.addLayer(L.geoJSON({ type: 'FeatureCollection', features }, {
    ...opts,
    style: () => ({ ...opts, color: PREVIEW, weight: 2, dashArray: '6 4' }),
    pointToLayer: (_, ll) => L.circleMarker(ll, { ...opts, radius: 3, color: PREVIEW, weight: 1.5, fill: false }),
  }));
  if (highlight) preview.addLayer(L.polyline(highlight, { ...opts, color: '#00e5ff', weight: 6, opacity: 0.8 }));
}

export function clearPreview() { preview?.clearLayers(); }

// index of the exterior edge of a polygon feature nearest to `latlng`, in screen space
export function nearestEdge(f, latlng) {
  const ring = f.geometry.coordinates[0].map((c) => map.latLngToLayerPoint([c[1], c[0]]));
  const p = map.latLngToLayerPoint(latlng);
  let best = 0;
  let bestDist = Infinity;
  for (let i = 0; i < ring.length - 1; i += 1) {
    const d = L.LineUtil.pointToSegmentDistance(p, ring[i], ring[i + 1]);
    if (d < bestDist) { bestDist = d; best = i; }
  }
  const c = f.geometry.coordinates[0];
  return { index: best, latlngs: [[c[best][1], c[best][0]], [c[best + 1][1], c[best + 1][0]]] };
}
export const getMap = () => map;

// --- view ---------------------------------------------------------------------------------

export function zoomTo(ids, { leftPanel = false } = {}) {
  const bounds = L.latLngBounds([]);
  for (const id of ids) {
    const layer = layers.get(id);
    if (!layer) continue;
    bounds.extend(layer.getBounds ? layer.getBounds() : layer.getLatLng());
  }
  if (!bounds.isValid()) return;
  if (leftPanel) map.fitBounds(bounds.pad(0.05), { maxZoom: 21, paddingTopLeft: [390, 30], paddingBottomRight: [30, 30] });
  else map.fitBounds(bounds.pad(0.3), { maxZoom: 21 });
}

export function fitAll() {
  if (state.fc.features.length) zoomTo(state.fc.features.map((f) => f.id));
  else if (state.fc.origin) map.setView([state.fc.origin.lat, state.fc.origin.lon], 17);
}

export function goTo(lat, lon) { map.setView([lat, lon], Math.max(map.getZoom(), 17)); }

let baseLayers;

export function initMap(el) {
  renderer = L.canvas({ tolerance: 5 });
  map = L.map(el, { renderer, preferCanvas: true, maxZoom: 23, zoomControl: true, boxZoom: false })
    .setView([46.5, 2.5], 5);
  baseLayers = {
    Satellite: L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxNativeZoom: 19, maxZoom: 23, attribution: 'Imagery © Esri, Maxar, Earthstar Geographics',
    }),
    OpenStreetMap: L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxNativeZoom: 19, maxZoom: 23, attribution: '© OpenStreetMap contributors',
    }),
    'None (offline)': L.layerGroup(),
  };
  baseLayers.Satellite.addTo(map);
  L.control.scale({ metric: true, imperial: false, maxWidth: 150 }).addTo(map);
  map.pm.setGlobalOptions(SNAP);

  map.on('pm:create', (e) => {
    const geometry = e.layer.toGeoJSON(false).geometry;
    map.removeLayer(e.layer);
    drawCallback?.(geometry);
  });
  map.on('pm:drawend', () => {
    drawCallback = null;
    emit('mode', '');
    emit('drawEnd');
    if (state.selection.size === 1 && !editingSuspended) startEditing([...state.selection][0]);  // the shape just drawn
  });
  map.on('click', (e) => {
    if (clickHook) clickHook(e.latlng);
    else if (!isBusy()) emit('mapClick');
  });
  map.on('overlayadd', reorder);
  map.on('mousemove', (e) => emit('cursor', e.latlng));
  return map;
}
