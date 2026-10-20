// Right-hand panels: properties of the selection (form built from the catalog) and the
// issue list from the last validation.
import { h } from './dom.js';
import { frameOf, headingDeg } from './geo.js';
import { setPick, zoomTo } from './map.js';
import {
  byId, COMMON_KEYS, emit, labelOf, newId, on, removeFeatures, select, state, typeOf, updateFeature, updateFeatures,
} from './state.js';
import { renderTree } from './tree.js';

const fmt = (v) => (Array.isArray(v) ? v.join(', ') : typeof v === 'number' ? String(Math.round(v * 100) / 100) : String(v));
const MIXED = '— mixed —';

// --- properties ---------------------------------------------------------------------------

function selected() { return [...state.selection].map(byId).filter(Boolean); }

function setProp(features, key, value) {
  updateFeatures(features.map((f) => {
    const properties = { ...f.properties };
    if (value === '' || value == null) delete properties[key];
    else properties[key] = value;
    return [f.id, { properties }];
  }), 'props');
}

function shared(features, key) {
  const values = new Set(features.map((f) => JSON.stringify(f.properties?.[key] ?? null)));
  return values.size === 1 ? JSON.parse([...values][0]) : MIXED;
}

function parseValue(attr, raw) {
  if (raw === '') return '';
  if (attr.type === 'float') return Number.isFinite(Number(raw)) ? Number(raw) : raw;
  if (attr.type === 'int') return Number.isInteger(Number(raw)) ? Number(raw) : raw;
  return raw;
}

function field(label, input, { unit, missing, note } = {}) {
  return h('label', { class: `field${missing ? ' missing' : ''}` },
    h('span', { class: 'key' }, label), input, unit ? h('span', { class: 'unit' }, unit) : null,
    note ? h('span', { class: 'note' }, note) : null);
}

function attrInput(features, attr) {
  const value = shared(features, attr.name);
  const shown = value === MIXED ? '' : (value ?? '');
  const onchange = (e) => setProp(features, attr.name, parseValue(attr, e.target.value.trim()));
  if (attr.type === 'enum') {
    return h('select', { onchange },
      h('option', { value: '' }, value === MIXED ? MIXED : ''),
      attr.values.map((v) => h('option', { value: v, selected: v === shown }, v)));
  }
  if (attr.type === 'uid_ref') {
    const candidates = state.fc.features.filter((f) => !attr.ref_kind || attr.ref_kind.includes(f.properties?.kind));
    const sel = h('select', { onchange },
      h('option', { value: '' }, value === MIXED ? MIXED : ''),
      candidates.map((f) => h('option', { value: f.id, selected: f.id === shown }, labelOf(f))));
    if (shown && !candidates.some((f) => f.id === shown)) sel.append(h('option', { value: shown, selected: true }, `${shown} (missing)`));
    const pick = h('button', {
      type: 'button', class: 'small', title: 'Pick on map',
      onclick: () => setPick((id) => {
        const kind = byId(id)?.properties?.kind;
        if (attr.ref_kind && !attr.ref_kind.includes(kind)) {
          emit('status', { msg: `${id} is a ${kind}, not a ${attr.ref_kind.join(' / ')}`, level: 'warn' });
          return;
        }
        setProp(features, attr.name, id);
        renderProps();
      }, `Click the ${(attr.ref_kind || ['feature']).join(' / ')} to link (Esc to cancel)`),
    }, 'Pick');
    return h('span', { class: 'ref' }, sel, pick);
  }
  const numeric = attr.type === 'float' || attr.type === 'int';
  return h('input', {
    type: numeric ? 'number' : 'text', step: numeric ? 'any' : null, min: attr.unit === 'm' ? '0' : null,
    value: shown, placeholder: value === MIXED ? MIXED : (attr.default ?? ''), onchange,
  });
}

function derivedBlock(f, t) {
  const d = state.derived[f.id];
  if (!d) return h('p', { class: 'muted' }, 'Validate to see derived values (parent, counts, overlays).');
  const keys = ['parent', 'overlays', ...(t?.derived || [])];
  const rows = keys.filter((k) => d[k] != null && !(Array.isArray(d[k]) && !d[k].length)).map((k) => {
    const v = d[k];
    const ids = k === 'parent' ? [v] : (k === 'overlays' || k === 'members') ? v : null;
    const value = ids
      ? ids.slice(0, 30).map((id) => h('a', { href: '#', onclick: (e) => { e.preventDefault(); select([id]); zoomTo([id]); } }, id))
        .flatMap((a, i) => (i ? [', ', a] : [a])).concat(ids.length > 30 ? [` … ${ids.length - 30} more`] : [])
      : [fmt(v)];
    return h('div', { class: 'derived-row' }, h('span', { class: 'key' }, k), h('span', {}, ...value));
  });
  return h('div', { class: `derived${state.derivedStale ? ' stale' : ''}` },
    h('h4', {}, 'Derived', state.derivedStale ? h('span', { class: 'muted' }, ' (out of date, validate again)') : null),
    rows.length ? rows : h('p', { class: 'muted' }, 'nothing derived'));
}

// --- coordinates: typed positions in the local frame (x/y, metres) or in lon/lat ------------

let coordMode = 'enu';
try { coordMode = localStorage.getItem('farmEditor.coords') || 'enu'; } catch { /* storage blocked */ }

function vertices(g) {
  if (g.type === 'Point') return [[g.coordinates], (v) => ({ type: 'Point', coordinates: v[0] })];
  if (g.type === 'LineString') return [g.coordinates, (v) => ({ type: 'LineString', coordinates: v })];
  if (g.type === 'Polygon') {
    return [g.coordinates[0].slice(0, -1),
      (v) => ({ type: 'Polygon', coordinates: [[...v, v[0]], ...g.coordinates.slice(1)] })];
  }
  return [null, null];
}

function coordsBlock(f) {
  const frame = frameOf(state.fc);
  const mode = frame ? coordMode : 'lonlat';
  const [verts, write] = vertices(f.geometry);
  if (!verts) return null;
  const head = h('h4', {}, 'Coordinates ', frame ? h('select', {
    class: 'small-select',
    onchange: (e) => {
      coordMode = e.target.value;
      try { localStorage.setItem('farmEditor.coords', coordMode); } catch { /* storage blocked */ }
      renderProps();
    },
  }, h('option', { value: 'enu', selected: mode === 'enu' }, 'x / y (m)'), h('option', { value: 'lonlat', selected: mode === 'lonlat' }, 'lon / lat'))
    : h('span', { class: 'muted' }, '(x/y once the farm boundary sets the origin)'));
  const out = h('div', { class: 'coords' }, head);
  if (f.geometry.type === 'LineString' && frame) {
    const a = frame.enu(verts[0]); const b = frame.enu(verts[verts.length - 1]);
    let len = 0;
    for (let i = 1; i < verts.length; i += 1) { const p = frame.enu(verts[i - 1]); const q = frame.enu(verts[i]); len += Math.hypot(q[0] - p[0], q[1] - p[1]); }
    out.append(h('p', { class: 'muted' }, `length ${len.toFixed(3)} m, heading ${headingDeg(b[0] - a[0], b[1] - a[1]).toFixed(2)}°`));
  }
  if (verts.length > 60) {
    out.append(h('p', { class: 'muted' }, `${verts.length} vertices: edit them on the map.`));
    return out;
  }
  const shown = (c) => (mode === 'enu' ? frame.enu(c).map((v) => v.toFixed(3)) : [c[0].toFixed(8), c[1].toFixed(8)]);
  verts.forEach((c, i) => {
    const [u, v] = shown(c);
    const iu = h('input', { type: 'number', step: 'any', value: u, title: mode === 'enu' ? 'x (east), m' : 'longitude' });
    const iv = h('input', { type: 'number', step: 'any', value: v, title: mode === 'enu' ? 'y (north), m' : 'latitude' });
    const onchange = () => {
      const pair = [Number(iu.value), Number(iv.value)];
      if (!pair.every(Number.isFinite)) return;
      const next = verts.map((x) => [...x]);
      next[i] = mode === 'enu' ? frame.wgs84(pair) : pair;
      updateFeature(f.id, { geometry: write(next) });
    };
    iu.onchange = onchange;
    iv.onchange = onchange;
    out.append(h('div', { class: 'coord-row' }, h('span', { class: 'muted' }, verts.length > 1 ? String(i) : ''), iu, iv));
  });
  return out;
}

function customBlock(f, t) {
  const known = new Set([...COMMON_KEYS, ...(t?.attributes || []).map((a) => a.name)]);
  const keys = Object.keys(f.properties || {}).filter((k) => !known.has(k));
  const keyInput = h('input', { type: 'text', placeholder: 'new key' });
  const valInput = h('input', { type: 'text', placeholder: 'value' });
  const parse = (s) => { try { return JSON.parse(s); } catch { return s; } };
  return h('div', { class: 'custom' },
    h('h4', {}, 'Custom properties'),
    keys.map((k) => h('div', { class: 'field' },
      h('span', { class: 'key' }, k),
      h('input', { type: 'text', value: typeof f.properties[k] === 'string' ? f.properties[k] : JSON.stringify(f.properties[k]),
        onchange: (e) => setProp([f], k, parse(e.target.value)) }),
      h('button', { type: 'button', class: 'small', title: 'Remove', onclick: () => setProp([f], k, null) }, '×'))),
    h('div', { class: 'field' }, keyInput, valInput,
      h('button', { type: 'button', class: 'small', onclick: () => {
        const k = keyInput.value.trim();
        if (k && !known.has(k)) setProp([f], k, parse(valInput.value));
      } }, 'Add')));
}

function classifyBlock(f) {
  const options = [];
  state.schema.types.forEach((t, i) => {
    if (!t.geometry.includes(f.geometry.type)) return;
    t.kinds.forEach((k) => options.push(h('option', { value: `${i}|${k}` }, t.kinds.length > 1 ? `${t.label}: ${k}` : t.label)));
  });
  const multi = f.geometry.type.startsWith('Multi');
  return h('div', { class: 'classify' },
    h('p', {}, multi ? `A ${f.geometry.type} is not allowed: split it before importing.`
      : 'This feature has no farm type yet. Choose what it is:'),
    multi ? null : h('select', {
      onchange: (e) => {
        const [i, kind] = e.target.value.split('|');
        const t = state.schema.types[Number(i)];
        const props = { ...f.properties, layer: t.layer, kind };
        if (t.category) props.category = t.category; else delete props.category;
        updateFeatures([[f.id, { properties: props, id: newId(t.prefix) }]]);
      },
    }, h('option', { value: '' }, 'Classify as…'), options));
}

function renderProps() {
  const el = document.getElementById('props');
  const features = selected();
  el.replaceChildren();
  if (!features.length) {
    el.append(h('h3', {}, 'Properties'), h('p', { class: 'muted' },
      'Pick a step on the left to draw, or click a feature to edit it. Shift-click adds to the selection.'));
    return;
  }
  const types = new Set(features.map((f) => typeOf(f.properties)));
  const t = types.size === 1 ? [...types][0] : null;
  const f = features[0];
  const single = features.length === 1;
  el.append(h('h3', {}, single ? labelOf(f) : `${features.length} features`,
    h('span', { class: 'type' }, t ? t.label : types.size > 1 ? 'mixed types' : 'unclassified')));

  if (single && !t) el.append(classifyBlock(f));
  if (t) {
    const kind = shared(features, 'kind');
    if (t.kinds.length > 1) {
      el.append(field('class', h('select', {
        onchange: (e) => { if (e.target.value) setProp(features, 'kind', e.target.value); },
      }, kind === MIXED ? h('option', { value: '' }, MIXED) : null,
      t.kinds.map((k) => h('option', { value: k, selected: k === kind }, k)))));
    }
    if (single) {
      const required = t.attributes.some((a) => a.name === 'name' && a.required);
      el.append(field('name', h('input', { type: 'text', value: f.properties.name ?? '',
        onchange: (e) => setProp(features, 'name', e.target.value.trim()) }), { missing: required && !f.properties.name }));
    }
    el.append(field('description', h('textarea', { rows: 2, onchange: (e) => setProp(features, 'description', e.target.value.trim()) },
      shared(features, 'description') === MIXED ? '' : (shared(features, 'description') ?? ''))));
    for (const attr of t.attributes) {
      if (attr.name === 'name') continue;
      const value = shared(features, attr.name);
      el.append(field(attr.name, attrInput(features, attr), {
        unit: attr.unit, missing: attr.required && (value == null || value === ''),
      }));
    }
  }
  if (single) {
    const coords = coordsBlock(f);
    if (coords) el.append(coords);
    if (t) el.append(derivedBlock(f, t));
    el.append(customBlock(f, t));
    const mine = state.issues.filter((i) => i.id === f.id);
    if (mine.length) {
      el.append(h('div', { class: 'own-issues' }, mine.map((i) => h('div', { class: `issue ${i.level}` },
        h('b', {}, i.rule), ' ', i.message))));
    }
  }
  el.append(h('div', { class: 'actions' },
    h('button', { type: 'button', onclick: () => zoomTo(features.map((x) => x.id)) }, 'Zoom to'),
    h('button', { type: 'button', class: 'danger', onclick: () => deleteSelection() }, `Delete${single ? '' : ` ${features.length}`}`)));
}

export function deleteSelection() {
  const ids = [...state.selection];
  if (!ids.length) return;
  if (ids.length > 10 && !window.confirm(`Delete ${ids.length} features?`)) return;
  removeFeatures(ids);
}

// --- issues / tree tabs ------------------------------------------------------------------

let tab = 'issues';

export function showTab(name) {
  tab = name;
  renderReport();
}

function issueList() {
  const list = h('div', { class: 'issue-list' });
  for (const i of state.issues.slice(0, 500)) {
    list.append(h('div', {
      class: `issue ${i.level}`,
      onclick: () => { if (i.id && byId(i.id)) { select([i.id]); zoomTo([i.id]); } },
    }, h('b', {}, i.rule), ' ', h('span', { class: 'id' }, i.id || '—'), ' ', i.message));
  }
  if (state.issues.length > 500) list.append(h('div', { class: 'muted' }, `… ${state.issues.length - 500} more`));
  if (!state.issues.length && !state.derivedStale) list.append(h('p', { class: 'ok' }, 'No issues: the farm is ready for the scene graph.'));
  return list;
}

function renderReport() {
  const el = document.getElementById('issues');
  el.replaceChildren();
  const errors = state.issues.filter((i) => i.level === 'error').length;
  const warnings = state.issues.length - errors;
  const validated = Object.keys(state.derived).length || state.issues.length;
  const tabButton = (name, label) => h('button', { type: 'button', class: `tab${tab === name ? ' active' : ''}`,
    onclick: () => showTab(name) }, label);
  el.append(h('div', { class: 'tabs' },
    tabButton('issues', `Issues${validated ? ` ${errors} ✕ ${warnings} !` : ''}`),
    tabButton('tree', 'Tree'),
    h('label', { class: 'live', title: 'Validate automatically 1 s after each change' },
      h('input', { type: 'checkbox', checked: state.live, onchange: (e) => emit('live', e.target.checked) }), 'live')));
  el.append(h('p', { class: `report-state${state.derivedStale ? ' stale' : ''}` },
    !validated ? 'Not validated yet.' : state.derivedStale ? 'Out of date: validate again.' : 'Up to date.'));
  const body = h('div', { class: 'tab-body' });
  if (tab === 'tree') renderTree(body); else body.append(issueList());
  el.append(body);
}

export function initPanels() {
  on('selection', renderProps);
  on('report', () => { renderProps(); renderReport(); });
  on('loaded', () => { renderProps(); renderReport(); });
  on('changed', ({ source }) => {
    if (source !== 'props') renderProps();  // a property edit keeps the form (and its focus) as it is
    emit('stale');
  });
  on('stale', () => {
    const p = document.querySelector('#issues .report-state');
    if (p && Object.keys(state.derived).length) { p.textContent = 'Out of date: validate again.'; p.classList.add('stale'); }
  });
}
