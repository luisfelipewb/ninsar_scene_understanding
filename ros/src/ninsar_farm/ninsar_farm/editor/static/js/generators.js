// Generator dialogs (docs/guides/FARM_EDITOR.md §5): rows in a parcel, plants and posts along rows.
// The server computes a preview on every parameter change; Apply commits it with new ids.
import { api } from './api.js';
import { h } from './dom.js';
import { clearPreview, nearestEdge, setClickHook, setPick, setPreview, suspendEditing, zoomTo } from './map.js';
import {
  addFeatures, byId, emit, group, labelOf, newIds, removeFeatures, select, state, updateFeatures,
} from './state.js';

let tool = null;  // {kind, el, targets, fields, result, timer, edge}

const status = (msg, level = '') => emit('status', { msg, level });
const isParcel = (f) => f?.properties?.layer === 'area' && f.properties.kind === 'parcel';
const isRow = (f) => f?.properties?.layer === 'row';

export const isToolOpen = () => !!tool;

export function closeTool() {
  if (!tool) return;
  clearTimeout(tool.timer);
  setClickHook(null);
  setPick(null);
  clearPreview();
  tool.el.remove();
  tool = null;
  suspendEditing(false);
  emit('mode', '');
}

function pickTarget(message, accept, noun) {
  return new Promise((resolve) => {
    const arm = () => setPick((id) => {
      const f = byId(id);
      if (accept(f)) { resolve(f); return; }
      status(`${f ? labelOf(f) : id} is not ${noun}`, 'warn');
      arm();
    }, `Click ${message} (Esc to cancel)`);
    arm();
  });
}

// --- form -------------------------------------------------------------------------------------

function fieldEl(f) {
  let input;
  if (f.type === 'select') {
    input = h('select', {}, f.options.map(([v, label]) => h('option', { value: v, selected: v === f.value }, label)));
  } else if (f.type === 'checkbox') {
    input = h('input', { type: 'checkbox', checked: !!f.value });
  } else {
    input = h('input', { type: f.type || 'number', step: 'any', min: f.min ?? null, value: f.value ?? '', placeholder: f.placeholder || '' });
  }
  input.addEventListener(f.type === 'text' || f.type === 'number' || !f.type ? 'input' : 'change', () => {
    f.onchange?.(read(f, input));
    schedule();
  });
  f.input = input;
  const row = h('label', { class: `field${f.type === 'checkbox' ? ' check' : ''}` },
    h('span', { class: 'key' }, f.label), input, f.unit ? h('span', { class: 'unit' }, f.unit) : h('span'));
  if (f.hidden) row.style.display = 'none';
  f.row = row;
  return row;
}

function read(f, input = f.input) {
  if (f.type === 'checkbox') return input.checked;
  if (f.type === 'select' || f.type === 'text') return input.value;
  return input.value === '' ? null : Number(input.value);
}

function params() {
  const out = {};
  for (const f of tool.fields) if (f.key && !f.row.hidden && f.row.style.display !== 'none') out[f.key] = read(f);
  return out;
}

function schedule() {
  clearTimeout(tool.timer);
  tool.timer = setTimeout(runPreview, 250);
}

// --- preview and apply ------------------------------------------------------------------------------

async function runPreview() {
  if (!tool) return;
  const t = tool;
  const body = { fc: state.fc, params: { ...params(), ...t.extraParams?.() } };
  if (t.kind === 'rows') body.parcel_id = t.targets[0].id;
  else if (t.kind === 'offset') body.row_id = t.targets[0].id;
  else body.target_ids = t.targets.map((f) => f.id);
  t.summary.textContent = 'Computing…';
  t.apply.disabled = true;
  try {
    const res = await api.generate(t.kind, body);
    if (tool !== t) return;
    t.result = res;
    t.resultParams = JSON.stringify(body.params);
    setPreview(res.features, t.edge?.latlngs);
    t.summary.replaceChildren(...t.describe(res.stats));
    t.replaceRow.style.display = res.existing.length ? '' : 'none';
    t.replaceLabel.textContent = t.describeExisting(res.stats);
    t.apply.disabled = !res.features.length;
  } catch (e) {
    if (tool !== t) return;
    t.result = null;
    clearPreview();
    t.summary.replaceChildren(h('span', { class: 'error' }, e.message));
  }
}

async function applyResult() {
  const t = tool;
  if (!t) return;
  if (!t.result || t.resultParams !== JSON.stringify({ ...params(), ...t.extraParams?.() })) {
    clearTimeout(t.timer);
    await runPreview();  // parameters changed since the last preview
    if (tool !== t || !t.result?.features.length) return;
  }
  const { features, updates, existing } = t.result;
  const replace = existing.length && t.replaceInput.checked;
  group(() => {  // one undo step
    if (replace) removeFeatures(existing.filter((id) => byId(id)));
    const prefix = { rows: 'r', plants: 'c', posts: 'i', offset: 'r' }[t.kind];
    const ids = newIds(prefix, features.length);
    features.forEach((f, i) => { f.id = ids[i]; });
    addFeatures(features);
    const changes = Object.entries(updates).filter(([id]) => byId(id))
      .map(([id, props]) => [id, { properties: { ...byId(id).properties, ...props } }]);
    if (changes.length) updateFeatures(changes, 'props');
  });
  const keep = t.targets.map((f) => f.id).filter((id) => byId(id));
  closeTool();
  select(keep);
  status(`${t.doneMessage(features.length, replace ? existing.length : 0)}`, 'ok');
}

// --- dialogs --------------------------------------------------------------------------------

function openPanel(kind, title, targets, fields, opts) {
  closeTool();
  const summary = h('div', { class: 'tool-summary muted' });
  const replaceInput = h('input', { type: 'checkbox', checked: true });
  const replaceLabel = h('span');
  const replaceRow = h('label', { class: 'tool-replace', style: 'display:none' }, replaceInput, replaceLabel);
  const apply = h('button', { type: 'button', class: 'primary', disabled: true, onclick: applyResult }, 'Apply');
  const targetText = targets.length === 1 ? labelOf(targets[0]) : `${targets.length} features`;
  const el = h('div', { class: 'tool-panel' },
    h('h3', {}, title),
    h('div', { class: 'tool-target' }, h('span', { class: 'muted' }, 'for '), targetText,
      h('button', { type: 'button', class: 'small', onclick: () => opts.restart() }, 'Change…')),
    fields.map(fieldEl),
    replaceRow, summary,
    h('div', { class: 'actions' }, apply, h('button', { type: 'button', onclick: closeTool }, 'Cancel')));
  document.getElementById('map-wrap').append(el);
  suspendEditing(true);
  zoomTo(targets.map((f) => f.id), { leftPanel: true });  // keep the targets clear of the panel
  tool = { kind, el, targets, fields, summary, apply, replaceInput, replaceLabel, replaceRow, ...opts };
  emit('mode', `${title}: adjust and Apply (Esc to cancel)`);
  runPreview();
}

const cropOptions = () => Object.keys(state.schema.crops).map((c) => [c, c]);

export async function openRows() {
  closeTool();
  let parcel = [...state.selection].map(byId).find(isParcel);
  if (!parcel) {
    const row = [...state.selection].map(byId).find(isRow);
    parcel = row && byId(state.derived[row.id]?.parent);
    if (!isParcel(parcel)) parcel = await pickTarget('the parcel to fill with rows', isParcel, 'a parcel');
  }
  const p = parcel.properties;
  const rowsNow = (state.derived[parcel.id]?.children || []).map(byId).filter(isRow);
  const crop = rowsNow[0]?.properties.kind || Object.keys(state.schema.crops)[0];
  let edge = null;
  const heading = { key: 'heading_deg', label: 'heading', unit: '°', placeholder: '0 = east, 90 = north', hidden: true };
  const fields = [
    { key: 'crop', label: 'crop', type: 'select', options: cropOptions(), value: crop },
    { key: 'variety', label: 'variety', type: 'text', value: rowsNow[0]?.properties.variety || '' },
    { label: 'direction', type: 'select', value: 'longest',
      options: [['longest', 'along the longest edge'], ['edge', 'along an edge I click'], ['typed', 'typed heading']],
      onchange: (v) => {
        heading.row.style.display = v === 'typed' ? '' : 'none';
        setClickHook(v === 'edge' ? (latlng) => {
          edge = nearestEdge(parcel, latlng);
          tool.edge = edge;
          emit('mode', 'Edge picked (cyan): click another edge, or Apply');
          schedule();
        } : null);
        if (v === 'edge') emit('mode', 'Click the parcel edge the rows run along');
        if (v !== 'edge') { edge = null; tool.edge = null; }
      } },
    heading,
    { key: 'reverse', label: 'reverse', type: 'checkbox', value: false },
    { key: 'row_spacing_m', label: 'row spacing', unit: 'm', min: 0, value: p.row_spacing_m ?? 2.5 },
    { key: 'headland_m', label: 'headland', unit: 'm', min: 0, value: p.headland_m ?? 4 },
    { key: 'first_offset_m', label: 'first row offset', unit: 'm', min: 0, placeholder: 'half a spacing' },
    { key: 'count', label: 'max rows', type: 'number', min: 1, placeholder: 'fill the parcel' },
  ];
  openPanel('rows', 'Generate rows', [parcel], fields, {
    restart: () => { closeTool(); state.selection.clear(); openRows(); },
    extraParams: () => (edge ? { edge: edge.index } : {}),
    describe: (s) => [h('b', {}, `${s.count} rows`), ` heading ${s.heading_deg}°, ${s.length_min_m}–${s.length_max_m} m long`],
    describeExisting: (s) => `Replace the ${s.existing_rows} existing rows${s.existing_objects ? ` and the ${s.existing_objects} plants/posts on them` : ''}`,
    doneMessage: (n, removed) => `Added ${n} rows${removed ? `, removed ${removed} old features` : ''}. Next: Fill rows with plants…`,
  });
}

function rowTargets() {
  return [...state.selection].map(byId).filter((f) => isRow(f) || isParcel(f));
}

async function openAlongRows(kind) {
  closeTool();
  let targets = rowTargets();
  if (!targets.length) targets = [await pickTarget(`the row or parcel to add ${kind} to`, (f) => isRow(f) || isParcel(f), 'a row or parcel')];
  const firstRow = targets.find(isRow) || (state.derived[targets[0].id]?.children || []).map(byId).find(isRow);
  const fields = kind === 'plants' ? [
    { key: 'plant_spacing_m', label: 'plant spacing', unit: 'm', min: 0, value: firstRow?.properties.plant_spacing_m ?? 1.0 },
    { key: 'start_offset_m', label: 'start offset', unit: 'm', min: 0, value: 0 },
    { key: 'end_margin_m', label: 'end margin', unit: 'm', min: 0, value: 0 },
  ] : [
    { key: 'post_spacing_m', label: 'post spacing', unit: 'm', min: 0, value: 5 },
    { key: 'end_anchors', label: 'end anchors', type: 'checkbox', value: true },
    { key: 'anchor_offset_m', label: 'anchor offset', unit: 'm', min: 0, value: 1.0 },
  ];
  const what = kind === 'plants' ? 'plants' : 'posts';
  openPanel(kind, kind === 'plants' ? 'Fill rows with plants' : 'Add posts', targets, fields, {
    restart: () => { closeTool(); state.selection.clear(); openAlongRows(kind); },
    describe: (s) => [h('b', {}, `${s.count} ${what}`), ` on ${s.rows} row${s.rows === 1 ? '' : 's'}`],
    describeExisting: (s) => `Replace the ${s.existing_objects} existing ${what}`,
    doneMessage: (n, removed) => `Added ${n} ${what}${removed ? `, removed ${removed} old ones` : ''}`,
  });
}

export const openPlants = () => openAlongRows('plants');
export const openPosts = () => openAlongRows('posts');

export async function openOffset() {
  closeTool();
  let row = [...state.selection].map(byId).find(isRow);
  if (!row) row = await pickTarget('the row to copy', isRow, 'a row');
  const parcel = byId(state.derived[row.id]?.parent);
  const fields = [
    { key: 'distance_m', label: 'distance', unit: 'm', value: parcel?.properties.row_spacing_m ?? 2.5,
      placeholder: '+ left, − right of the row direction' },
    { key: 'count', label: 'copies', type: 'number', min: 1, value: 1 },
  ];
  openPanel('offset', 'Offset row', [row], fields, {
    restart: () => { closeTool(); state.selection.clear(); openOffset(); },
    describe: (s) => [h('b', {}, `${s.count} ${s.count === 1 ? 'copy' : 'copies'}`), ` spanning ${s.span_m} m (positive: left of the row direction)`],
    describeExisting: () => '',
    doneMessage: (n) => `Added ${n} parallel row${n === 1 ? '' : 's'}`,
  });
}
