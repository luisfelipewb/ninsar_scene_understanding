// Left panel: the guided steps of farm_schema.yaml `workflow`, each with its buttons.
import { h } from './panels.js';
import { emit, entryFor, on, state, typeOf } from './state.js';

const steps = [];  // {el, count, entries: Set}

function buttonRow(b) {
  if (b.action) {
    return h('div', { class: 'btn-row' }, h('button', { type: 'button', 'data-action': b.action,
      onclick: () => emit('action', b.action) }, b.label));
  }
  const entry = entryFor(b.layer, b.category, b.kind);
  const geometry = b.geometry || entry.geometry[0];
  let chooser = null;
  if (!b.kind && entry.kinds.length > 1) {
    chooser = h('select', { title: 'Class' }, entry.kinds.map((k) => h('option', { value: k }, k)));
  }
  const btn = h('button', {
    type: 'button',
    onclick: () => emit('preset', { button: b, entry, geometry, kind: b.kind || chooser?.value || entry.kinds[0] }),
  }, h('span', { class: 'swatch', style: `background:${entry.color}` }), b.label);
  btn.preset = { button: b, entry };
  return h('div', { class: 'btn-row' }, btn, chooser);
}

function refresh() {
  for (const s of steps) {
    s.count.textContent = String(state.fc.features.filter((f) => s.entries.has(typeOf(f.properties))).length || '');
    for (const btn of s.el.querySelectorAll('button')) {
      if (btn.preset?.button.single) {
        btn.disabled = state.fc.features.some((f) => typeOf(f.properties) === btn.preset.entry);
      }
    }
  }
}

export function setActionEnabled(action, enabled, title = '') {
  for (const btn of document.querySelectorAll(`#workflow button[data-action="${action}"]`)) {
    btn.disabled = !enabled;
    btn.title = title;
  }
}

export function initWorkflow() {
  const el = document.getElementById('workflow');
  el.replaceChildren(h('h2', {}, 'Describe the farm'));
  state.schema.workflow.forEach((step, i) => {
    const count = h('span', { class: 'count' });
    const entries = new Set(step.buttons.filter((b) => !b.action).map((b) => entryFor(b.layer, b.category, b.kind)));
    const details = h('details', { class: 'step', open: i === 0 },
      h('summary', {}, h('span', { class: 'num' }, String(i + 1)), step.step, count),
      step.hint ? h('p', { class: 'hint' }, step.hint) : null,
      step.buttons.map(buttonRow));
    el.append(details);
    steps.push({ el: details, count, entries });
  });
  on('loaded', refresh);
  on('changed', refresh);
}
