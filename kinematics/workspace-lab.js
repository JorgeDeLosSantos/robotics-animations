import * as THREE from 'three';
import { RobotViewer } from './shared-robot-viewer.js';
import { generateWorkspaceSamples, resolveSamplingRanges } from './workspace-core.js';
import { fitOrthographicObjects } from '../shared/camera-fit.js';

const presets = {
  planar2r: {
    description: 'Two coplanar revolute joints: a useful analytical reference.',
    params: [
      { type: 'R', theta: 30, d: 0, a: 220, alpha: 0, limits: { min: -180, max: 180 } },
      { type: 'R', theta: -45, d: 0, a: 160, alpha: 0, limits: { min: -150, max: 150 } }
    ]
  },
  spatial3r: {
    description: 'Three revolute joints with spatial positioning.',
    params: [
      { type: 'R', theta: 0, d: 200, a: 0, alpha: 90, limits: { min: -170, max: 170 } },
      { type: 'R', theta: 20, d: 0, a: 200, alpha: 0, limits: { min: -120, max: 120 } },
      { type: 'R', theta: -30, d: 0, a: 200, alpha: 0, limits: { min: -150, max: 150 } }
    ]
  },
  mixedRRP: {
    description: 'A spatial RRP chain: the prism extends the reachable region.',
    params: [
      { type: 'R', theta: 0, d: 140, a: 0, alpha: 90, limits: { min: -170, max: 170 } },
      { type: 'R', theta: 30, d: 0, a: 170, alpha: 0, limits: { min: -150, max: 150 } },
      { type: 'P', theta: 0, d: 130, a: 0, alpha: 0, limits: { min: 0, max: 260 } }
    ]
  }
};
const copyParams = params => params.map(joint => ({
  ...joint,
  limits: joint.limits ? { ...joint.limits } : null
}));
const el = id => document.getElementById(id);
const viewer = new RobotViewer({
  initialDHParams: copyParams(presets.planar2r.params),
  canvasContainerId: 'canvas-container',
  tableBodyId: 'dh-table-body',
  addRowBtnId: 'add-row-btn',
  matrixContainerId: 'unused-matrix-display',
  frustumSize: 1100,
  maxLinks: 6
});
let activePreset = 'planar2r';
let baseline = copyParams(presets[activePreset].params);
let perJointDomain = new WeakMap();
let knownJointType = new WeakMap();
let homePose = new WeakMap();
let cloud = null;
let generatedSignature = null;

function defaultDomain(joint) {
  if (joint.limits) return { ...joint.limits };
  return joint.type === 'R' ? { min: -180, max: 180 } : { min: 0, max: 300 };
}

function syncJointMetadata() {
  for (const joint of viewer.dhParams) {
    const previousType = knownJointType.get(joint);
    if (previousType !== undefined && previousType !== joint.type) {
      // Revolute degrees and prismatic lengths have incompatible meanings.
      joint.limits = null;
      perJointDomain.delete(joint);
      homePose.delete(joint);
    }
    knownJointType.set(joint, joint.type);
    if (!perJointDomain.has(joint)) perJointDomain.set(joint, defaultDomain(joint));
    if (!homePose.has(joint)) homePose.set(joint, joint.type === 'R' ? joint.theta : joint.d);
  }
}

function getDomain() {
  syncJointMetadata();
  return viewer.dhParams.map(joint => ({ ...perJointDomain.get(joint) }));
}

function modelSignature() {
  syncJointMetadata();
  return JSON.stringify(viewer.dhParams.map(joint => [
    joint.type, joint.a, joint.alpha,
    joint.type === 'R' ? joint.d : joint.theta,
    joint.limits?.min ?? null, joint.limits?.max ?? null,
    perJointDomain.get(joint).min, perJointDomain.get(joint).max
  ]));
}

function presetSignature(params) {
  return JSON.stringify(params.map(joint => [
    joint.type, joint.a, joint.alpha,
    joint.type === 'R' ? joint.d : joint.theta,
    joint.limits?.min ?? null, joint.limits?.max ?? null
  ]));
}

function refreshPresetIdentity() {
  const isPreset = presetSignature(viewer.dhParams) === presetSignature(baseline);
  el('robot-preset').value = isPreset ? activePreset : 'custom';
  if (isPreset) {
    el('preset-description').textContent = presets[activePreset].description;
  } else {
    el('preset-description').textContent =
      'Custom model. Select a preset to replace the edited DH geometry and limits.';
  }
}

function status(message, error = false) {
  el('workspace-status').textContent = message;
  el('workspace-status').classList.toggle('text-rose-400', error);
  el('workspace-status').classList.toggle('text-slate-400', !error);
}

function clearWorkspace(message = 'Generate the workspace to see reachable positions.') {
  if (cloud) {
    viewer.scene.remove(cloud);
    cloud.geometry.dispose();
    cloud.material.dispose();
    cloud = null;
  }
  generatedSignature = null;
  el('sample-summary').textContent = 'No samples generated';
  status(message);
}

function refreshWorkspaceValidity() {
  if (generatedSignature !== null && generatedSignature !== modelSignature()) {
    clearWorkspace('Model or sampling domain changed. Regenerate the workspace.');
  }
  refreshPresetIdentity();
}

function updatePoseWarning() {
  const invalid = viewer.dhParams.flatMap((joint, i) => {
    if (!joint.limits) return [];
    const q = joint.type === 'R' ? joint.theta : joint.d;
    return (q < joint.limits.min || q > joint.limits.max) ? [i + 1] : [];
  });
  const warning = el('pose-warning');
  warning.hidden = invalid.length === 0;
  warning.textContent = invalid.length ?
    'Current pose exceeds mechanical limits at joint(s): ' + invalid.join(', ') +
    '. The workspace still uses the permitted sampling domain.' : '';
}

function makeNumberInput(value, description, onCommit, disabled = false) {
  const input = document.createElement('input');
  input.type = 'number';
  input.step = 'any';
  input.value = String(value);
  input.disabled = disabled;
  input.className = 'w-full min-w-0 bg-slate-900 border border-slate-600 text-slate-100 rounded px-2 py-1.5 text-xs disabled:opacity-40';
  input.setAttribute('aria-label', description);
  input.addEventListener('change', () => {
    if (!input.value.trim() || !Number.isFinite(Number(input.value))) {
      input.value = String(value);
      status('Enter a finite number.', true);
      return;
    }
    onCommit(Number(input.value));
  });
  return input;
}

function renderJointControls() {
  syncJointMetadata();
  const poseRows = el('pose-rows');
  const configRows = el('domain-rows');
  poseRows.replaceChildren();
  configRows.replaceChildren();

  viewer.dhParams.forEach((joint, i) => {
    const isR = joint.type === 'R';
    const prop = isR ? 'theta' : 'd';
    const unit = isR ? '°' : ' units';
    const domain = perJointDomain.get(joint);
    const position = joint[prop];
    const reference = joint.limits ?? domain;
    const min = Math.min(position, reference.min, domain.min);
    const max = Math.max(position, reference.max, domain.max);
    const poseRow = document.createElement('div');
    poseRow.className = 'flex flex-col gap-1';
    const label = document.createElement('div');
    label.className = 'flex justify-between gap-2 text-xs';
    const title = document.createElement('span');
    title.textContent = 'Joint ' + (i + 1) + ' · ' + joint.type;
    const output = document.createElement('output');
    output.className = 'font-mono text-indigo-300';
    output.textContent = position.toFixed(1) + unit;
    label.append(title, output);
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(min === max ? min - 1 : min);
    slider.max = String(min === max ? max + 1 : max);
    slider.step = 'any';
    slider.value = String(position);
    slider.className = 'w-full accent-indigo-500';
    slider.setAttribute('aria-label', 'Current value of joint ' + (i + 1));
    slider.addEventListener('input', () => {
      const value = Number(slider.value);
      joint[prop] = value;
      output.textContent = value.toFixed(1) + unit;
      const dhInput = viewer.tableBody.querySelector('[data-index="' + i + '"][data-prop="' + prop + '"]');
      if (dhInput) dhInput.value = String(value);
      viewer.updateRobotGeometry();
    });
    poseRow.append(label, slider);
    poseRows.append(poseRow);

    const row = document.createElement('div');
    row.className = 'rounded border border-slate-700 bg-slate-900/50 p-2 flex flex-col gap-2';
    const header = document.createElement('div');
    header.className = 'flex items-center justify-between gap-2 text-xs';
    const name = document.createElement('strong');
    name.className = 'font-semibold text-slate-200';
    name.textContent = 'Joint ' + (i + 1) + ' · ' + joint.type + (isR ? ' (deg)' : ' (model units)');
    const toggleLabel = document.createElement('label');
    toggleLabel.className = 'flex items-center gap-1 text-slate-300';
    const toggle = document.createElement('input');
    toggle.type = 'checkbox';
    toggle.checked = joint.limits !== null && joint.limits !== undefined;
    toggle.className = 'accent-indigo-500';
    toggle.setAttribute('aria-label', 'Enable mechanical limits for joint ' + (i + 1));
    toggleLabel.append(toggle, document.createTextNode('Joint limits'));
    header.append(name, toggleLabel);
    row.append(header);

    const limitsRow = document.createElement('div');
    limitsRow.className = 'grid grid-cols-2 gap-2';
    const minLimit = joint.limits?.min ?? domain.min;
    const maxLimit = joint.limits?.max ?? domain.max;
    for (const [key, value] of [['min', minLimit], ['max', maxLimit]]) {
      const wrap = document.createElement('label');
      wrap.className = 'flex flex-col gap-1 text-[11px] text-slate-400';
      wrap.append(document.createTextNode('Mechanical ' + key));
      wrap.append(makeNumberInput(value, 'Mechanical ' + key + ' for joint ' + (i + 1), next => {
        const candidate = { ...joint.limits, [key]: next };
        if (candidate.min > candidate.max) {
          status('Mechanical minimum cannot exceed maximum.', true);
          renderJointControls();
          return;
        }
        // The sampling domain must be contained in any declared limits.
        if (domain.min < candidate.min || domain.max > candidate.max) {
          status('Adjust the sampling domain before narrowing joint limits.', true);
          renderJointControls();
          return;
        }
        joint.limits = candidate;
        refreshWorkspaceValidity();
        renderJointControls();
      }, !toggle.checked));
      limitsRow.append(wrap);
    }
    toggle.addEventListener('change', () => {
      joint.limits = toggle.checked ? { ...domain } : null;
      refreshWorkspaceValidity();
      renderJointControls();
    });
    row.append(limitsRow);
    const domainTitle = document.createElement('div');
    domainTitle.className = 'text-[11px] font-semibold text-indigo-300';
    domainTitle.textContent = 'Sampling domain (independent)';
    row.append(domainTitle);
    const rangeRow = document.createElement('div');
    rangeRow.className = 'grid grid-cols-2 gap-2';
    for (const key of ['min', 'max']) {
      const wrap = document.createElement('label');
      wrap.className = 'flex flex-col gap-1 text-[11px] text-slate-400';
      wrap.append(document.createTextNode(key === 'min' ? 'From' : 'To'));
      wrap.append(makeNumberInput(domain[key], 'Sampling ' + key + ' for joint ' + (i + 1), next => {
        const candidate = { ...domain, [key]: next };
        try {
          const domains = getDomain();
          domains[i] = candidate;
          resolveSamplingRanges(viewer.dhParams, domains);
          perJointDomain.set(joint, candidate);
          refreshWorkspaceValidity();
          renderJointControls();
          status('Sampling domain changed. Generate to update the workspace.');
        } catch (error) {
          status(error.message, true);
          renderJointControls();
        }
      }));
      rangeRow.append(wrap);
    }
    row.append(rangeRow);
    configRows.append(row);
  });
  updatePoseWarning();
}

function fitView() {
  if (!viewer.renderer) return;
  fitOrthographicObjects({
    camera: viewer.camera, controls: viewer.controls,
    canvas: viewer.renderer.domElement,
    objects: [viewer.robotGroup, cloud],
    padding: 1.25
  });
}

function generate() {
  try {
    const method = el('sampling-method').value;
    const options = { method, samplingRanges: getDomain() };
    if (method === 'grid') {
      options.samplesPerJoint = Number(el('grid-steps').value);
    } else {
      options.sampleCount = Number(el('sample-count').value);
    }
    const result = generateWorkspaceSamples(viewer.dhParams, options);
    if (cloud) {
      viewer.scene.remove(cloud);
      cloud.geometry.dispose();
      cloud.material.dispose();
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(result.positions, 3));
    geometry.computeBoundingSphere();
    const light = window.RoboticsTheme?.getTheme() === 'light';
    const material = new THREE.PointsMaterial({
      color: light ? 0x2563eb : 0x38bdf8,
      size: 3, sizeAttenuation: false, opacity: 0.72,
      transparent: true, depthWrite: false
    });
    cloud = new THREE.Points(geometry, material);
    cloud.name = 'reachable-workspace-samples';
    cloud.visible = el('show-workspace').checked;
    viewer.scene.add(cloud);
    generatedSignature = modelSignature();
    el('sample-summary').textContent =
      result.sampleCount.toLocaleString('en-US') + ' reachable samples · ' + result.jointCount + ' joints';
    status('Workspace generated. Empty regions are not proof of unreachability.');
    fitView();
  } catch (error) {
    status('Cannot generate workspace: ' + error.message, true);
  }
}

const originalRenderTable = viewer.renderTable.bind(viewer);
viewer.renderTable = () => {
  originalRenderTable();
  renderJointControls();
  refreshWorkspaceValidity();
};
const originalUpdateRobotGeometry = viewer.updateRobotGeometry.bind(viewer);
viewer.updateRobotGeometry = () => {
  originalUpdateRobotGeometry();
  refreshWorkspaceValidity();
  updatePoseWarning();
};

viewer.tableBody.addEventListener('input', event => {
  if (!event.target.matches('[data-prop]')) return;
  // The shared viewer handles the mutation before this listener.
  renderJointControls();
  refreshWorkspaceValidity();
});

el('robot-preset').addEventListener('change', () => {
  activePreset = el('robot-preset').value;
  const selected = presets[activePreset];
  if (!selected) return;
  viewer.dhParams = copyParams(selected.params);
  baseline = copyParams(selected.params);
  perJointDomain = new WeakMap();
  knownJointType = new WeakMap();
  homePose = new WeakMap();
  clearWorkspace('Manipulator changed. Generate its reachable workspace.');
  viewer.renderTable();
  viewer.updateRobotGeometry();
  fitView();
});
el('reset-joints').addEventListener('click', () => {
  viewer.dhParams.forEach(j => { j[j.type === 'R' ? 'theta' : 'd'] = homePose.get(j); });
  viewer.renderTable();
  viewer.updateRobotGeometry();
});
el('show-workspace').addEventListener('change', () => {
  if (cloud) cloud.visible = el('show-workspace').checked;
});
el('sampling-method').addEventListener('change', () => {
  const grid = el('sampling-method').value === 'grid';
  el('grid-controls').hidden = !grid;
  el('halton-controls').hidden = grid;
  if (cloud) clearWorkspace('Sampling method changed. Generate a new workspace.');
});
for (const id of ['sample-count', 'grid-steps']) {
  el(id).addEventListener('change', () => {
    if (cloud) clearWorkspace('Sampling resolution changed. Generate a new workspace.');
  });
}
el('generate-workspace').addEventListener('click', generate);
el('clear-workspace').addEventListener('click', () => clearWorkspace());
el('fit-view').addEventListener('click', fitView);
for (const view of ['top', 'front', 'side', 'iso']) {
  el('btn-view-' + view).addEventListener('click', () => viewer.changeView(view));
}
document.addEventListener('robotics:themechange', () => {
  if (cloud) cloud.material.color.setHex(window.RoboticsTheme.getTheme() === 'light' ? 0x2563eb : 0x38bdf8);
});
window.addEventListener('beforeunload', () => {
  if (cloud) { cloud.geometry.dispose(); cloud.material.dispose(); }
  viewer.destroy();
});

viewer.init3D();
viewer.controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN
};
viewer.renderTable();
viewer.updateRobotGeometry();
fitView();
status('Configure the domains, then select Generate Workspace.');
