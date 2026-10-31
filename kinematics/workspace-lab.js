import * as THREE from 'three';
import { RobotViewer } from './shared-robot-viewer.js';
import { generateWorkspaceSamples, resolveSamplingRanges } from './workspace-core.js';
import { workspaceSampleAt, rankProjectedSampleHits, nextSelectedSample } from './workspace-selection.js';
import { fitOrthographicObjects } from '../shared/camera-fit.js';
import { gridResolutionSettings, gridPointCount } from './workspace-grid.js';
import { summarizeWorkspaceSamples } from './workspace-metrics.js';
import { WorkspaceSectionView } from './workspace-projection-view.js';
import { SECTION_AXES, sectionSettings, sliceWorkspaceSamples } from './workspace-sections.js';

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
let gridJointCount = null;
let lastGenerated = null;
let selectedSampleIndex = null;
let applyingPickedSample = false;
let selectionMarker = null;
let pointerStart = null;
let lastPick = null;
let sampleMetrics = null;
let sectionSlice = null;
let sectionCloud = null;
let sectionPlane = null;
const pickRaycaster = new THREE.Raycaster();
const pickNdc = new THREE.Vector2();
const sectionView = new WorkspaceSectionView({
  getSelectedIndex: () => selectedSampleIndex,
  onPick: (index, nearbyCount) => {
    lastPick = null;
    showSample(index, nearbyCount);
    el('selection-hint').textContent = nearbyCount > 1
      ? 'Multiple samples overlap in this section. Click again to cycle.'
      : 'Configuration recovered from an existing section sample (not IK).';
  }
});

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

function updateGridSummary() {
  const jointCount = viewer.dhParams.length;
  const settings = gridResolutionSettings(jointCount);
  const input = el('grid-steps');
  const info = el('grid-summary');
  input.min = '2';
  input.max = String(settings.maxPerJoint);
  const value = input.value.trim() ? Number(input.value) : NaN;
  try {
    const total = gridPointCount(jointCount, value);
    info.textContent = value + ' per joint × ' + jointCount + ' joints = ' +
      total.toLocaleString('en-US') + ' total samples. Max: ' +
      settings.maxPerJoint + ' per joint.';
    info.classList.remove('text-rose-400');
    info.classList.add('text-slate-400');
    el('generate-workspace').disabled = false;
  } catch (error) {
    info.textContent = error.message;
    info.classList.add('text-rose-400');
    info.classList.remove('text-slate-400');
    el('generate-workspace').disabled = el('sampling-method').value === 'grid';
  }
}

function syncGridResolution() {
  const jointCount = viewer.dhParams.length;
  if (jointCount !== gridJointCount) {
    el('grid-steps').value = String(gridResolutionSettings(jointCount).recommendedPerJoint);
    gridJointCount = jointCount;
  }
  updateGridSummary();
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



// Display controls never change the numerical sample domain or the cloud buffers.
function applyDisplaySettings() {
  const size = Number(el('point-size').value);
  const opacity = Number(el('point-opacity').value);
  el('point-size-value').textContent = size + ' px';
  el('point-opacity-value').textContent = Math.round(opacity * 100) + '%';
  if (viewer.robotGroup) viewer.robotGroup.visible = el('show-robot').checked;
  if (cloud) {
    cloud.material.size = size;
    cloud.material.opacity = el('section-enabled').checked && el('section-context').checked
      ? Math.min(opacity, 0.12) : opacity;
  }
  if (sectionCloud) {
    sectionCloud.material.size = Math.max(size, 3);
    sectionCloud.material.opacity = opacity;
  }
}

function resetSampleMetrics() {
  el('metrics-rows').replaceChildren();
  el('metrics-count').textContent = '';
  el('metrics-details').hidden = true;
  el('metrics-empty').hidden = false;
}

function renderSampleMetrics(stats) {
  const values = [
    ['X', stats.x, true],
    ['Y', stats.y, true],
    ['Z', stats.z, true],
    ['XY radius', stats.xyRadius, false],
    ['3D distance', stats.distance, false]
  ];
  const format = value => (Math.abs(value) < 0.005 ? 0 : value).toFixed(2);
  const body = el('metrics-rows');
  body.replaceChildren();
  for (const [title, range, showSpan] of values) {
    const row = document.createElement('tr');
    row.className = 'border-t border-slate-700';
    for (const [i, label] of [title, format(range.min), format(range.max),
      showSpan ? format(range.span) : '—'].entries()) {
      const cell = document.createElement(i === 0 ? 'th' : 'td');
      cell.className = 'px-2 py-2 whitespace-nowrap font-mono' + (i === 0 ? ' font-semibold text-slate-200' : '');
      if (i === 0) cell.scope = 'row';
      cell.textContent = label;
      row.append(cell);
    }
    body.append(row);
  }
  el('metrics-count').textContent = stats.sampleCount.toLocaleString('en-US') + ' positions sampled';
  el('metrics-empty').hidden = true;
  el('metrics-details').hidden = false;
}

function resetSampleSelection() {
  selectedSampleIndex = null;
  sectionView.highlight(null);
  lastPick = null;
  if (selectionMarker) selectionMarker.visible = false;
  el('selection-details').hidden = true;
  el('selection-empty').hidden = false;
  el('selection-empty').textContent = lastGenerated
    ? 'Click a visible sample, or enter its number to inspect a configuration.'
    : 'Generate a workspace to inspect sampled configurations.';
  el('sample-index').value = '';
  el('selection-feedback').textContent = '';
  el('sample-index').disabled = !lastGenerated;
  el('go-to-sample').disabled = !lastGenerated;
  el('clear-selected').disabled = true;
}

function showSample(index, nearbyCount = 1) {
  if (!lastGenerated || !cloud) return;
  const sample = workspaceSampleAt(lastGenerated, index);
  if (sample.jointValues.length !== viewer.dhParams.length) {
    throw new Error('Sample and manipulator joint counts differ.');
  }

  // The numerical samples belong to the static sampled domain, not the pose.
  // Rendering the recovered q does not invalidate or regenerate the cloud.
  applyingPickedSample = true;
  try {
    viewer.dhParams.forEach((joint, i) => {
      joint[joint.type === 'R' ? 'theta' : 'd'] = sample.jointValues[i];
    });
    viewer.renderTable();
    viewer.updateRobotGeometry();
  } finally {
    applyingPickedSample = false;
  }

  selectedSampleIndex = index;
  el('selection-feedback').textContent = '';
  el('selection-empty').hidden = true;
  el('selection-details').hidden = false;
  el('selected-sample').textContent =
    'Sample ' + (index + 1).toLocaleString('en-US') +
    ' / ' + lastGenerated.sampleCount.toLocaleString('en-US');
  const format = x => Math.abs(x) < 0.0005 ? '0.000' : x.toFixed(3);
  el('selected-position').textContent = '(' + sample.position.map(format).join(', ') + ')';
  const rows = el('selected-joint-values');
  rows.replaceChildren();
  sample.jointValues.forEach((value, i) => {
    const joint = viewer.dhParams[i];
    const row = document.createElement('div');
    row.className = 'flex items-center justify-between gap-2 text-xs border-b border-slate-700 py-1';
    const name = document.createElement('span');
    name.textContent = 'q' + (i + 1) + ' · ' + joint.type;
    const amount = document.createElement('output');
    amount.className = 'font-mono text-indigo-300';
    amount.textContent = format(value) + (joint.type === 'R' ? '°' : ' units');
    row.append(name, amount);
    rows.append(row);
  });
  el('selection-hint').textContent = nearbyCount > 1
    ? nearbyCount + ' nearby samples in this view. Click again at the same spot to cycle.'
    : 'This joint configuration generated the highlighted point; it is not an IK result.';
  el('sample-index').value = String(index + 1);
  el('clear-selected').disabled = false;

  if (selectionMarker) {
    selectionMarker.position.set(...sample.position);
    selectionMarker.visible = selectedSampleIsVisible(index);
  }
  sectionView.highlight(index);
  if (el('section-enabled').checked && sectionSlice &&
      !sectionSlice.indices.includes(index)) {
    el('selection-hint').textContent = 'This sample is outside the current section band.';
  }
}

function cameraPickSignature() {
  const c = viewer.camera;
  // A different camera orientation/zoom must not continue an older click cycle.
  return [...c.position.toArray(), ...c.quaternion.toArray(), c.zoom]
    .map(x => x.toFixed(6)).join(',');
}

function pickWorkspaceSample(event) {
  const activeCloud = el('section-enabled').checked ? sectionCloud : cloud;
  if (!lastGenerated || !activeCloud?.visible) return;
  const canvas = viewer.renderer.domElement;
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;
  const pointer = { x: event.clientX - rect.left, y: event.clientY - rect.top };
  pickNdc.set(2 * pointer.x / rect.width - 1, 1 - 2 * pointer.y / rect.height);
  viewer.camera.updateMatrixWorld(true);
  pickRaycaster.setFromCamera(pickNdc, viewer.camera);
  // Three.js point-raycast threshold is in model units, unlike PointsMaterial
  // whose size is fixed in screen pixels; convert an 11 px tolerance.
  const c = viewer.camera;
  pickRaycaster.params.Points.threshold = 11 * Math.max(
    (c.right - c.left) / (c.zoom * rect.width),
    (c.top - c.bottom) / (c.zoom * rect.height)
  );
  const hits = pickRaycaster.intersectObject(activeCloud, false);
  // Section geometry is a compact buffer; map its vertex indices back
  // to the stable sample indices stored in the complete workspace result.
  const mappedHits = el('section-enabled').checked
    ? hits.map(hit => ({ ...hit, index: sectionSlice.indices[hit.index] }))
    : hits;
  const projection = index => {
    const pos = lastGenerated.positions;
    const p = new THREE.Vector3(pos[3 * index], pos[3 * index + 1], pos[3 * index + 2]);
    p.project(c);
    return { x: (p.x + 1) * rect.width / 2,
      y: (1 - p.y) * rect.height / 2, z: p.z };
  };
  const candidates = rankProjectedSampleHits(mappedHits, projection, pointer);
  if (!candidates.length) return;

  const signature = cameraPickSignature();
  const sameSpot = lastPick && lastPick.cloud === activeCloud &&
    lastPick.signature === signature &&
    Math.hypot(lastPick.x - pointer.x, lastPick.y - pointer.y) <= 6;
  const chosen = nextSelectedSample(candidates, selectedSampleIndex, Boolean(sameSpot));
  showSample(chosen, candidates.length);
  lastPick = { x: pointer.x, y: pointer.y, cloud: activeCloud, signature };
}

function attachWorkspacePicking() {
  const canvas = viewer.renderer.domElement;
  canvas.addEventListener('pointerdown', event => {
    if (event.button !== 0 || !event.isPrimary) return;
    pointerStart = { pointerId: event.pointerId, x: event.clientX, y: event.clientY };
  });
  canvas.addEventListener('pointerup', event => {
    if (!pointerStart || event.pointerId !== pointerStart.pointerId) return;
    const moved = Math.hypot(event.clientX - pointerStart.x, event.clientY - pointerStart.y);
    pointerStart = null;
    if (moved > 6) { lastPick = null; return; }
    pickWorkspaceSample(event);
  });
  canvas.addEventListener('pointercancel', () => { pointerStart = null; lastPick = null; });
}

function initSampleMarker() {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0], 3));
  selectionMarker = new THREE.Points(geometry, new THREE.PointsMaterial({
    color: 0xfbbf24, size: 13, sizeAttenuation: false,
    depthTest: false, depthWrite: false
  }));
  selectionMarker.name = 'workspace-picked-sample';
  selectionMarker.renderOrder = 1000;
  selectionMarker.visible = false;
  viewer.scene.add(selectionMarker);
}


function clearSectionGeometry() {
  if (sectionCloud) {
    viewer.scene.remove(sectionCloud);
    sectionCloud.geometry.dispose();
    sectionCloud.material.dispose();
    sectionCloud = null;
  }
  if (sectionPlane) {
    viewer.scene.remove(sectionPlane);
    sectionPlane.geometry.dispose();
    sectionPlane.material.dispose();
    sectionPlane = null;
  }
}

function selectedSampleIsVisible(index) {
  if (!lastGenerated || !el('show-workspace').checked || !cloud) return false;
  if (!el('section-enabled').checked) return cloud.visible;
  return Boolean(sectionCloud?.visible && sectionSlice?.indices.includes(index));
}

function updateSectionVisibility() {
  const enabled = el('section-enabled').checked;
  const shown = el('show-workspace').checked;
  if (cloud) cloud.visible = shown && (!enabled || el('section-context').checked);
  if (sectionCloud) sectionCloud.visible = shown && enabled;
  if (sectionPlane) sectionPlane.visible = enabled;
  if (selectionMarker) selectionMarker.visible =
    selectedSampleIndex !== null && selectedSampleIsVisible(selectedSampleIndex);
  applyDisplaySettings();
  lastPick = null;
}

function resetSectionControls() {
  sectionSlice = null;
  sectionView.clear();
  clearSectionGeometry();
  el('section-enabled').checked = false;
  el('section-enabled').disabled = !lastGenerated;
  el('section-position').disabled = !lastGenerated;
  el('section-thickness').disabled = !lastGenerated;
  el('section-position-value').textContent = '—';
  el('section-thickness-value').textContent = '—';
  el('section-count').textContent = lastGenerated
    ? 'Enable a section to filter sampled positions.'
    : 'Generate a workspace to explore sections.';
  updateSectionVisibility();
}

function configureSectionAxis() {
  if (!sampleMetrics) return;
  const settings = sectionSettings(sampleMetrics, el('section-axis').value);
  const position = el('section-position');
  const thickness = el('section-thickness');
  const flat = settings.min === settings.max;
  position.min = String(flat ? settings.min - settings.thickness : settings.min);
  position.max = String(flat ? settings.max + settings.thickness : settings.max);
  position.step = 'any';
  position.value = String(settings.center);
  thickness.min = String(Math.max(settings.thickness / 40, 1e-7));
  thickness.max = String(settings.maximumThickness);
  thickness.step = 'any';
  thickness.value = String(settings.thickness);
  updateSection();
}

function updateSection() {
  if (!lastGenerated || !sampleMetrics || !el('section-enabled').checked) {
    sectionSlice = null;
    sectionView.clear();
    clearSectionGeometry();
    updateSectionVisibility();
    return;
  }
  const axis = el('section-axis').value;
  const center = Number(el('section-position').value);
  const thickness = Number(el('section-thickness').value);
  el('section-position-value').textContent = center.toFixed(2);
  el('section-thickness-value').textContent = thickness.toFixed(2);
  const slice = sliceWorkspaceSamples(lastGenerated, axis, center, thickness);
  sectionSlice = slice;
  el('section-count').textContent = slice.count.toLocaleString('en-US') + ' / ' +
    slice.total.toLocaleString('en-US') + ' samples inside the band (' +
    (100 * slice.count / slice.total).toFixed(1) + '%).';
  el('section-plane-label').textContent = SECTION_AXES[axis].plane.toUpperCase() +
    ' band · ' + axis.toUpperCase() + ' = ' + center.toFixed(2);
  if (selectedSampleIndex !== null && !slice.indices.includes(selectedSampleIndex)) {
    resetSampleSelection();
  }
  clearSectionGeometry();
  if (slice.count > 0) {
    const positions = new Float32Array(3 * slice.count);
    for (let i = 0; i < slice.count; i++) {
      const source = 3 * slice.indices[i];
      positions[3 * i] = lastGenerated.positions[source];
      positions[3 * i + 1] = lastGenerated.positions[source + 1];
      positions[3 * i + 2] = lastGenerated.positions[source + 2];
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.computeBoundingSphere();
    sectionCloud = new THREE.Points(geometry, new THREE.PointsMaterial({
      color: window.RoboticsTheme?.getTheme() === 'light' ? 0x0f766e : 0xfde047,
      size: Math.max(3, Number(el('point-size').value)),
      sizeAttenuation: false,
      transparent: true, opacity: Number(el('point-opacity').value),
      depthWrite: false
    }));
    sectionCloud.name = 'sampled-workspace-section';
    viewer.scene.add(sectionCloud);
  }

  const fullSpan = Math.max(sampleMetrics.x.span, sampleMetrics.y.span,
    sampleMetrics.z.span, thickness, 1);
  const planeGeometry = new THREE.PlaneGeometry(fullSpan * 1.12, fullSpan * 1.12);
  const planeMaterial = new THREE.MeshBasicMaterial({
    color: 0xf59e0b, transparent: true, opacity: 0.15,
    side: THREE.DoubleSide, depthWrite: false
  });
  sectionPlane = new THREE.Mesh(planeGeometry, planeMaterial);
  sectionPlane.name = 'workspace-section-plane';
  sectionPlane.position.set(
    (sampleMetrics.x.min + sampleMetrics.x.max) / 2,
    (sampleMetrics.y.min + sampleMetrics.y.max) / 2,
    (sampleMetrics.z.min + sampleMetrics.z.max) / 2
  );
  sectionPlane.position[axis] = center;
  if (axis === 'x') sectionPlane.rotation.y = Math.PI / 2;
  else if (axis === 'y') sectionPlane.rotation.x = -Math.PI / 2;
  sectionPlane.renderOrder = 1;
  viewer.scene.add(sectionPlane);

  sectionView.show(lastGenerated, sampleMetrics, slice);
  updateSectionVisibility();
}

function clearWorkspace(message = 'Generate the workspace to see reachable positions.') {
  if (cloud) {
    viewer.scene.remove(cloud);
    cloud.geometry.dispose();
    cloud.material.dispose();
    cloud = null;
  }
  generatedSignature = null;
  lastGenerated = null;
  sampleMetrics = null;
  resetSectionControls();
  resetSampleMetrics();
  resetSampleSelection();
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
  syncGridResolution();
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
      const text = el('grid-steps').value.trim();
      options.samplesPerJoint = text ? Number(text) : NaN;
      // Catch invalid values before sampling (in addition to disabled UI).
      gridPointCount(viewer.dhParams.length, options.samplesPerJoint);
    } else {
      options.sampleCount = Number(el('sample-count').value);
    }
    const result = generateWorkspaceSamples(viewer.dhParams, options);
    const metrics = summarizeWorkspaceSamples(result);
    resetSampleSelection();
    clearSectionGeometry();
    sectionSlice = null;
    sectionView.clear();
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
      size: Number(el('point-size').value), sizeAttenuation: false,
      opacity: Number(el('point-opacity').value),
      transparent: true, depthWrite: false
    });
    cloud = new THREE.Points(geometry, material);
    cloud.name = 'reachable-workspace-samples';
    cloud.visible = el('show-workspace').checked;
    viewer.scene.add(cloud);
    renderSampleMetrics(metrics);
    lastGenerated = result;
    sampleMetrics = metrics;
    resetSampleSelection();
    resetSectionControls();
    configureSectionAxis();
    el('sample-index').max = String(result.sampleCount);
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
  if (!applyingPickedSample && selectedSampleIndex !== null) resetSampleSelection();
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
el('show-robot').addEventListener('change', applyDisplaySettings);
for (const control of ['point-size', 'point-opacity']) {
  el(control).addEventListener('input', applyDisplaySettings);
}
el('show-workspace').addEventListener('change', updateSectionVisibility);
el('section-enabled').addEventListener('change', updateSection);
el('section-axis').addEventListener('change', configureSectionAxis);
el('section-position').addEventListener('input', updateSection);
el('section-thickness').addEventListener('input', updateSection);
el('section-context').addEventListener('change', updateSectionVisibility);
el('sampling-method').addEventListener('change', () => {
  const grid = el('sampling-method').value === 'grid';
  el('grid-controls').hidden = !grid;
  el('halton-controls').hidden = grid;
  updateGridSummary();
  if (cloud) clearWorkspace('Sampling method changed. Generate a new workspace.');
});
el('grid-steps').addEventListener('input', () => {
  updateGridSummary();
  if (cloud) clearWorkspace('Sampling resolution changed. Generate a new workspace.');
});
el('sample-count').addEventListener('change', () => {
  if (cloud) clearWorkspace('Sampling resolution changed. Generate a new workspace.');
});
el('go-to-sample').addEventListener('click', () => {
  const value = Number(el('sample-index').value);
  if (!lastGenerated || !Number.isSafeInteger(value) ||
      value < 1 || value > lastGenerated.sampleCount) {
    el('selection-feedback').textContent = lastGenerated
      ? 'Enter an integer from 1 to ' + lastGenerated.sampleCount + '.'
      : 'Generate the workspace before selecting a sample.';
    return;
  }
  lastPick = null;
  showSample(value - 1);
});
el('sample-index').addEventListener('keydown', event => {
  if (event.key === 'Enter') {
    event.preventDefault();
    el('go-to-sample').click();
  }
});
el('clear-selected').addEventListener('click', resetSampleSelection);
el('generate-workspace').addEventListener('click', generate);
el('clear-workspace').addEventListener('click', () => clearWorkspace());
el('fit-view').addEventListener('click', fitView);
el('section-panel').addEventListener('toggle', () => {
  if (el('section-panel').open) sectionView.redraw();
});
// RobotViewer.renderTable() already binds view controls once.
document.addEventListener('robotics:themechange', () => {
  if (cloud) cloud.material.color.setHex(window.RoboticsTheme.getTheme() === 'light' ? 0x2563eb : 0x38bdf8);
  if (sectionCloud) sectionCloud.material.color.setHex(
    window.RoboticsTheme.getTheme() === 'light' ? 0x0f766e : 0xfde047);
  sectionView.redraw();
});
window.addEventListener('beforeunload', () => {
  if (cloud) { cloud.geometry.dispose(); cloud.material.dispose(); }
  if (selectionMarker) {
    selectionMarker.geometry.dispose();
    selectionMarker.material.dispose();
  }
  clearSectionGeometry();
  sectionView.destroy();
  viewer.destroy();
});

viewer.init3D();
applyDisplaySettings();
resetSampleMetrics();
resetSectionControls();
initSampleMarker();
attachWorkspacePicking();
resetSampleSelection();
viewer.controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN
};
viewer.renderTable();
viewer.updateRobotGeometry();
fitView();
status('Configure the domains, then select Generate Workspace.');
