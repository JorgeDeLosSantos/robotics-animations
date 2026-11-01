import * as THREE from 'three';
import { RobotViewer } from '../kinematics/shared-robot-viewer.js';
import { forwardKinematicsPosition } from '../kinematics/workspace-core.js';
import { fitOrthographicObjects } from '../shared/camera-fit.js';
import { createJointTrajectory } from './joint-trajectory-core.js';

// Match the existing DH examples and mechanical limits in the Workspace lab.
// A common preset registry can be extracted when more labs require the same data.
const PRESETS = {
  planar2r: {
    description: 'Two revolute joints in the XY plane.',
    params: [
      { type: 'R', theta: 30, d: 0, a: 220, alpha: 0, limits: { min: -180, max: 180 } },
      { type: 'R', theta: -45, d: 0, a: 160, alpha: 0, limits: { min: -150, max: 150 } }
    ],
    final: [90, 30]
  },
  spatial3r: {
    description: 'Three revolute joints for spatial positioning.',
    params: [
      { type: 'R', theta: 0, d: 200, a: 0, alpha: 90, limits: { min: -170, max: 170 } },
      { type: 'R', theta: 20, d: 0, a: 200, alpha: 0, limits: { min: -120, max: 120 } },
      { type: 'R', theta: -30, d: 0, a: 200, alpha: 0, limits: { min: -150, max: 150 } }
    ],
    final: [80, 45, 25]
  },
  mixedRRP: {
    description: 'A spatial RRP manipulator with a prismatic third joint.',
    params: [
      { type: 'R', theta: 0, d: 140, a: 0, alpha: 90, limits: { min: -170, max: 170 } },
      { type: 'R', theta: 30, d: 0, a: 170, alpha: 0, limits: { min: -150, max: 150 } },
      { type: 'P', theta: 0, d: 130, a: 0, alpha: 0, limits: { min: 0, max: 260 } }
    ],
    final: [80, -20, 210]
  }
};
const cloneParams = joints => joints.map(joint => ({
  ...joint, limits: joint.limits ? { ...joint.limits } : null
}));
const el = id => document.getElementById(id);
const currentValues = () => viewer.dhParams.map(j => j.type === 'R' ? j.theta : j.d);
const topology = () => viewer.dhParams.map(j => j.type).join(',');
const fmt = number => Number(number).toFixed(1);
const unit = joint => joint.type === 'R' ? '°' : ' units';

const viewer = new RobotViewer({
  canvasContainerId: 'canvas-container',
  tableBodyId: 'dh-table-body',
  addRowBtnId: 'add-row-btn',
  matrixContainerId: 'unused-matrix-display',
  initialDHParams: cloneParams(PRESETS.planar2r.params),
  frustumSize: 1100,
  maxLinks: 6
});

const state = {
  active: 'initial',
  mode: 'edit',                // edit | playback
  initial: currentValues(),
  final: [...PRESETS.planar2r.final],
  trajectory: null,
  time: 0,
  playing: false,
  raf: null,
  lastFrame: null,
  activePreset: 'planar2r',
  lastTopology: null
};
const pathGroup = new THREE.Group();
const ghostGroup = new THREE.Group();
pathGroup.name = 'joint-trajectory-tcp-path';
ghostGroup.name = 'joint-trajectory-endpoint-ghosts';

function setStatus(message, error = false) {
  el('trajectory-status').textContent = error ? '' : message;
  el('trajectory-error').textContent = error ? message : '';
  el('trajectory-error').hidden = !error;
}

function stopAnimation() {
  state.playing = false;
  if (state.raf !== null) cancelAnimationFrame(state.raf);
  state.raf = null;
  state.lastFrame = null;
}

function clearGroup(group) {
  const geometries = new Set();
  const materials = new Set();
  for (const child of [...group.children]) {
    child.traverse(object => {
      if (object.geometry) geometries.add(object.geometry);
      if (object.material) {
        for (const material of (Array.isArray(object.material) ? object.material : [object.material])) {
          materials.add(material);
        }
      }
    });
    group.remove(child);
  }
  geometries.forEach(g => g.dispose());
  materials.forEach(m => m.dispose());
}

function showPose(q) {
  if (!q || q.length !== viewer.dhParams.length) return;
  viewer.dhParams.forEach((joint, i) => {
    joint[joint.type === 'R' ? 'theta' : 'd'] = q[i];
    const input = viewer.tableBody.querySelector('[data-index="' + i + '"][data-prop="' + (joint.type === 'R' ? 'theta' : 'd') + '"]');
    if (input) input.value = String(q[i]);
  });
  viewer.updateRobotGeometry();
}

function timeFrame() {
  if (state.mode === 'playback' && state.trajectory) {
    return state.trajectory.evaluate(state.time);
  }
  return { q: state[state.active] };
}

function syncPlayback() {
  const traj = state.trajectory;
  const button = el('play-pause');
  button.textContent = state.playing ? 'Pause' : 'Play';
  button.disabled = !traj;
  el('restart').disabled = !traj;
  el('timeline').disabled = !traj;
  el('timeline').value = traj ? String(Math.round(1000 * state.time / traj.duration)) : '0';
  el('time-label').textContent = fmt(state.time).replace(/\.0$/, '.00') + ' / ' +
    (traj ? traj.duration.toFixed(2) : '—') + ' s';
  el('view-mode').textContent = state.mode === 'edit'
    ? 'Editing ' + state.active
    : (state.playing ? 'Playing trajectory' : 'Trajectory preview');
  const frame = timeFrame();
  el('playback-values').textContent = frame.q.map((v, i) =>
    'q' + (i + 1) + ' ' + fmt(v) + unit(viewer.dhParams[i])).join('  ·  ');
}

function renderPoseSelection() {
  for (const key of ['initial', 'final']) {
    el('pose-' + key).setAttribute('aria-pressed', String(state.mode === 'edit' && state.active === key));
    el('summary-' + key).textContent = state[key].map((v, i) =>
      fmt(v) + (viewer.dhParams[i]?.type === 'R' ? '°' : ' u')).join(' · ');
  }
  el('editor-hint').textContent = state.mode === 'playback'
    ? 'Playback preview. Choose Initial or Final to resume editing.'
    : 'Editing ' + state.active + ' configuration. Changes are saved automatically.';
  syncPlayback();
}

function editingRange(joint, value) {
  const bounds = joint.limits ??
    (joint.type === 'R' ? { min: -180, max: 180 } : { min: 0, max: 300 });
  return { min: Math.min(bounds.min, value), max: Math.max(bounds.max, value) };
}

function setJointValue(index, next) {
  const joint = viewer.dhParams[index];
  if (!Number.isFinite(next)) {
    setStatus('Joint ' + (index + 1) + ' requires a finite number.', true);
    return false;
  }
  if (joint.limits && (next < joint.limits.min || next > joint.limits.max)) {
    setStatus('Joint ' + (index + 1) + ' must remain within [' +
      joint.limits.min + ', ' + joint.limits.max + '].', true);
    return false;
  }
  state[state.active][index] = next;
  rebuildTrajectory();
  return true;
}

function renderJointControls() {
  const root = el('joint-controls');
  root.replaceChildren();
  const selected = state[state.active];
  viewer.dhParams.forEach((joint, i) => {
    const row = document.createElement('div');
    row.className = 'flex flex-col gap-1';
    const heading = document.createElement('div');
    heading.className = 'flex items-center justify-between gap-2';
    const label = document.createElement('label');
    label.className = 'text-xs text-slate-300 font-medium';
    label.textContent = 'Joint ' + (i + 1) + ' · ' + joint.type;
    const numerical = document.createElement('input');
    numerical.type = 'number';
    numerical.step = 'any';
    numerical.value = String(selected[i]);
    numerical.id = 'joint-value-' + i;
    numerical.className = 'w-24 bg-slate-900 border border-slate-600 rounded px-2 py-1 text-xs text-right';
    numerical.setAttribute('aria-label', 'Joint ' + (i + 1) + ' ' + state.active + ' position (' +
      (joint.type === 'R' ? 'degrees' : 'model units') + ')');
    if (joint.limits) {
      numerical.min = String(joint.limits.min);
      numerical.max = String(joint.limits.max);
    }
    label.htmlFor = numerical.id;
    const unitLabel = document.createElement('span');
    unitLabel.className = 'text-xs text-slate-400';
    unitLabel.textContent = unit(joint);
    const entry = document.createElement('div');
    entry.className = 'flex items-center gap-1';
    entry.append(numerical, unitLabel);
    heading.append(label, entry);

    const slider = document.createElement('input');
    slider.type = 'range';
    const bounds = editingRange(joint, selected[i]);
    slider.min = String(bounds.min === bounds.max ? bounds.min - 1 : bounds.min);
    slider.max = String(bounds.min === bounds.max ? bounds.max + 1 : bounds.max);
    slider.step = 'any';
    slider.value = String(selected[i]);
    slider.disabled = bounds.min === bounds.max;
    slider.className = 'w-full accent-indigo-500';
    slider.setAttribute('aria-label', 'Joint ' + (i + 1) + ' slider');
    slider.addEventListener('input', () => {
      if (state.mode !== 'edit') return;
      if (setJointValue(i, Number(slider.value))) {
        numerical.value = slider.value;
        renderPoseSelection();
      }
    });
    numerical.addEventListener('change', () => {
      if (state.mode !== 'edit') return;
      const value = numerical.value.trim() === '' ? NaN : Number(numerical.value);
      if (!setJointValue(i, value)) {
        numerical.value = String(state[state.active][i]);
        return;
      }
      // An unconstrained value can be outside the slider's previous visual range.
      const nextRange = editingRange(joint, value);
      slider.min = String(nextRange.min);
      slider.max = String(nextRange.max);
      slider.value = String(value);
      renderPoseSelection();
    });
    numerical.disabled = state.mode !== 'edit';
    slider.disabled = slider.disabled || state.mode !== 'edit';
    row.append(heading, slider);
    root.append(row);
  });
}

function poseEndpoints() {
  return { initial: state.initial, final: state.final };
}

function drawPath() {
  clearGroup(pathGroup);
  clearGroup(ghostGroup);
  if (!state.trajectory) return;
  const samples = state.trajectory.sample(161);
  const points = samples.map(frame =>
    new THREE.Vector3(...forwardKinematicsPosition(viewer.dhParams, frame.q)));
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const path = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: 0x22d3ee }));
  path.name = 'tcp-trajectory';
  pathGroup.add(path);
  for (const [point, color] of [[points[0], 0x14b8a6], [points.at(-1), 0xf59e0b]]) {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(9, 12, 8),
      new THREE.MeshBasicMaterial({ color, depthTest: false })
    );
    marker.position.copy(point);
    pathGroup.add(marker);
  }
  for (const [key, color] of [['initial', 0x14b8a6], ['final', 0xf59e0b]]) {
    const poses = [new THREE.Vector3(0, 0, 0)];
    for (let i = 0; i < viewer.dhParams.length; i++) {
      poses.push(new THREE.Vector3(...forwardKinematicsPosition(
        viewer.dhParams.slice(0, i + 1), state[key].slice(0, i + 1))));
    }
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(poses),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.38, depthWrite: false })
    );
    ghostGroup.add(line);
  }
  pathGroup.visible = el('show-path').checked;
  ghostGroup.visible = el('show-endpoints').checked;
}

function updateDisplayedPose() {
  showPose(timeFrame().q);
  syncPlayback();
}

function rebuildTrajectory() {
  stopAnimation();
  state.mode = 'edit';
  state.time = 0;
  el('phase-controls').hidden = el('trajectory-method').value !== 'trapezoidal';
  el('acceleration-label').textContent =
    Math.round(Number(el('acceleration-fraction').value) * 100) + '%';
  try {
    state.trajectory = createJointTrajectory({
      ...poseEndpoints(),
      duration: Number(el('duration').value),
      method: el('trajectory-method').value,
      accelerationFraction: Number(el('acceleration-fraction').value),
      jointTypes: viewer.dhParams.map(j => j.type),
      jointLimits: viewer.dhParams.map(j => j.limits ?? null)
    });
    drawPath();
    setStatus('Trajectory ready · ' + state.trajectory.method +
      ' · ' + state.trajectory.duration.toFixed(2) + ' s. Select Play or scrub the timeline.');
  } catch (error) {
    state.trajectory = null;
    clearGroup(pathGroup);
    clearGroup(ghostGroup);
    setStatus('Cannot generate trajectory: ' + error.message, true);
  }
  renderJointControls();
  renderPoseSelection();
  updateDisplayedPose();
}

function selectPose(key) {
  stopAnimation();
  state.mode = 'edit';
  state.active = key;
  state.time = 0;
  renderJointControls();
  renderPoseSelection();
  updateDisplayedPose();
}

function setPreviewTime(time) {
  if (!state.trajectory) return;
  state.mode = 'playback';
  state.time = Math.max(0, Math.min(state.trajectory.duration, time));
  updateDisplayedPose();
  renderJointControls();
  renderPoseSelection();
}

function tick(timestamp) {
  if (!state.playing || !state.trajectory) return;
  if (state.lastFrame !== null) {
    const elapsed = Math.max(0, Math.min(0.1, (timestamp - state.lastFrame) / 1000));
    const speed = Number(el('playback-speed').value);
    state.time = Math.min(state.trajectory.duration, state.time + elapsed * speed);
  }
  state.lastFrame = timestamp;
  if (state.time >= state.trajectory.duration) {
    stopAnimation();
    updateDisplayedPose();
    setStatus('Trajectory finished. Replay or choose a configuration to edit.');
    return;
  }
  updateDisplayedPose();
  state.raf = requestAnimationFrame(tick);
}

function startPlayback() {
  if (!state.trajectory) return;
  if (state.time >= state.trajectory.duration) state.time = 0;
  state.mode = 'playback';
  state.playing = true;
  state.lastFrame = null;
  renderJointControls();
  renderPoseSelection();
  updateDisplayedPose();
  state.raf = requestAnimationFrame(tick);
}

function fitView() {
  fitOrthographicObjects({
    camera: viewer.camera,
    controls: viewer.controls,
    canvas: viewer.renderer.domElement,
    objects: [viewer.robotGroup, pathGroup, ghostGroup],
    padding: 1.5
  });
}

function syncTable() {
  // The DH table edits constants and topology; active coordinates are set in
  // the Initial/Final editor, not duplicated as a second interactive control.
  viewer.dhParams.forEach((joint, i) => {
    const activeProp = joint.type === 'R' ? 'theta' : 'd';
    const cell = viewer.tableBody.querySelector(
      '[data-index="' + i + '"][data-prop="' + activeProp + '"]');
    if (cell) {
      cell.disabled = true;
      cell.title = 'Edit this joint coordinate in the Initial/Final controls.';
      cell.classList.add('opacity-50', 'cursor-not-allowed');
    }
  });
}

const baseRenderTable = viewer.renderTable.bind(viewer);
viewer.renderTable = () => {
  baseRenderTable();
  syncTable();
  const nextTopology = topology();
  const changed = state.lastTopology !== null && nextTopology !== state.lastTopology;
  if (changed) {
    const previousTypes = state.lastTopology.split(',');
    viewer.dhParams.forEach((joint, i) => {
      // A limit measured in degrees must never become a prismatic length limit.
      if (previousTypes[i] && previousTypes[i] !== joint.type) joint.limits = null;
    });
    stopAnimation();
    state.mode = 'edit';
    state.time = 0;
    state.initial = currentValues();
    state.final = [...state.initial];
  }
  state.lastTopology = nextTopology;
  if (changed) {
    rebuildTrajectory();
    if (state.trajectory) {
      setStatus('Joint topology changed: both endpoints reset to the current DH values. ' +
        'Limits on retyped joints were cleared.');
    }
  } else {
    renderJointControls();
    renderPoseSelection();
  }
};

viewer.tableBody.addEventListener('input', event => {
  if (!event.target.matches('[data-prop]')) return;
  if (event.target.type === 'number' &&
      (event.target.value.trim() === '' || event.target.validity.badInput)) return;
  // Called after the shared viewer processes the DH edit.
  stopAnimation();
  state.mode = 'edit';
  state.time = 0;
  rebuildTrajectory();
});

el('robot-preset').addEventListener('change', () => {
  const key = el('robot-preset').value;
  const preset = PRESETS[key];
  if (!preset) return;
  stopAnimation();
  state.activePreset = key;
  state.active = 'initial';
  state.mode = 'edit';
  state.time = 0;
  viewer.dhParams = cloneParams(preset.params);
  state.initial = currentValues();
  state.final = [...preset.final];
  state.lastTopology = topology();
  el('preset-description').textContent = preset.description;
  viewer.renderTable();
  rebuildTrajectory();
  fitView();
});
el('pose-initial').addEventListener('click', () => selectPose('initial'));
el('pose-final').addEventListener('click', () => selectPose('final'));
el('copy-pose').addEventListener('click', () => {
  state.final = [...state.initial];
  rebuildTrajectory();
});
el('swap-pose').addEventListener('click', () => {
  [state.initial, state.final] = [state.final, state.initial];
  rebuildTrajectory();
});
for (const id of ['trajectory-method', 'duration', 'acceleration-fraction']) {
  el(id).addEventListener(id === 'acceleration-fraction' ? 'input' : 'change', rebuildTrajectory);
}
el('play-pause').addEventListener('click', () => {
  if (state.playing) {
    stopAnimation();
    syncPlayback();
  } else startPlayback();
});
el('restart').addEventListener('click', () => {
  stopAnimation();
  setPreviewTime(0);
});
el('timeline').addEventListener('input', () => {
  stopAnimation();
  if (state.trajectory) {
    setPreviewTime(state.trajectory.duration * Number(el('timeline').value) / 1000);
  }
});
el('show-path').addEventListener('change', () => {
  pathGroup.visible = el('show-path').checked;
});
el('show-endpoints').addEventListener('change', () => {
  ghostGroup.visible = el('show-endpoints').checked;
});
el('fit-view').addEventListener('click', fitView);
document.addEventListener('visibilitychange', () => {
  if (document.hidden && state.playing) {
    stopAnimation();
    syncPlayback();
  }
});
window.addEventListener('beforeunload', () => {
  stopAnimation();
  clearGroup(pathGroup);
  clearGroup(ghostGroup);
  viewer.destroy();
});

viewer.init3D();
viewer.controls.mouseButtons = {
  LEFT: THREE.MOUSE.ROTATE,
  MIDDLE: THREE.MOUSE.DOLLY,
  RIGHT: THREE.MOUSE.PAN
};
viewer.scene.add(pathGroup, ghostGroup);
el('preset-description').textContent = PRESETS.planar2r.description;
viewer.renderTable();
rebuildTrajectory();
fitView();
