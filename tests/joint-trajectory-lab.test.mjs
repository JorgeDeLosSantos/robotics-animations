import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
const read = path => readFileSync(new URL('../' + path, import.meta.url), 'utf8');
const html = read('trajectories/joint-space.html');
const lab = read('trajectories/joint-trajectory-lab.js');
const nav = read('shared/navigation.js');
const index = read('index.html');

test('joint trajectory page has unique IDs and required controls', () => {
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]);
  assert.equal(new Set(ids).size, ids.length);
  const expected = [
    'canvas-container','robot-preset','dh-table-body','add-row-btn',
    'pose-initial','pose-final','joint-controls','summary-initial','summary-final',
    'copy-pose','swap-pose','trajectory-method','duration','phase-controls',
    'acceleration-fraction','acceleration-label',
    'show-path','show-endpoints','trajectory-error','trajectory-status',
    'play-pause','restart','timeline','time-label','playback-speed',
    'playback-values','view-mode','fit-view',
    'btn-view-top','btn-view-front','btn-view-side','btn-view-iso'
  ];
  for (const id of expected) assert.ok(ids.includes(id), 'Missing #' + id);
  for (const [, id] of lab.matchAll(/\bel\('([^']+)'\)/g)) {
    assert.ok(ids.includes(id), 'Missing element for JS reference #' + id);
  }
});

test('page contains valid module and library wiring', () => {
  assert.match(html, /type="importmap"/);
  assert.match(html, /src="\.\/joint-trajectory-lab\.js"/);
  assert.match(html, /src="\.\.\/shared\/navigation\.js"/);
  assert.match(lab, /from '\.\/joint-trajectory-core\.js'/);
  assert.match(lab, /from '\.\.\/kinematics\/shared-robot-viewer\.js'/);
  assert.match(lab, /from '\.\.\/kinematics\/workspace-core\.js'/);
  assert.match(lab, /from '\.\.\/shared\/camera-fit\.js'/);
});

test('laboratory can be discovered in home and global navigation', () => {
  assert.match(index, /trajectories\/joint-space\.html/);
  assert.match(nav, /Trajectories/);
  assert.match(nav, /trajectories\/joint-space\.html/);
});

test('initial and final editor is distinct from time-based playback', () => {
  assert.match(lab, /state\.mode = 'playback'/);
  assert.match(lab, /state\.mode = 'edit'/);
  assert.match(lab, /state\[state\.active\]\[index\] = next/);
  assert.match(lab, /createJointTrajectory\(/);
  assert.match(lab, /state\.final = \[\.\.\.state\.initial\]/);
  assert.match(lab, /\[state\.initial, state\.final\] = \[state\.final, state\.initial\]/);
  assert.match(lab, /timeFrame\(\)\.q/);
  assert.match(lab, /renderJointControls\(\)/);
});

test('playback timeline and displayed rate remain separate from physical T', () => {
  assert.match(lab, /requestAnimationFrame\(tick\)/);
  assert.match(lab, /stopAnimation\(\)/);
  assert.match(lab, /state\.trajectory\.duration \* Number\(el\('timeline'\)\.value\) \/ 1000/);
  assert.match(lab, /elapsed \* speed/);
  assert.match(lab, /visibilitychange/);
  assert.match(html, /Playback speed changes only the display rate/);
});

test('path and endpoint ghosts use FK, and camera uses shared fit', () => {
  assert.match(lab, /forwardKinematicsPosition\(viewer\.dhParams, frame\.q\)/);
  assert.match(lab, /new THREE\.Line\(/);
  assert.match(lab, /ghostGroup\.visible/);
  assert.match(lab, /pathGroup\.visible/);
  assert.match(lab, /fitOrthographicObjects\(/);
  assert.match(lab, /viewer\.controls\.mouseButtons/);
  assert.doesNotMatch(lab, /solveIK/);
});

test('DH topology edits invalidate old endpoint dimensions and trajectories', () => {
  assert.match(lab, /state\.lastTopology !== null && nextTopology !== state\.lastTopology/);
  assert.match(lab, /if \(changed\) \{\s*rebuildTrajectory\(\)/);
  assert.match(lab, /if \(previousTypes\[i\] && previousTypes\[i\] !== joint\.type\) joint\.limits = null/);
  assert.match(lab, /cell\.disabled = true/);
  assert.match(lab, /jointLimits: viewer\.dhParams\.map\(j => j\.limits \?\? null\)/);
  assert.match(lab, /Joint .* must remain within/);
});

test('changing a trajectory parameter recalculates without generating stale frame', () => {
  assert.match(lab, /state\.trajectory = null/);
  assert.match(lab, /clearGroup\(pathGroup\)/);
  assert.match(lab, /renderPoseSelection\(\)/);
  assert.match(lab, /setStatus\('Cannot generate trajectory:/);
  assert.match(lab, /id === 'acceleration-fraction' \? 'input' : 'change'/);
});
