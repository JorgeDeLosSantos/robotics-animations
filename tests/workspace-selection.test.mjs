import test from 'node:test';
import assert from 'node:assert/strict';
import { generateWorkspaceSamples, forwardKinematicsPosition } from '../kinematics/workspace-core.js';
import { workspaceSampleAt, rankProjectedSampleHits, nextSelectedSample }
  from '../kinematics/workspace-selection.js';

const close = (a, b, tolerance = 1e-4) =>
  assert.ok(Math.abs(a - b) <= tolerance, 'Expected ' + a + ' ~ ' + b);
const planar = [
  { type: 'R', a: 220, alpha: 0, d: 0, theta: 30, limits: { min: -180, max: 180 } },
  { type: 'R', a: 160, alpha: 0, d: 0, theta: -45, limits: { min: -150, max: 150 } }
];

test('recovers sampled q and XYZ for a planar 2R without solving IK', () => {
  const dataset = generateWorkspaceSamples(planar, { sampleCount: 97 });
  for (const i of [0, 11, 42, 96]) {
    const sample = workspaceSampleAt(dataset, i);
    assert.equal(sample.index, i);
    assert.equal(sample.jointValues.length, 2);
    const xyz = forwardKinematicsPosition(planar, sample.jointValues);
    xyz.forEach((value, j) => close(sample.position[j], value));
  }
});

test('recovers revolute and prismatic values in their original units', () => {
  const robot = [
    { type: 'R', a: 0, alpha: 90, d: 0, theta: 0, limits: { min: 0, max: 0 } },
    { type: 'P', a: 0, alpha: 0, d: 0, theta: 0, limits: { min: 0, max: 200 } }
  ];
  const samples = generateWorkspaceSamples(robot, { method: 'grid', samplesPerJoint: 3 });
  const selected = workspaceSampleAt(samples, 7);
  assert.equal(selected.jointValues[0], 0);
  assert.equal(selected.jointValues[1], 100);
  close(selected.position[1], -100);
});

test('rejects bad buffers, noninteger indices and out-of-range indices', () => {
  const samples = generateWorkspaceSamples(planar, { sampleCount: 5 });
  assert.throws(() => workspaceSampleAt(samples, -1), RangeError);
  assert.throws(() => workspaceSampleAt(samples, 5), RangeError);
  assert.throws(() => workspaceSampleAt(samples, 2.5), RangeError);
  assert.throws(() => workspaceSampleAt({ ...samples, jointCount: 4 }, 0), TypeError);
  assert.throws(() => workspaceSampleAt(null, 0), TypeError);
});

test('ranks nearby projected hits by pixel distance (not raycast order)', () => {
  const projected = {
    2: { x: 107, y: 100, z: 0 },
    5: { x: 100.8, y: 99.7, z: 0.5 },
    8: { x: 100, y: 100, z: -2 },
    9: { x: 151, y: 100, z: 0.2 }
  };
  const hits = [
    { index: 2, distance: 2 }, { index: 9, distance: 1 },
    { index: 8, distance: 1 }, { index: 5, distance: 8 }
  ];
  assert.deepEqual(rankProjectedSampleHits(hits, i => projected[i], { x: 100, y: 100 }), [5]);
});

test('overlaps are ordered deterministically with depth as tie-breaker', () => {
  const hits = [{ index: 7, distance: 8 }, { index: 1, distance: 3 },
    { index: 3, distance: 5 }, { index: 1, distance: 3 }];
  const projected = {
    1: { x: 100, y: 100, z: 0.2 },
    3: { x: 101.5, y: 100, z: 0.1 },
    7: { x: 100, y: 100, z: 0.3 }
  };
  assert.deepEqual(rankProjectedSampleHits(hits, i => projected[i], { x: 100, y: 100 }), [1,7,3]);
});

test('picking returns none when point is farther than the pixel tolerance', () => {
  const hits = [{ index: 4, distance: 1 }];
  const position = () => ({ x: 30, y: 20, z: 0 });
  assert.deepEqual(rankProjectedSampleHits(hits, position, { x: 100, y: 100 }), []);
  assert.deepEqual(rankProjectedSampleHits([], position, { x: 100, y: 100 }), []);
});

test('same-position clicks cycle; moving the pointer resets to nearest', () => {
  const candidates = [3, 9, 12];
  assert.equal(nextSelectedSample(candidates), 3);
  assert.equal(nextSelectedSample(candidates, 3, true), 9);
  assert.equal(nextSelectedSample(candidates, 12, true), 3);
  assert.equal(nextSelectedSample(candidates, 9, false), 3);
  assert.equal(nextSelectedSample([], 9, true), null);
});
