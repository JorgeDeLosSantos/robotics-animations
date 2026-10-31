import test from 'node:test';
import assert from 'node:assert/strict';
import {
  WORKSPACE_PLANES, projectionLayout, projectPosition, nearestProjectionSamples
} from '../kinematics/workspace-projections.js';
import { summarizeWorkspaceSamples } from '../kinematics/workspace-metrics.js';
import { generateWorkspaceSamples } from '../kinematics/workspace-core.js';

const almost = (actual, expected, eps = 1e-9) =>
  assert.ok(Math.abs(actual - expected) <= eps,
    'Expected ' + actual + ' ~ ' + expected);

const data = {
  positions: new Float32Array([
    -100, -20, -300,
      0,   0,    0,
    100,  20,  300
  ]),
  sampleCount: 3
};
const stats = summarizeWorkspaceSamples(data);

test('XY, XZ, YZ plot expected Cartesian axes without swapping vertical direction', () => {
  assert.deepEqual(WORKSPACE_PLANES.map(p => p.id), ['xy', 'xz', 'yz']);
  for (const [id, expected] of [
    ['xy', ['x', 'y', 0, 1]],
    ['xz', ['x', 'z', 0, 2]],
    ['yz', ['y', 'z', 1, 2]]
  ]) {
    const plane = WORKSPACE_PLANES.find(p => p.id === id);
    assert.deepEqual(
      [plane.horizontal, plane.vertical, plane.hIndex, plane.vIndex],
      expected
    );
    const layout = projectionLayout(stats, plane, 420, 240, 20);
    const origin = projectPosition([0, 0, 0], layout);
    almost(origin.x, 210);
    almost(origin.y, 120);
    const positive = projectPosition([100, 20, 300], layout);
    assert.ok(positive.x > origin.x);
    assert.ok(positive.y < origin.y);
    const negative = projectPosition([-100, -20, -300], layout);
    assert.ok(negative.x < origin.x);
    assert.ok(negative.y > origin.y);
  }
});

test('projection uses the same pixel scale on both axes (no distortion)', () => {
  const layout = projectionLayout(stats, WORKSPACE_PLANES[0], 500, 300);
  const zero = projectPosition([0, 0, 0], layout);
  const ux = projectPosition([10, 0, 0], layout);
  const uy = projectPosition([0, 10, 0], layout);
  almost(ux.x - zero.x, zero.y - uy.y);
});

test('degenerate sampled extents produce a finite centered plot', () => {
  const identical = summarizeWorkspaceSamples({
    positions: new Float32Array([4, -2, 7]), sampleCount: 1
  });
  for (const plane of WORKSPACE_PLANES) {
    const layout = projectionLayout(identical, plane, 250, 160);
    const position = projectPosition([4, -2, 7], layout);
    almost(position.x, 125);
    almost(position.y, 80);
    assert.ok(Number.isFinite(layout.scale));
    assert.ok(layout.scale > 0);
  }
});

test('nearest samples use visible pixel distances and deterministic ties', () => {
  const layout = projectionLayout(stats, WORKSPACE_PLANES[0], 400, 200);
  const pointer = projectPosition([0, 0, 0], layout);
  assert.deepEqual(nearestProjectionSamples(data, layout, pointer, 5), [1]);
  assert.deepEqual(nearestProjectionSamples(data, layout, { x: -10, y: -10 }, 5), []);
  assert.deepEqual(nearestProjectionSamples(data, layout, pointer, 5, 1), [1]);
});

test('overlapping projections retain alternative configurations to cycle', () => {
  const sameXY = {
    positions: new Float32Array([5, 8, 0, 5, 8, 50, 5, 8, -70]),
    sampleCount: 3
  };
  const sameStats = summarizeWorkspaceSamples(sameXY);
  const layout = projectionLayout(sameStats, WORKSPACE_PLANES[0], 360, 220);
  const pointer = projectPosition([5, 8, 0], layout);
  assert.deepEqual(nearestProjectionSamples(sameXY, layout, pointer, 8), [0, 1, 2]);
  assert.deepEqual(nearestProjectionSamples(sameXY, layout, pointer, 8, 2), [0, 1]);
  const xzLayout = projectionLayout(sameStats, WORKSPACE_PLANES[1], 360, 220);
  const xzPointer = projectPosition([5, 8, 50], xzLayout);
  assert.deepEqual(nearestProjectionSamples(sameXY, xzLayout, xzPointer, 2), [1]);
});

test('invalid input is rejected explicitly', () => {
  assert.throws(() => projectionLayout(stats, WORKSPACE_PLANES[0], 0, 100), RangeError);
  assert.throws(() => projectionLayout(stats, {}, 200, 100), RangeError);
  assert.throws(() => projectionLayout(null, WORKSPACE_PLANES[0], 200, 100), TypeError);
  const layout = projectionLayout(stats, WORKSPACE_PLANES[0], 200, 100);
  assert.throws(() => projectPosition([Infinity, 0, 0], layout), RangeError);
  assert.throws(() => nearestProjectionSamples(data, layout, { x: NaN, y: 10 }), TypeError);
});

test('projection input is compatible with sampled joint-space grid results', () => {
  const robot = [
    { type: 'R', a: 2, d: 0, alpha: 0, theta: 0 },
    { type: 'R', a: 1, d: 0, alpha: 0, theta: 0 }
  ];
  const samples = generateWorkspaceSamples(robot, {
    method: 'grid', samplesPerJoint: 5
  });
  const sampledStats = summarizeWorkspaceSamples(samples);
  const layout = projectionLayout(sampledStats, WORKSPACE_PLANES[0], 400, 250);
  const sample = projectPosition(samples.positions.subarray(0, 3), layout);
  assert.ok(Number.isFinite(sample.x) && Number.isFinite(sample.y));
  assert.ok(sample.x >= 0 && sample.x <= 400);
  assert.ok(sample.y >= 0 && sample.y <= 250);
});
