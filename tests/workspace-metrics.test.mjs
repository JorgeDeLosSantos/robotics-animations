import test from 'node:test';
import assert from 'node:assert/strict';
import { summarizeWorkspaceSamples } from '../kinematics/workspace-metrics.js';
import { generateWorkspaceSamples } from '../kinematics/workspace-core.js';

test('computes observed XYZ extents, spans and radial ranges', () => {
  const data = {
    sampleCount: 4,
    positions: new Float32Array([
      -3, 4, 0,
       0, 0, 5,
       2, 0, -6,
       3, 4, 12
    ])
  };
  const stats = summarizeWorkspaceSamples(data);
  assert.deepEqual(stats.x, { min: -3, max: 3, span: 6 });
  assert.deepEqual(stats.y, { min: 0, max: 4, span: 4 });
  assert.deepEqual(stats.z, { min: -6, max: 12, span: 18 });
  assert.deepEqual(stats.xyRadius, { min: 0, max: 5 });
  assert.deepEqual(stats.distance, { min: 5, max: 13 });
  assert.equal(stats.sampleCount, 4);
});

test('one or repeated sample has zero Cartesian spans', () => {
  const stats = summarizeWorkspaceSamples({
    sampleCount: 1, positions: new Float32Array([3, 4, 12])
  });
  assert.equal(stats.x.span, 0);
  assert.equal(stats.y.span, 0);
  assert.equal(stats.z.span, 0);
  assert.equal(stats.xyRadius.min, 5);
  assert.equal(stats.xyRadius.max, 5);
  assert.equal(stats.distance.min, 13);
});

test('rejects empty, inconsistent and nonfinite position buffers', () => {
  assert.throws(() => summarizeWorkspaceSamples(null), TypeError);
  assert.throws(() => summarizeWorkspaceSamples({ sampleCount: 0, positions: [] }), TypeError);
  assert.throws(() => summarizeWorkspaceSamples({ sampleCount: 2, positions: [0, 0, 0] }), TypeError);
  assert.throws(() => summarizeWorkspaceSamples({
    sampleCount: 1, positions: new Float32Array([0, Infinity, 0])
  }), RangeError);
});

test('grid generated for a simple planar 2R shows expected sampled radii', () => {
  const model = [
    { type: 'R', a: 2, alpha: 0, d: 0, theta: 0, limits: { min: 0, max: 360 } },
    { type: 'R', a: 1, alpha: 0, d: 0, theta: 0, limits: { min: 0, max: 180 } }
  ];
  const samples = generateWorkspaceSamples(model, { method: 'grid', samplesPerJoint: 3 });
  const stats = summarizeWorkspaceSamples(samples);
  assert.equal(stats.sampleCount, 9);
  assert.ok(Math.abs(stats.xyRadius.min - 1) < 1e-6);
  assert.ok(Math.abs(stats.xyRadius.max - 3) < 1e-6);
  assert.ok(Math.abs(stats.distance.min - 1) < 1e-6);
  assert.ok(Math.abs(stats.distance.max - 3) < 1e-6);
});
