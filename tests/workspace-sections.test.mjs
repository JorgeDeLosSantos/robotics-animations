import test from 'node:test';
import assert from 'node:assert/strict';
import {
  SECTION_AXES, sectionSettings, sliceWorkspaceSamples,
  sectionContains, nearestSectionSamples
} from '../kinematics/workspace-sections.js';
import { WORKSPACE_PLANES, projectionLayout, projectPosition } from '../kinematics/workspace-projections.js';
import { summarizeWorkspaceSamples } from '../kinematics/workspace-metrics.js';
import { generateWorkspaceSamples } from '../kinematics/workspace-core.js';

const data = {
  sampleCount: 5,
  positions: new Float32Array([
    0, 10, -2,
    1, 11, 0,
    2, 12, 0.2,
    3, 13, 1,
    4, 14, 2
  ])
};
const bounds = summarizeWorkspaceSamples(data);

test('slice bands use half-thickness with inclusive boundaries', () => {
  const slice = sliceWorkspaceSamples(data, 'z', 0, 2);
  assert.deepEqual([...slice.indices], [1, 2, 3]);
  assert.equal(slice.count, 3);
  assert.equal(slice.total, 5);
  assert.equal(sectionContains(data, 'z', 0, 2, 3), true);
  assert.equal(sectionContains(data, 'z', 0, 2, 4), false);
});

test('x, y and z axes select expected samples', () => {
  assert.deepEqual([SECTION_AXES.x.plane, SECTION_AXES.y.plane, SECTION_AXES.z.plane],
    ['yz', 'xz', 'xy']);
  assert.deepEqual([...sliceWorkspaceSamples(data, 'x', 2, 2).indices], [1, 2, 3]);
  assert.deepEqual([...sliceWorkspaceSamples(data, 'y', 12, 2).indices], [1, 2, 3]);
  assert.deepEqual([...sliceWorkspaceSamples(data, 'z', -1, 2).indices], [0, 1]);
});

test('empty bands are legitimate, not errors', () => {
  const slice = sliceWorkspaceSamples(data, 'z', 100, 0.01);
  assert.equal(slice.count, 0);
  assert.deepEqual([...slice.indices], []);
});

test('section controls use sampled axis bounds and handle degenerate planar axis', () => {
  const settings = sectionSettings(bounds, 'z');
  assert.equal(settings.min, -2);
  assert.equal(settings.max, 2);
  assert.equal(settings.center, 0);
  assert.ok(settings.thickness > 0);
  const planar = summarizeWorkspaceSamples({
    sampleCount: 2, positions: new Float32Array([-10, 2, 0, 10, 2, 0])
  });
  const zSettings = sectionSettings(planar, 'z');
  assert.equal(zSettings.min, 0);
  assert.equal(zSettings.max, 0);
  assert.ok(zSettings.thickness > 0);
});

test('2D picking considers only samples inside the selected band', () => {
  const slice = sliceWorkspaceSamples(data, 'z', 0, 0.5);
  const plane = WORKSPACE_PLANES.find(p => p.id === 'xy');
  const layout = projectionLayout(bounds, plane, 450, 260);
  const inside = projectPosition([2, 12, 0.2], layout);
  assert.deepEqual(nearestSectionSamples(data, slice, layout, inside, 2), [2]);
  const excluded = projectPosition([4, 14, 2], layout);
  assert.deepEqual(nearestSectionSamples(data, slice, layout, excluded, 2), []);
});

test('near-overlaps in a section return stable original sample indices', () => {
  const samples = {
    sampleCount: 3, positions: new Float32Array([1, 2, -1, 1, 2, 1, 5, 7, 3])
  };
  const metrics = summarizeWorkspaceSamples(samples);
  const slice = sliceWorkspaceSamples(samples, 'z', 0, 2);
  const layout = projectionLayout(metrics, WORKSPACE_PLANES[0], 360, 220);
  const hit = projectPosition([1, 2, -1], layout);
  assert.deepEqual(nearestSectionSamples(samples, slice, layout, hit), [0, 1]);
});

test('invalid axes, band widths and sample indices reject explicitly', () => {
  assert.throws(() => sliceWorkspaceSamples(data, 'w', 0, 1), RangeError);
  assert.throws(() => sliceWorkspaceSamples(data, 'z', 0, 0), RangeError);
  assert.throws(() => sliceWorkspaceSamples(data, 'z', Infinity, 2), RangeError);
  assert.throws(() => sectionContains(data, 'z', 0, 2, -1), RangeError);
  assert.throws(() => sectionSettings(bounds, 'w'), TypeError);
});

test('slicing a 2R grid uses stored positions without rerunning forward kinematics', () => {
  const robot = [
    { type: 'R', a: 2, alpha: 0, d: 0, theta: 0 },
    { type: 'R', a: 1, alpha: 0, d: 0, theta: 0 }
  ];
  const samples = generateWorkspaceSamples(robot, { method: 'grid', samplesPerJoint: 5 });
  const slice = sliceWorkspaceSamples(samples, 'z', 0, 1);
  assert.equal(slice.count, samples.sampleCount);
  assert.equal(slice.indices.length, 25);
});
