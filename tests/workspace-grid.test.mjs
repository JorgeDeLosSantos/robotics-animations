import test from 'node:test';
import assert from 'node:assert/strict';
import { gridResolutionSettings, gridPointCount } from '../kinematics/workspace-grid.js';
import { generateWorkspaceSamples } from '../kinematics/workspace-core.js';

for (const [joints, recommended, max, total] of [
  [2, 158, 447, 24_964],
  [3, 29, 58, 24_389],
  [4, 12, 21, 20_736],
  [5, 7, 11, 16_807],
  [6, 5, 7, 15_625]
]) {
  test('grid defaults for ' + joints + ' joints respect sample budget', () => {
    const settings = gridResolutionSettings(joints);
    assert.equal(settings.recommendedPerJoint, recommended);
    assert.equal(settings.maxPerJoint, max);
    assert.equal(settings.recommendedTotal, total);
    assert.equal(gridPointCount(joints, recommended), total);
    assert.ok(gridPointCount(joints, max) <= settings.maxSamples);
    assert.throws(() => gridPointCount(joints, max + 1), RangeError);
  });
}

test('numeric grid input requires a finite integer and valid resolution', () => {
  for (const invalid of [-1, 0, 1, 2.5, NaN, Infinity, '32']) {
    assert.throws(() => gridPointCount(3, invalid), RangeError);
  }
  assert.throws(() => gridResolutionSettings(0), RangeError);
  assert.throws(() => gridResolutionSettings(2.5), RangeError);
  assert.throws(() => gridResolutionSettings(2, 0), RangeError);
});

test('grid sampling actually produces the recommended total for planar 2R', () => {
  const robot = [
    { type: 'R', a: 220, alpha: 0, d: 0, theta: 0 },
    { type: 'R', a: 160, alpha: 0, d: 0, theta: 0 }
  ];
  const resolution = gridResolutionSettings(robot.length);
  const samples = generateWorkspaceSamples(robot, {
    method: 'grid', samplesPerJoint: resolution.recommendedPerJoint
  });
  assert.equal(samples.sampleCount, resolution.recommendedTotal);
  assert.equal(samples.positions.length, 3 * resolution.recommendedTotal);
  assert.equal(samples.jointValues.length, robot.length * resolution.recommendedTotal);
});

test('joint count changes the suggestion without requiring a global grid resolution', () => {
  const two = gridResolutionSettings(2);
  const three = gridResolutionSettings(3);
  assert.ok(two.recommendedPerJoint > three.recommendedPerJoint);
  assert.ok(two.recommendedTotal > 10_000);
  assert.ok(three.recommendedTotal > 10_000);
});
