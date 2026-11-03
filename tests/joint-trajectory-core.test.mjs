import test from 'node:test';
import assert from 'node:assert/strict';
import {
  JOINT_TRAJECTORY_METHODS, evaluateTimeLaw, createJointTrajectory
} from '../trajectories/joint-trajectory-core.js';

const close = (actual, expected, tolerance = 1e-10) =>
  assert.ok(Math.abs(actual - expected) <= tolerance,
    `Expected ${actual} ≈ ${expected} (tolerance ${tolerance})`);
const closeArrays = (actual, expected, tolerance) => {
  assert.equal(actual.length, expected.length);
  actual.forEach((value, i) => close(value, expected[i], tolerance));
};

const joint = (method, extra = {}) => createJointTrajectory({
  initial: [10, -2], final: [90, 4], duration: 4, method,
  jointTypes: ['R', 'P'], ...extra
});

for (const method of JOINT_TRAJECTORY_METHODS) {
  test(`${method}: endpoints and shared progress for mixed R/P joints`, () => {
    const trajectory = joint(method);
    assert.deepEqual(trajectory.evaluate(0).q, [10, -2]);
    assert.deepEqual(trajectory.evaluate(4).q, [90, 4]);
    for (const time of [0, 0.4, 1, 2, 3, 3.6, 4]) {
      const frame = trajectory.evaluate(time);
      close(frame.q[1], -2 + (frame.q[0] - 10) * 6 / 80);
      assert.ok(frame.progress >= 0 && frame.progress <= 1);
      close(frame.q[0], 10 + 80 * frame.progress);
      close(frame.dq[0], (80 / 6) * frame.dq[1]);
    }
    assert.deepEqual(trajectory.jointTypes, ['R', 'P']);
  });
}

test('linear: exact constant internal velocity and zero internal acceleration', () => {
  const trajectory = joint('linear');
  closeArrays(trajectory.evaluate(1).q, [30, -0.5]);
  closeArrays(trajectory.evaluate(1).dq, [20, 1.5]);
  closeArrays(trajectory.evaluate(1).ddq, [0, 0]);
  // At the overall interval endpoints, derivatives are one-sided only.
  closeArrays(trajectory.evaluate(0).dq, [20, 1.5]);
});

test('cubic: zero endpoint velocities and analytic acceleration', () => {
  const trajectory = joint('cubic');
  for (const time of [0, 4]) closeArrays(trajectory.evaluate(time).dq, [0, 0]);
  closeArrays(trajectory.evaluate(0).ddq, [30, 2.25]);
  closeArrays(trajectory.evaluate(4).ddq, [-30, -2.25]);
  closeArrays(trajectory.evaluate(2).q, [50, 1]);
  closeArrays(trajectory.evaluate(2).dq, [30, 2.25]);
  closeArrays(trajectory.evaluate(2).ddq, [0, 0]);
});

test('quintic: zero endpoint velocities and accelerations', () => {
  const trajectory = joint('quintic');
  for (const time of [0, 4]) {
    closeArrays(trajectory.evaluate(time).dq, [0, 0]);
    closeArrays(trajectory.evaluate(time).ddq, [0, 0]);
  }
  closeArrays(trajectory.evaluate(2).q, [50, 1]);
  closeArrays(trajectory.evaluate(2).dq, [37.5, 2.8125]);
  closeArrays(trajectory.evaluate(2).ddq, [0, 0]);
});

test('trapezoidal: acceleration, cruising, deceleration, and switching times', () => {
  const trajectory = createJointTrajectory({
    initial: [0], final: [12], duration: 4, method: 'trapezoidal',
    accelerationFraction: 0.25
  });
  const beginning = trajectory.evaluate(0);
  const rising = trajectory.evaluate(0.5);
  const cruise = trajectory.evaluate(2);
  const falling = trajectory.evaluate(3.5);
  const ending = trajectory.evaluate(4);
  closeArrays(beginning.dq, [0]);
  closeArrays(ending.dq, [0]);
  closeArrays(rising.q, [0.5]);
  closeArrays(rising.dq, [2]);
  closeArrays(rising.ddq, [4]);
  closeArrays(cruise.q, [6]);
  closeArrays(cruise.dq, [4]);
  closeArrays(cruise.ddq, [0]);
  closeArrays(falling.q, [11.5]);
  closeArrays(falling.dq, [2]);
  closeArrays(falling.ddq, [-4]);
  for (const time of [1, 3]) {
    const frame = trajectory.evaluate(time);
    assert.deepEqual(frame.ddq, [null]);
    close(frame.dq[0], 4);
  }
});

test('triangular velocity is the alpha=0.5 trapezoid limit', () => {
  const trajectory = createJointTrajectory({
    initial: [0], final: [2], duration: 2, method: 'trapezoidal',
    accelerationFraction: 0.5
  });
  close(trajectory.evaluate(0.5).q[0], 0.25);
  close(trajectory.evaluate(0.5).dq[0], 1);
  close(trajectory.evaluate(0.5).ddq[0], 2);
  close(trajectory.evaluate(1).q[0], 1);
  close(trajectory.evaluate(1).dq[0], 2);
  assert.equal(trajectory.evaluate(1).ddq[0], null);
  close(trajectory.evaluate(1.5).ddq[0], -2);
});

test('zero-displacement joints remain constant, even at trapezoid switches', () => {
  const trajectory = createJointTrajectory({
    initial: [2, -5], final: [2, 5], duration: 2, method: 'trapezoidal'
  });
  for (const time of [0, 0.5, 1, 1.5, 2]) {
    const { q, dq, ddq } = trajectory.evaluate(time);
    assert.equal(q[0], 2);
    assert.equal(dq[0], 0);
    assert.equal(ddq[0], 0);
  }
  assert.equal(trajectory.evaluate(0.5).ddq[1], null);
});

test('duration scales velocities by 1/T and accelerations by 1/T²', () => {
  for (const method of JOINT_TRAJECTORY_METHODS) {
    const fast = joint(method, { duration: 2 });
    const slow = joint(method, { duration: 8 });
    for (const tau of [0.15, 0.35, 0.7, 0.9]) {
      const a = fast.evaluate(2 * tau);
      const b = slow.evaluate(8 * tau);
      closeArrays(a.q, b.q);
      closeArrays(a.dq, b.dq.map(value => value * 4));
      closeArrays(a.ddq, b.ddq.map(value => value * 16));
    }
  }
});

test('analytic derivatives agree with finite differences away from switches', () => {
  const h = 1e-5;
  for (const method of JOINT_TRAJECTORY_METHODS) {
    const traj = joint(method);
    for (const t of [0.35, 1.4, 2.5, 3.7]) {
      const center = traj.evaluate(t);
      const left = traj.evaluate(t - h);
      const right = traj.evaluate(t + h);
      close(center.dq[0], (right.q[0] - left.q[0]) / (2 * h), 1e-7);
      close(center.ddq[0], (right.dq[0] - left.dq[0]) / (2 * h), 1e-7);
    }
  }
});

test('limits accept endpoints on boundary and reject endpoints outside', () => {
  const limits = [{min: 10, max: 90}, null];
  const trajectory = joint('cubic', {jointLimits: limits});
  assert.deepEqual(trajectory.jointLimits, limits);
  for (const sample of trajectory.sample(51)) {
    assert.ok(sample.q[0] >= 10 && sample.q[0] <= 90);
  }
  assert.throws(() => joint('cubic', {jointLimits: [{min: 11, max: 90}, null]}), RangeError);
  assert.throws(() => joint('cubic', {jointLimits: [{min: 10, max: 89}, null]}), RangeError);
  assert.throws(() => joint('cubic', {jointLimits: [{min: 90, max: 10}, null]}), RangeError);
  assert.throws(() => joint('cubic', {jointLimits: [null]}), TypeError);
});

test('revolute values are not wrapped: 170° to -170° moves -340°', () => {
  const trajectory = createJointTrajectory({
    initial: [170], final: [-170], duration: 2, method: 'cubic', jointTypes: ['R']
  });
  close(trajectory.evaluate(1).q[0], 0);
  close(trajectory.evaluate(1).dq[0], -255);
});

test('inputs and trajectory metadata are not mutated or aliased', () => {
  const initial = [10, -2], final = [90, 4];
  const limits = [{min: 0, max: 100}, null];
  const types = ['R', 'P'];
  const traj = createJointTrajectory({initial, final, duration: 3,
    jointTypes: types, jointLimits: limits});
  initial[0] = -100;
  final[0] = -100;
  limits[0].min = -100;
  types[0] = 'P';
  const first = traj.evaluate(0);
  first.q[0] = -100;
  assert.deepEqual(traj.initial, [10, -2]);
  assert.deepEqual(traj.final, [90, 4]);
  assert.deepEqual(traj.jointTypes, ['R', 'P']);
  assert.deepEqual(traj.jointLimits[0], {min: 0, max: 100});
  assert.equal(traj.evaluate(0).q[0], 10);
  assert.ok(Object.isFrozen(traj));
});

test('sampling includes endpoints, is deterministic, and validates budget', () => {
  const traj = joint('quintic');
  const a = traj.sample(5), b = traj.sample(5);
  assert.deepEqual(a, b);
  assert.deepEqual(a.map(frame => frame.time), [0, 1, 2, 3, 4]);
  assert.deepEqual(a[0].q, [10, -2]);
  assert.deepEqual(a[4].q, [90, 4]);
  for (const count of [0, 1, 2.5, 10_002, Infinity, NaN]) {
    assert.throws(() => traj.sample(count), RangeError);
  }
});

test('rejects invalid methods, durations, time and profile parameters', () => {
  for (const method of ['unknown', null, '']) {
    assert.throws(() => joint(method), RangeError);
    assert.throws(() => evaluateTimeLaw(method, 0, 1), RangeError);
  }
  for (const duration of [0, -1, Infinity, NaN]) {
    assert.throws(() => joint('cubic', {duration}), RangeError);
  }
  const traj = joint('cubic');
  for (const time of [-1, 4.01, Infinity, NaN]) {
    assert.throws(() => traj.evaluate(time), RangeError);
  }
  for (const fraction of [0, -1, 0.51, Infinity, NaN, 1e-320]) {
    assert.throws(() => joint('trapezoidal', {accelerationFraction:fraction}), RangeError);
  }
  assert.throws(() => evaluateTimeLaw('cubic', 1, 2, null), TypeError);
});

test('rejects malformed joint vectors and type annotations', () => {
  assert.throws(() => createJointTrajectory({initial:[], final:[], duration:1}), RangeError);
  assert.throws(() => createJointTrajectory({initial:[0], final:[0,1], duration:1}), RangeError);
  assert.throws(() => createJointTrajectory({initial:[0,NaN], final:[0,1], duration:1}), RangeError);
  assert.throws(() => createJointTrajectory({initial:[0], final:[Infinity], duration:1}), RangeError);
  assert.throws(() => joint('cubic', {jointTypes:['R']}), TypeError);
  assert.throws(() => joint('cubic', {jointTypes:['R','X']}), TypeError);
  assert.throws(() => createJointTrajectory({initial:[1e308], final:[-1e308], duration:1}), RangeError);
});

test('endpoints preserve the exact input values including decimals', () => {
  const traj = createJointTrajectory({initial:[0.1],final:[0.3],duration:2});
  assert.equal(traj.evaluate(0).q[0], 0.1);
  assert.equal(traj.evaluate(2).q[0], 0.3);
});

test('time law is dimensionless and independently usable', () => {
  const cubic = evaluateTimeLaw('cubic', 1, 2);
  close(cubic.progress, 0.5);
  close(cubic.velocity, 0.75);
  close(cubic.acceleration, 0);
  assert.equal(evaluateTimeLaw('trapezoidal', 0.5, 2, {accelerationFraction:0.25}).acceleration, null);
});
