import test from 'node:test';
import assert from 'node:assert/strict';
import { createJointTrajectory, JOINT_TRAJECTORY_METHODS } from '../trajectories/joint-trajectory-core.js';
import { buildJointPlotSeries, JOINT_PLOT_METRICS } from '../trajectories/joint-trajectory-plot-data.js';

const close = (actual, expected, epsilon = 1e-7) =>
  assert.ok(Math.abs(actual - expected) < epsilon, String(actual) + ' != ' + String(expected));
const build = (method, extra = {}) => createJointTrajectory({
  initial: [10, 100], final: [90, 250], duration: 4,
  jointTypes: ['R','P'], method, ...extra
});

for (const method of JOINT_TRAJECTORY_METHODS) {
  test(method + ': graph data agrees with analytic evaluator for R and P joints', () => {
    const traj = build(method);
    for (const joint of [0, 1]) {
      const series = buildJointPlotSeries(traj, joint, 81);
      assert.deepEqual(Object.keys(series), JOINT_PLOT_METRICS);
      for (const [metric, field] of [['position','q'],['velocity','dq'],['acceleration','ddq']]) {
        const data = series[metric];
        assert.ok(data.min < data.max);
        assert.ok(data.segments.length >= 1);
        for (const segment of data.segments) {
          assert.ok(segment.length >= 2);
          for (const point of segment) {
            assert.ok(point.time >= 0 && point.time <= traj.duration);
            assert.ok(Number.isFinite(point.value));
            assert.ok(point.value >= data.min && point.value <= data.max);
            if (method !== 'trapezoidal' || metric !== 'acceleration') {
              close(point.value, traj.evaluate(point.time)[field][joint]);
            }
          }
        }
      }
    }
  });
}

test('trapezoidal acceleration shows separate phases without false ramps', () => {
  const traj = createJointTrajectory({
    initial: [0], final: [12], duration: 4,
    method: 'trapezoidal', accelerationFraction: 0.25
  });
  const plots = buildJointPlotSeries(traj, 0);
  assert.equal(plots.position.segments.length, 1);
  assert.equal(plots.velocity.segments.length, 1);
  assert.equal(plots.acceleration.segments.length, 3);
  const [rise, cruise, fall] = plots.acceleration.segments;
  close(rise[0].time, 0); close(rise.at(-1).time, 1);
  close(cruise[0].time, 1); close(cruise.at(-1).time, 3);
  close(fall[0].time, 3); close(fall.at(-1).time, 4);
  for (const point of rise) close(point.value, 4);
  for (const point of cruise) close(point.value, 0);
  for (const point of fall) close(point.value, -4);
  assert.equal(traj.evaluate(1).ddq[0], null);
  assert.equal(traj.evaluate(3).ddq[0], null);
});

test('triangular limit has only two acceleration phases', () => {
  const traj = build('trapezoidal', { accelerationFraction: 0.5 });
  const plots = buildJointPlotSeries(traj, 0);
  assert.equal(plots.acceleration.segments.length, 2);
  close(plots.acceleration.segments[0].at(-1).time, 2);
  close(plots.acceleration.segments[1][0].time, 2);
});

test('constant joint profiles have nondegenerate axes and stay finite at switches', () => {
  const traj = build('trapezoidal', {final:[10,250]});
  const plots = buildJointPlotSeries(traj, 0);
  for(const metric of JOINT_PLOT_METRICS) {
    const p = plots[metric];
    assert.ok(p.max > p.min);
    for(const segment of p.segments) {
      for (const point of segment) assert.ok(Number.isFinite(point.value));
    }
  }
});

test('rejects invalid joint selections and unreasonable sample resolutions', () => {
  const traj = build('cubic');
  for (const idx of [-1, 2, 0.5, NaN]) {
    assert.throws(() => buildJointPlotSeries(traj,idx), RangeError);
  }
  for (const count of [0, 2, 1002, 4.5]) {
    assert.throws(() => buildJointPlotSeries(traj,0,count), RangeError);
  }
  assert.throws(() => buildJointPlotSeries(null,0), TypeError);
});

test('independent series reflect new duration and selected joint', () => {
  const a = build('quintic', {duration:2});
  const b = build('quintic', {duration:8});
  const first = buildJointPlotSeries(a, 0);
  const second = buildJointPlotSeries(b, 0);
  const changedJoint = buildJointPlotSeries(a, 1);
  close(first.position.segments[0].at(-1).time,2);
  close(second.position.segments[0].at(-1).time,8);
  close(first.position.segments[0][0].value,10);
  close(changedJoint.position.segments[0][0].value,100);
  const maxAbs = p => Math.max(...p.segments.flat().map(x=>Math.abs(x.value)));
  close(maxAbs(first.velocity), maxAbs(second.velocity)*4);
  close(maxAbs(first.acceleration), maxAbs(second.acceleration)*16);
});
