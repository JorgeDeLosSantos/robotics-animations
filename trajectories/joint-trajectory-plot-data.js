/**
 * Pure chart-series adapter for an evaluable joint-space trajectory.
 * Plot values come from the same analytic trajectory evaluator used by 3D playback.
 * Trapezoidal acceleration is split into separate segments at its jump times.
 */
const METRICS = Object.freeze({ position: 'q', velocity: 'dq', acceleration: 'ddq' });
export const JOINT_PLOT_METRICS = Object.freeze(Object.keys(METRICS));

function plotLimits(segments) {
  const values = segments.flatMap(segment => segment.map(point => point.value))
    .filter(Number.isFinite);
  if (!values.length) return { min: -1, max: 1 };
  const min = Math.min(...values), max = Math.max(...values);
  const span = max - min;
  const padding = span > 0 ? span * 0.12 : Math.max(Math.abs(min) * 0.1, 0.1);
  return { min: min - padding, max: max + padding };
}

export function buildJointPlotSeries(trajectory, jointIndex, sampleCount = 121) {
  if (!trajectory || typeof trajectory.evaluate !== 'function' ||
      !Array.isArray(trajectory.initial) ||
      !Number.isFinite(trajectory.duration) || trajectory.duration <= 0) {
    throw new TypeError('An evaluable joint trajectory is required.');
  }
  if (!Number.isSafeInteger(jointIndex) ||
      jointIndex < 0 || jointIndex >= trajectory.initial.length) {
    throw new RangeError('jointIndex must identify an existing joint.');
  }
  if (!Number.isSafeInteger(sampleCount) || sampleCount < 3 || sampleCount > 1001) {
    throw new RangeError('sampleCount must be an integer in [3, 1001].');
  }
  const duration = trajectory.duration;
  const sampled = trajectory.sample(sampleCount);
  const series = {};
  for (const [metric, field] of Object.entries(METRICS)) {
    let segments = [sampled.map(frame => ({
      time: frame.time, value: frame[field][jointIndex]
    })).filter(point => Number.isFinite(point.value))];

    if (metric === 'acceleration' && trajectory.method === 'trapezoidal') {
      const a = trajectory.accelerationFraction * duration;
      const breaks = [...new Set([0, a, duration - a, duration])].sort((x, y) => x - y);
      segments = [];
      for (let k = 0; k + 1 < breaks.length; k++) {
        const start = breaks[k], end = breaks[k + 1];
        if (end <= start) continue;
        // Sample INSIDE each phase at one-sided endpoints. Keep the actual
        // boundary x coordinate and do not draw across any acceleration jump.
        const offset = Math.min((end - start) / 1000, duration * 1e-8);
        const points = [];
        for (let i = 0; i < 5; i++) {
          const time = start + (end - start) * i / 4;
          const probe = i === 0 && start > 0 ? start + offset
            : i === 4 && end < duration ? end - offset : time;
          const value = trajectory.evaluate(probe).ddq[jointIndex];
          if (Number.isFinite(value)) points.push({ time, value });
        }
        if (points.length) segments.push(points);
      }
    }
    series[metric] = { segments, ...plotLimits(segments) };
  }
  return series;
}
