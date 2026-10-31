/**
 * Numerical summary of finite, sampled end-effector positions.
 *
 * These are observed extrema of sampled Cartesian points, NOT analytic
 * workspace bounds. No convex hull, volume or reachability guarantees.
 */
export function summarizeWorkspaceSamples(samples) {
  const positions = samples?.positions;
  const count = samples?.sampleCount;
  if (!Number.isSafeInteger(count) || count <= 0 ||
      !positions || positions.length !== 3 * count) {
    throw new TypeError('Expected nonempty, packed XYZ workspace samples.');
  }

  const xRange = { min: Infinity, max: -Infinity };
  const yRange = { min: Infinity, max: -Infinity };
  const zRange = { min: Infinity, max: -Infinity };
  const xyRadius = { min: Infinity, max: -Infinity };
  const distance = { min: Infinity, max: -Infinity };

  for (let k = 0; k < count; k++) {
    const x = positions[3 * k];
    const y = positions[3 * k + 1];
    const z = positions[3 * k + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      throw new RangeError('Workspace positions must be finite.');
    }
    xRange.min = Math.min(xRange.min, x);
    xRange.max = Math.max(xRange.max, x);
    yRange.min = Math.min(yRange.min, y);
    yRange.max = Math.max(yRange.max, y);
    zRange.min = Math.min(zRange.min, z);
    zRange.max = Math.max(zRange.max, z);
    const radius = Math.hypot(x, y);
    const radialDistance = Math.hypot(x, y, z);
    xyRadius.min = Math.min(xyRadius.min, radius);
    xyRadius.max = Math.max(xyRadius.max, radius);
    distance.min = Math.min(distance.min, radialDistance);
    distance.max = Math.max(distance.max, radialDistance);
  }

  for (const range of [xRange, yRange, zRange]) {
    range.span = range.max - range.min;
  }
  return {
    sampleCount: count,
    x: xRange,
    y: yRange,
    z: zRange,
    xyRadius,
    distance
  };
}
