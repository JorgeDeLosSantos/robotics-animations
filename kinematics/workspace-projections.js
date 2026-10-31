/**
 * Pure Cartesian projections of a sampled positional workspace.
 *
 * XY: horizontal X, vertical Y; XZ: horizontal X, vertical Z;
 * YZ: horizontal Y, vertical Z. Vertical axes point upwards.
 * All plots use an isotropic scale and only display sampled positions:
 * they are projections, not planar cross-sections or analytic boundaries.
 */
export const WORKSPACE_PLANES = Object.freeze([
  Object.freeze({ id: 'xy', horizontal: 'x', vertical: 'y', hIndex: 0, vIndex: 1 }),
  Object.freeze({ id: 'xz', horizontal: 'x', vertical: 'z', hIndex: 0, vIndex: 2 }),
  Object.freeze({ id: 'yz', horizontal: 'y', vertical: 'z', hIndex: 1, vIndex: 2 })
]);

export function projectionLayout(metrics, plane, width, height, padding = 18) {
  if (!WORKSPACE_PLANES.includes(plane) ||
      !Number.isFinite(width) || !Number.isFinite(height) ||
      !Number.isFinite(padding) || padding < 0 ||
      width <= 2 * padding || height <= 2 * padding) {
    throw new RangeError('Invalid workspace projection dimensions or plane.');
  }
  const u = metrics?.[plane.horizontal];
  const v = metrics?.[plane.vertical];
  if (!u || !v || ![u.min,u.max,v.min,v.max].every(Number.isFinite) ||
      u.min > u.max || v.min > v.max) {
    throw new TypeError('Invalid sampled Cartesian bounds.');
  }

  const spanU = u.max - u.min;
  const spanV = v.max - v.min;
  const widthAvailable = width - 2 * padding;
  const heightAvailable = height - 2 * padding;
  const scale = Math.min(
    widthAvailable / (spanU || 1),
    heightAvailable / (spanV || 1)
  );
  return {
    plane,
    width,
    height,
    scale,
    midU: (u.min + u.max) / 2,
    midV: (v.min + v.max) / 2
  };
}

export function projectPosition(position, layout) {
  const { plane, width, height, scale, midU, midV } = layout;
  const u = position[plane.hIndex];
  const v = position[plane.vIndex];
  if (!Number.isFinite(u) || !Number.isFinite(v)) {
    throw new RangeError('Projected position must be finite.');
  }
  return {
    x: width / 2 + (u - midU) * scale,
    y: height / 2 - (v - midV) * scale
  };
}

/**
 * Find visible sampled positions close to a click in CSS pixels.
 * Intentionally checks actual points, not an interpolated or filled region.
 * Keep only a small bounded set of closest indices for overlap cycling.
 */
export function nearestProjectionSamples(samples, layout, pointer, radiusPx = 8, maxResults = 24) {
  if (!samples || !Number.isSafeInteger(samples.sampleCount) || samples.sampleCount < 1 ||
      !samples.positions || samples.positions.length !== 3 * samples.sampleCount ||
      !Number.isFinite(pointer?.x) || !Number.isFinite(pointer?.y) ||
      !Number.isFinite(radiusPx) || radiusPx <= 0 ||
      !Number.isSafeInteger(maxResults) || maxResults < 1) {
    throw new TypeError('Invalid projected sample selection data.');
  }
  const { hIndex, vIndex } = layout.plane;
  const { width, height, midU, midV, scale } = layout;
  const r2 = radiusPx ** 2;
  const picked = [];
  const positions = samples.positions;
  for (let i = 0; i < samples.sampleCount; i++) {
    const x = width / 2 + (positions[3 * i + hIndex] - midU) * scale;
    const y = height / 2 - (positions[3 * i + vIndex] - midV) * scale;
    const d2 = (x - pointer.x) ** 2 + (y - pointer.y) ** 2;
    if (d2 > r2) continue;
    const candidate = { index: i, d2 };
    // Ordered insertion maintains a bounded candidate list even at dense overlaps.
    let pos = picked.length;
    while (pos > 0 && (
      d2 < picked[pos - 1].d2 ||
      (d2 === picked[pos - 1].d2 && i < picked[pos - 1].index)
    )) pos--;
    if (pos < maxResults) {
      picked.splice(pos, 0, candidate);
      if (picked.length > maxResults) picked.pop();
    }
  }
  return picked.map(item => item.index);
}
