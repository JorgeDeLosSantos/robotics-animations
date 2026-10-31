/**
 * Approximate sections of the sampled positional workspace.
 *
 * The band |p_axis - center| <= thickness/2 filters existing FK samples.
 * It is not an analytic cross section, interpolation, or a guarantee of
 * complete reachability on the requested plane.
 */
export const SECTION_AXES = Object.freeze({
  x: Object.freeze({ id: 'x', index: 0, plane: 'yz', horizontal: 'Y', vertical: 'Z' }),
  y: Object.freeze({ id: 'y', index: 1, plane: 'xz', horizontal: 'X', vertical: 'Z' }),
  z: Object.freeze({ id: 'z', index: 2, plane: 'xy', horizontal: 'X', vertical: 'Y' })
});

export function sectionSettings(metrics, axisId = 'z') {
  const axis = SECTION_AXES[axisId];
  const range = metrics?.[axisId];
  if (!axis || !range || !Number.isFinite(range.min) ||
      !Number.isFinite(range.max) || range.min > range.max) {
    throw new TypeError('Invalid section axis or sampled bounds.');
  }
  const span = range.max - range.min;
  const overallSpan = Math.max(metrics.x.span, metrics.y.span, metrics.z.span, 1e-6);
  // A useful default for dense clouds and even for a planar robot (zero Z span).
  const thickness = Math.max(overallSpan / 25, 1e-6);
  return {
    axis,
    min: range.min,
    max: range.max,
    center: (range.min + range.max) / 2,
    thickness,
    maximumThickness: Math.max(overallSpan, thickness * 2),
    positionStep: Math.max(span / 300, 1e-6)
  };
}

export function sectionContains(samples, axisId, center, thickness, index) {
  const axis = SECTION_AXES[axisId];
  if (!axis || !Number.isFinite(center) || !Number.isFinite(thickness) ||
      thickness <= 0 || !samples || !Number.isSafeInteger(index) ||
      index < 0 || index >= samples.sampleCount) {
    throw new RangeError('Invalid section parameters or sample index.');
  }
  return Math.abs(samples.positions[3 * index + axis.index] - center) <= thickness / 2;
}

export function sliceWorkspaceSamples(samples, axisId, center, thickness) {
  const axis = SECTION_AXES[axisId];
  if (!axis || !Number.isFinite(center) || !Number.isFinite(thickness) ||
      thickness <= 0 || !Number.isSafeInteger(samples?.sampleCount) ||
      samples.sampleCount < 1 || !samples.positions ||
      samples.positions.length !== 3 * samples.sampleCount) {
    throw new RangeError('Invalid section parameters or workspace samples.');
  }
  const selected = new Uint32Array(samples.sampleCount);
  const positions = samples.positions;
  const half = thickness / 2;
  let count = 0;
  for (let i = 0; i < samples.sampleCount; i++) {
    const axisValue = positions[3 * i + axis.index];
    if (Math.abs(axisValue - center) <= half) selected[count++] = i;
  }
  return {
    axisId,
    center,
    thickness,
    indices: selected.slice(0, count),
    count,
    total: samples.sampleCount
  };
}

export function nearestSectionSamples(samples, slice, layout, pointer,
                                      radiusPx = 9, maxResults = 24) {
  if (!slice || !slice.indices || !Number.isSafeInteger(slice.count) ||
      !Number.isFinite(pointer?.x) || !Number.isFinite(pointer?.y) ||
      !Number.isFinite(radiusPx) || radiusPx <= 0 ||
      !Number.isSafeInteger(maxResults) || maxResults < 1) {
    throw new TypeError('Invalid section picking arguments.');
  }
  const { hIndex, vIndex } = layout.plane;
  const { width, height, midU, midV, scale } = layout;
  const coords = samples.positions;
  const radiusSquared = radiusPx * radiusPx;
  const ranked = [];
  for (const index of slice.indices) {
    const x = width / 2 + (coords[3 * index + hIndex] - midU) * scale;
    const y = height / 2 - (coords[3 * index + vIndex] - midV) * scale;
    const d2 = (x - pointer.x) ** 2 + (y - pointer.y) ** 2;
    if (d2 > radiusSquared) continue;
    let j = ranked.length;
    while (j > 0 && (d2 < ranked[j - 1].d2 ||
      (d2 === ranked[j - 1].d2 && index < ranked[j - 1].index))) j--;
    if (j < maxResults) {
      ranked.splice(j, 0, { index, d2 });
      if (ranked.length > maxResults) ranked.pop();
    }
  }
  return ranked.map(hit => hit.index);
}
