/**
 * Workspace point inspection helpers; no DOM or Three.js dependencies.
 * The sampled XYZ positions use Float32 storage; joint variables use Float64.
 */

/** Retrieve the position and the exact sampled joint vector at zero-based index. */
export function workspaceSampleAt(samples, index) {
  if (!samples || !Number.isSafeInteger(samples.sampleCount) ||
      !Number.isSafeInteger(samples.jointCount) ||
      samples.sampleCount < 1 || samples.jointCount < 1 ||
      !samples.positions || !samples.jointValues ||
      samples.positions.length !== 3 * samples.sampleCount ||
      samples.jointValues.length !== samples.jointCount * samples.sampleCount) {
    throw new TypeError('Invalid workspace sample buffers.');
  }
  if (!Number.isSafeInteger(index) || index < 0 || index >= samples.sampleCount) {
    throw new RangeError('Sample index is out of bounds.');
  }
  return {
    index,
    position: Array.from(samples.positions.subarray(3 * index, 3 * index + 3)),
    jointValues: Array.from(samples.jointValues.subarray(
      samples.jointCount * index, samples.jointCount * (index + 1)
    ))
  };
}

/**
 * Rank Three.js raycast hits using screen-space proximity to the pointer.
 * projectIndex(index) yields a screen-space {x,y,z} where x/y are in CSS
 * pixels and z is camera NDC depth in [-1,1]. The raycast supplies a coarse
 * 3D filter; this step prevents selecting visually distant points when zoomed.
 * Only similarly close hits (within ambiguityPx of the best) are returned,
 * so repeated clicks can cycle overlapping samples.
 */
export function rankProjectedSampleHits(hits, projectIndex, pointer, options = {}) {
  const { radiusPx = 10, ambiguityPx = 2.5, maxCandidates = 24 } = options;
  if (!Array.isArray(hits) || !Number.isFinite(pointer?.x) || !Number.isFinite(pointer?.y) ||
      typeof projectIndex !== 'function' || !(radiusPx > 0) ||
      !(ambiguityPx >= 0) || !Number.isSafeInteger(maxCandidates) ||
      maxCandidates < 1) {
    throw new TypeError('Invalid selection settings.');
  }
  const seen = new Set();
  const ranked = [];
  for (const hit of hits) {
    if (!Number.isSafeInteger(hit?.index) || hit.index < 0 || seen.has(hit.index)) continue;
    seen.add(hit.index);
    const p = projectIndex(hit.index);
    if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y) ||
        !Number.isFinite(p.z) || p.z < -1 || p.z > 1) continue;
    const distance = Math.hypot(p.x - pointer.x, p.y - pointer.y);
    if (distance > radiusPx) continue;
    const rayDepth = Number.isFinite(hit.distance) ? hit.distance : Infinity;
    ranked.push({ index: hit.index, distance, rayDepth });
  }
  ranked.sort((a, b) => a.distance - b.distance ||
    a.rayDepth - b.rayDepth || a.index - b.index);
  if (!ranked.length) return [];
  const tolerance = ranked[0].distance + ambiguityPx;
  return ranked.filter(item => item.distance <= tolerance)
    .slice(0, maxCandidates).map(item => item.index);
}

/** Cycle candidates only when the next click is at the same screen location. */
export function nextSelectedSample(candidates, previousIndex = null, sameSpot = false) {
  if (!Array.isArray(candidates) || candidates.length === 0) return null;
  const previous = sameSpot ? candidates.indexOf(previousIndex) : -1;
  return candidates[previous < 0 ? 0 : (previous + 1) % candidates.length];
}
