import { MAX_WORKSPACE_SAMPLES } from './workspace-core.js';

const DEFAULT_GRID_TARGET = 25_000;

/**
 * Suggest a useful grid resolution for a robot with n joints, while respecting
 * the same sample-budget ceiling enforced by the pure numerical core.
 *
 * The recommendation scales with dimensionality: roughly 25k total samples,
 * not 25k samples for *each* joint. A maximum is also returned for custom input.
 */
export function gridResolutionSettings(jointCount, targetSamples = DEFAULT_GRID_TARGET) {
  if (!Number.isSafeInteger(jointCount) || jointCount < 1 ||
      !Number.isSafeInteger(targetSamples) || targetSamples < 1) {
    throw new RangeError('Joint count and target sample count must be positive integers.');
  }

  let maxPerJoint = Math.floor(MAX_WORKSPACE_SAMPLES ** (1 / jointCount));
  // Correct floating-point root rounding near exact integer boundaries.
  while ((maxPerJoint + 1) ** jointCount <= MAX_WORKSPACE_SAMPLES) maxPerJoint++;
  while (maxPerJoint ** jointCount > MAX_WORKSPACE_SAMPLES) maxPerJoint--;
  if (maxPerJoint < 2) {
    throw new RangeError('Too many joints for a uniform grid of at least 2 samples per joint.');
  }

  let recommendedPerJoint = Math.max(
    2, Math.floor(Math.min(targetSamples, MAX_WORKSPACE_SAMPLES) ** (1 / jointCount))
  );
  recommendedPerJoint = Math.min(recommendedPerJoint, maxPerJoint);
  return {
    jointCount,
    maxSamples: MAX_WORKSPACE_SAMPLES,
    maxPerJoint,
    recommendedPerJoint,
    recommendedTotal: recommendedPerJoint ** jointCount
  };
}

/** Validate a grid count in its model-dependent feasible range. */
export function gridPointCount(jointCount, samplesPerJoint) {
  const { maxPerJoint } = gridResolutionSettings(jointCount);
  if (!Number.isSafeInteger(samplesPerJoint) ||
      samplesPerJoint < 2 || samplesPerJoint > maxPerJoint) {
    throw new RangeError(
      'Samples per joint must be an integer from 2 to ' + maxPerJoint +
      ' for ' + jointCount + ' joints.'
    );
  }
  return samplesPerJoint ** jointCount;
}
