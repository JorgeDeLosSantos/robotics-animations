/**
 * Numerical foundation for the reachable POSITION workspace of a serial robot.
 *
 * Standard DH convention: Rz(theta) Tz(d) Tx(a) Rx(alpha).
 * Revolute variables are theta in DEGREES; prismatic variables are d in model
 * length units. Geometry and inputs are never mutated. No Three.js dependency.
 *
 * Joint object: { type: 'R'|'P', a, alpha, d, theta, limits?: {min,max}|null }
 * The optional limits refer to the active joint variable, NOT to its DH constants.
 */

const DEG = Math.PI / 180;
const MAX_SAMPLES = 200_000;

/** Validate a finite interval, or return null when limits are unspecified. */
export function validateJointLimits(limits) {
  if (limits == null) return null;
  if (typeof limits !== 'object' || Array.isArray(limits)) {
    throw new TypeError('Joint limits must be {min, max} or null.');
  }
  const { min, max } = limits;
  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    throw new RangeError('Joint limits must contain finite min and max values.');
  }
  if (min > max) throw new RangeError('Joint limits require min <= max.');
  return { min, max };
}

/** Check the existing DH model without changing the current joint values. */
export function validateDhModel(dhParams) {
  if (!Array.isArray(dhParams) || dhParams.length === 0) {
    throw new TypeError('A nonempty array of DH joints is required.');
  }
  dhParams.forEach((joint, i) => {
    if (!joint || (joint.type !== 'R' && joint.type !== 'P')) {
      throw new TypeError(`Joint ${i + 1} must have type R or P.`);
    }
    for (const key of ['a', 'alpha', 'd', 'theta']) {
      if (!Number.isFinite(joint[key])) {
        throw new TypeError(`Joint ${i + 1} must have a finite ${key}.`);
      }
    }
    validateJointLimits(joint.limits);
  });
}

/**
 * Determine a FINITE sampling domain, separate from mechanical limits.
 * - Explicit samplingRanges[i] wins, but must lie inside declared limits.
 * - Otherwise use limits when available.
 * - An unconstrained revolute joint defaults to one full turn [-180, 180].
 * - An unconstrained prismatic joint REQUIRES an explicit sampling range.
 * These conventions define an exploration domain, not new mechanical limits.
 */
export function resolveSamplingRanges(dhParams, samplingRanges = null) {
  validateDhModel(dhParams);
  if (samplingRanges != null &&
      (!Array.isArray(samplingRanges) || samplingRanges.length !== dhParams.length)) {
    throw new TypeError('samplingRanges must match the number of joints.');
  }
  return dhParams.map((joint, i) => {
    const limits = validateJointLimits(joint.limits);
    const proposed = samplingRanges?.[i];
    let range;
    if (proposed != null) {
      range = validateJointLimits(proposed);
    } else if (limits) {
      range = { ...limits };
    } else if (joint.type === 'R') {
      range = { min: -180, max: 180 };
    } else {
      throw new RangeError(
        `Prismatic joint ${i + 1} needs finite limits or a sampling range.`
      );
    }
    if (limits && (range.min < limits.min || range.max > limits.max)) {
      throw new RangeError(`Sampling range for joint ${i + 1} exceeds its limits.`);
    }
    return range;
  });
}

function forwardPositionUnchecked(dhParams, q) {
  // Rotation of the current DH frame relative to the base, row major.
  let r00 = 1, r01 = 0, r02 = 0;
  let r10 = 0, r11 = 1, r12 = 0;
  let r20 = 0, r21 = 0, r22 = 1;
  let px = 0, py = 0, pz = 0;

  for (let i = 0; i < dhParams.length; i++) {
    const joint = dhParams[i];
    const theta = (joint.type === 'R' ? q[i] : joint.theta) * DEG;
    const d = joint.type === 'P' ? q[i] : joint.d;
    const alpha = joint.alpha * DEG;
    const c = Math.cos(theta), s = Math.sin(theta);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);
    const dx = joint.a * c, dy = joint.a * s;

    // Position update p' = p + R * [a*cos(theta), a*sin(theta), d].
    px += r00 * dx + r01 * dy + r02 * d;
    py += r10 * dx + r11 * dy + r12 * d;
    pz += r20 * dx + r21 * dy + r22 * d;

    // Orientation update R' = R * Rz(theta) * Rx(alpha).
    const n00 = r00 * c + r01 * s;
    const n01 = ca * (-r00 * s + r01 * c) + r02 * sa;
    const n02 = sa * (r00 * s - r01 * c) + r02 * ca;
    const n10 = r10 * c + r11 * s;
    const n11 = ca * (-r10 * s + r11 * c) + r12 * sa;
    const n12 = sa * (r10 * s - r11 * c) + r12 * ca;
    const n20 = r20 * c + r21 * s;
    const n21 = ca * (-r20 * s + r21 * c) + r22 * sa;
    const n22 = sa * (r20 * s - r21 * c) + r22 * ca;
    r00 = n00; r01 = n01; r02 = n02;
    r10 = n10; r11 = n11; r12 = n12;
    r20 = n20; r21 = n21; r22 = n22;
  }
  return [px, py, pz];
}

/** Calculate end-effector position (XYZ), optionally at a different q. */
export function forwardKinematicsPosition(dhParams, jointValues = null) {
  validateDhModel(dhParams);
  const q = jointValues ?? dhParams.map(j => j.type === 'R' ? j.theta : j.d);
  if (!Array.isArray(q) && !ArrayBuffer.isView(q)) {
    throw new TypeError('jointValues must be an array or typed array.');
  }
  if (q.length !== dhParams.length || Array.from(q).some(v => !Number.isFinite(v))) {
    throw new RangeError('jointValues must contain one finite value per joint.');
  }
  return forwardPositionUnchecked(dhParams, q);
}

function firstPrimes(count) {
  const primes = [];
  for (let candidate = 2; primes.length < count; candidate++) {
    if (primes.every(p => p * p > candidate || candidate % p !== 0)) {
      primes.push(candidate);
    }
  }
  return primes;
}

function radicalInverse(index, base) {
  let result = 0, fraction = 1 / base;
  while (index > 0) {
    result += (index % base) * fraction;
    index = Math.floor(index / base);
    fraction /= base;
  }
  return result;
}

/**
 * Generate reproducible joint configurations and reachable XYZ positions.
 * method='halton' uses sampleCount; method='grid' uses samplesPerJoint.
 * Returns packed buffers: positions[3*k ... 3*k+2], jointValues[n*k ...].
 * Every output position is reachable by construction; missing points do NOT
 * prove a region unreachable. No joint limits are imposed on the current pose.
 */
export function generateWorkspaceSamples(dhParams, options = {}) {
  const {
    samplingRanges = null,
    method = 'halton',
    sampleCount = 5_000,
    samplesPerJoint = 25,
  } = options;
  const ranges = resolveSamplingRanges(dhParams, samplingRanges);
  const n = ranges.length;
  if (method !== 'halton' && method !== 'grid') {
    throw new RangeError('Sampling method must be halton or grid.');
  }
  if (!Number.isSafeInteger(sampleCount) || sampleCount < 1) {
    throw new RangeError('sampleCount must be a positive integer.');
  }
  if (!Number.isSafeInteger(samplesPerJoint) || samplesPerJoint < 1) {
    throw new RangeError('samplesPerJoint must be a positive integer.');
  }
  const count = method === 'halton' ? sampleCount : samplesPerJoint ** n;
  if (!Number.isSafeInteger(count) || count > MAX_SAMPLES) {
    throw new RangeError(`The sampling budget cannot exceed ${MAX_SAMPLES} points.`);
  }

  const positions = new Float32Array(3 * count);
  const jointValues = new Float64Array(n * count);
  const q = new Float64Array(n);
  const primes = method === 'halton' ? firstPrimes(n) : null;

  for (let k = 0; k < count; k++) {
    let gridIndex = k;
    for (let j = n - 1; j >= 0; j--) {
      const { min, max } = ranges[j];
      let t;
      if (method === 'grid') {
        const digit = gridIndex % samplesPerJoint;
        gridIndex = Math.floor(gridIndex / samplesPerJoint);
        t = samplesPerJoint === 1 ? 0.5 : digit / (samplesPerJoint - 1);
      } else {
        t = radicalInverse(k + 1, primes[j]);
      }
      q[j] = min + (max - min) * t;
      jointValues[k * n + j] = q[j];
    }
    const [x, y, z] = forwardPositionUnchecked(dhParams, q);
    positions.set([x, y, z], 3 * k);
  }
  return { positions, jointValues, ranges, method, sampleCount: count, jointCount: n };
}
