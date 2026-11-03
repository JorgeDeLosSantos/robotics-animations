/**
 * Pure joint-space trajectory generator for Robotics-Lab.
 *
 * A single scalar progress law s(t) drives a straight joint-space path:
 *     q(t) = initial + (final - initial) * s(t).
 *
 * The values are in the units supplied by the caller (R: degrees at the UI
 * boundary; P: DH model length units). No angular wrapping, FK, or Three.js.
 */

export const JOINT_TRAJECTORY_METHODS = Object.freeze([
  'linear', 'cubic', 'quintic', 'trapezoidal'
]);

const DEFAULT_ACCELERATION_FRACTION = 0.25;
const MAX_TRAJECTORY_SAMPLES = 10_001;
const BREAKPOINT_TOLERANCE = 1e-12;

function finitePositive(value, name) {
  if (!Number.isFinite(value) || value <= 0) {
    throw new RangeError(`${name} must be a finite positive number.`);
  }
}

function validateMethod(method) {
  if (!JOINT_TRAJECTORY_METHODS.includes(method)) {
    throw new RangeError(`Unknown trajectory method: ${String(method)}.`);
  }
}

function validateFraction(value) {
  if (!Number.isFinite(value) || value <= 0 || value > 0.5 ||
      !Number.isFinite(1 / (value * (1 - value)))) {
    throw new RangeError('accelerationFraction must be finite and in (0, 0.5].');
  }
}

/**
 * Evaluate a dimensionless progress law and its TIME derivatives.
 * The returned acceleration is null where a trapezoid changes acceleration
 * discontinuously (there is no classical second derivative at those instants).
 * Elsewhere these values are one-sided at the overall time interval endpoints.
 * Values outside [0, duration] are rejected, not silently extrapolated.
 */
export function evaluateTimeLaw(method, time, duration, options = {}) {
  validateMethod(method);
  finitePositive(duration, 'duration');
  if (!Number.isFinite(time) || time < 0 || time > duration) {
    throw new RangeError('time must be finite and in [0, duration].');
  }
  if (options == null || typeof options !== 'object' || Array.isArray(options)) {
    throw new TypeError('options must be an object.');
  }

  const tau = time / duration;
  let progress, velocity, acceleration;

  if (method === 'linear') {
    progress = tau;
    velocity = 1;
    acceleration = 0;
  } else if (method === 'cubic') {
    progress = tau * tau * (3 - 2 * tau);
    velocity = 6 * tau * (1 - tau);
    acceleration = 6 - 12 * tau;
  } else if (method === 'quintic') {
    progress = tau ** 3 * (10 + tau * (-15 + 6 * tau));
    velocity = 30 * tau * tau * (1 - tau) ** 2;
    acceleration = 60 * tau * (1 - tau) * (1 - 2 * tau);
  } else {
    const alpha = options.accelerationFraction ?? DEFAULT_ACCELERATION_FRACTION;
    validateFraction(alpha);
    const slope = 1 / (alpha * (1 - alpha));
    const cruiseVelocity = 1 / (1 - alpha);
    const firstBreak = Math.abs(tau - alpha) <= BREAKPOINT_TOLERANCE;
    const secondBreak = Math.abs(tau - (1 - alpha)) <= BREAKPOINT_TOLERANCE;

    if (tau < alpha) {
      progress = slope * tau * tau / 2;
      velocity = slope * tau;
      acceleration = slope;
    } else if (tau <= 1 - alpha) {
      progress = cruiseVelocity * (tau - alpha / 2);
      velocity = cruiseVelocity;
      acceleration = 0;
    } else {
      progress = 1 - slope * (1 - tau) ** 2 / 2;
      velocity = slope * (1 - tau);
      acceleration = -slope;
    }
    if (firstBreak || secondBreak) acceleration = null;
  }

  if (time === 0) progress = 0;
  if (time === duration) progress = 1;
  return {
    progress,
    velocity: velocity / duration,
    acceleration: acceleration === null ? null : acceleration / (duration * duration)
  };
}

function copyJointValues(values, label) {
  if (!Array.isArray(values) && !ArrayBuffer.isView(values)) {
    throw new TypeError(`${label} must be an array or typed array.`);
  }
  if (values.length === 0 || Array.from(values).some(x => !Number.isFinite(x))) {
    throw new RangeError(`${label} must be a nonempty array of finite values.`);
  }
  return Object.freeze(Array.from(values));
}

function copyJointTypes(types, count) {
  if (types == null) return null;
  if (!Array.isArray(types) || types.length !== count ||
      types.some(type => type !== 'R' && type !== 'P')) {
    throw new TypeError('jointTypes must contain one R or P type per joint.');
  }
  return Object.freeze([...types]);
}

function copyJointLimits(limits, count) {
  if (limits == null) return null;
  if (!Array.isArray(limits) || limits.length !== count) {
    throw new TypeError('jointLimits must match the number of joints.');
  }
  return Object.freeze(limits.map((limit, i) => {
    if (limit == null) return null;
    if (typeof limit !== 'object' || Array.isArray(limit) ||
        !Number.isFinite(limit.min) || !Number.isFinite(limit.max) ||
        limit.min > limit.max) {
      throw new RangeError(`Joint ${i + 1} limits require finite min <= max.`);
    }
    return Object.freeze({ min: limit.min, max: limit.max });
  }));
}

/**
 * Build an immutable, evaluable two-posture joint trajectory.
 * Inputs and output snapshots are independent copies. Monotonic progress means
 * endpoint limit validation also guarantees interior positions stay in bounds.
 */
export function createJointTrajectory({
  initial, final, duration, method = 'cubic',
  accelerationFraction = DEFAULT_ACCELERATION_FRACTION,
  jointTypes = null, jointLimits = null
} = {}) {
  const start = copyJointValues(initial, 'initial');
  const end = copyJointValues(final, 'final');
  if (end.length !== start.length) {
    throw new RangeError('initial and final must have the same joint count.');
  }
  validateMethod(method);
  finitePositive(duration, 'duration');
  if (method === 'trapezoidal') validateFraction(accelerationFraction);
  const types = copyJointTypes(jointTypes, start.length);
  const limits = copyJointLimits(jointLimits, start.length);
  const delta = Object.freeze(start.map((value, i) => end[i] - value));
  if (delta.some(value => !Number.isFinite(value))) {
    throw new RangeError('Joint displacements must be finite.');
  }
  limits?.forEach((limit, i) => {
    if (!limit) return;
    if (start[i] < limit.min || start[i] > limit.max ||
        end[i] < limit.min || end[i] > limit.max) {
      throw new RangeError(`Joint ${i + 1} trajectory exceeds mechanical limits.`);
    }
  });

  function evaluate(time) {
    const law = evaluateTimeLaw(method, time, duration, { accelerationFraction });
    return {
      time,
      progress: law.progress,
      q: time === 0 ? [...start] : time === duration ? [...end] :
        delta.map((change, i) => start[i] + change * law.progress),
      dq: delta.map(change => change * law.velocity),
      ddq: delta.map(change => change === 0 ? 0 :
        law.acceleration === null ? null : change * law.acceleration)
    };
  }

  function sample(count = 101) {
    if (!Number.isSafeInteger(count) || count < 2 || count > MAX_TRAJECTORY_SAMPLES) {
      throw new RangeError(`sample count must be an integer in [2, ${MAX_TRAJECTORY_SAMPLES}].`);
    }
    return Array.from({ length: count }, (_, i) =>
      evaluate(i === count - 1 ? duration : duration * i / (count - 1)));
  }

  return Object.freeze({
    initial: start, final: end, jointTypes: types, jointLimits: limits,
    duration, method,
    accelerationFraction: method === 'trapezoidal' ? accelerationFraction : null,
    evaluate, sample
  });
}
