# Joint-space trajectory core — Stage A

Pure ES module without Three.js, browser APIs or robot geometry dependencies.

## Model

For initial and final configurations of equal dimension:

- `q(t) = q0 + (qf - q0) s(t)`
- `dq(t) = (qf - q0) ds/dt`
- `ddq(t) = (qf - q0) d²s/dt²`

All joints share one scalar time law. The joint-space path is a straight line.
The module does not wrap angles, compute FK, solve IK or enforce velocity and acceleration limits.

Inputs remain in caller units: revolute degrees, prismatic model length units,
and time in seconds. Their derivatives use corresponding units per second and
per second squared.

## API

```js
import { createJointTrajectory } from './joint-trajectory-core.js';

const trajectory = createJointTrajectory({
  initial: [0, 10],
  final: [90, 40],
  duration: 2,
  method: 'quintic', // linear | cubic | quintic | trapezoidal
  jointTypes: ['R', 'P'],
  jointLimits: [{min: -180, max: 180}, null],
  // accelerationFraction: 0.25 // trapezoidal only
});

trajectory.evaluate(1); // {time, progress, q, dq, ddq}
trajectory.sample(101); // includes t=0 and t=duration
```

`evaluateTimeLaw(method, time, duration, options?)` independently returns
`{progress, velocity, acceleration}` where derivatives are with respect to
physical seconds rather than normalized time.

Arrays are copied on input and output; the trajectory object and its metadata
are frozen. Time outside `[0,T]` is rejected. Samples must number 2–10,001.
Mechanical joint limits, if provided, constrain both endpoints; no clamping
occurs. Monotonic progress ensures the interior remains between the endpoints.

## Time laws

With `tau=t/T`:

- Linear: `s=tau`. Velocity is constant inside the interval; joining to rest
  outside the interval introduces velocity jumps.
- Cubic: `s=3tau²-2tau³`. Endpoint velocities vanish, while one-sided endpoint
  accelerations generally do not.
- Quintic: `s=10tau³-15tau⁴+6tau⁵`. Endpoint velocities and accelerations vanish.
- Trapezoidal: equal acceleration/deceleration phases each lasting
  `accelerationFraction*T`, separated by constant velocity. The default
  fraction is 0.25, valid range `(0,0.5]`. A value of 0.5 gives a triangular
  velocity profile. At the phase switches, acceleration is discontinuous:
  the core reports `null` instead of an arbitrary value, except for stationary
  joints where acceleration remains zero.

Endpoint derivatives are one-sided derivatives from *inside* `[0,T]`.

## Testing

```bash
node --test tests/joint-trajectory-core.test.mjs
node --test tests/*.test.mjs
```

Stage B integrates the editor and 3D playback; Stage C adds synchronized plots.
