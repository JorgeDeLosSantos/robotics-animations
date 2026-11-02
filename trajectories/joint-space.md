# Joint-Space Trajectories — interactive lab

## Run

Serve the repository root with `python -m http.server 8000`, then open
`http://localhost:8000/trajectories/joint-space.html`.
The page uses Three.js and Tailwind via CDN.

## A: Numerical model

The pure module `joint-trajectory-core.js` generates a straight joint-space
path with shared normalized time law for all R/P joints. See
[joint-trajectory-core.md](./joint-trajectory-core.md) for the mathematical
contract and behavior at discontinuities.

## B: Editing, visualization, playback

Select an example robot or edit its standard-DH geometry, then select **Initial**
or **Final** and adjust the active joint values. Editors use degrees for R and
model length units for P. Mechanical limits are validated, while slider ranges
without limits are only editing conveniences. The trajectory is recalculated on
changes, without a separate Generate button.

Choose linear, cubic, quintic or trapezoidal interpolation and a positive
duration in seconds. Use Play/Pause, Restart, the timeline and 0.5×/1×/2×
visual playback speed. The speed affects *only playback*, not derivatives
computed for physical time. The TCP path and simplified translucent endpoint
poses can be shown/hidden independently.

The camera toolbar stays within the 3D stage. The playback controls and
trajectory charts are docked in separate bands below, so **Fit View** is based
on actual canvas dimensions and top toolbar space.

## C: Synchronized analytic plots

Expand **Trajectory Plots** for position, velocity and acceleration of a selected
joint. All charts use physical time in seconds and units corresponding to the
selected R/P joint: degrees or model units, divided by seconds for derivatives.

A single time cursor follows the 3D motion. Drag inside any plot, or focus it
and use Left/Right arrow keys (1% of the full duration), Home or End to seek.
Seeking pauses playback. Editing the initial or final pose sets the time cursor
to 0 or T, respectively, rather than misleadingly indicating the opposite end.
Selecting a different joint does not modify the trajectory or robot pose.

The acceleration chart splits at trapezoidal acceleration switches; there is
no fictitious diagonal ramp. At an exact switch the instantaneous acceleration
is undefined and shown as such. Linear interpolation shows its internal
constant velocity and zero internal acceleration; endpoint velocity jumps
on joining to rest outside the interval are documented in the numerical core.

Charts are responsive SVG small multiples with a scrollable plot area. Expanding
or contracting the panel updates and refits the Three.js canvas independently
of the window resize event.

## Tests

Run `node --test tests/*.test.mjs`. Specific suites:
- `tests/joint-trajectory-core.test.mjs` (analytic interpolation)
- `tests/joint-trajectory-plot-data.test.mjs` (chart data and jumps)
- `tests/joint-trajectory-lab.test.mjs` (DOM wiring, interaction contracts and syntax)

## Out of scope

Independent time laws per joint, arbitrary boundary conditions, waypoints,
joint velocity/acceleration constrained optimization, collision planning,
dynamic torques and simultaneous multi-method comparison are future work.
