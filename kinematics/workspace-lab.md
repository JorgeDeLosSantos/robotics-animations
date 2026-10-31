# Workspace lab (W2)

Interactive **position-reachable** workspace built on W1's pure sampling core.

- SPA: `kinematics/workspace.html`, controller: `workspace-lab.js`.
- Robot presets: planar 2R, spatial 3R and mixed RRP (standard DH).
- Editable DH table, pose sliders, optional mechanical limits, and **separate** finite sampling domains.
- Deterministic Halton or joint-space grid samples; each result is rendered as a single `THREE.Points` geometry separate from `RobotViewer.robotGroup`.
- Samples are invalidated after geometry, topology, limits, domain or sampling-resolution changes; changing the *current pose* leaves the samples untouched.
- The current pose is not silently clamped to mechanical limits. A warning appears if the pose violates them.
- View presets and Fit View preserve the Z-up / XY-horizontal convention.

## Run

From the repository root, run `python -m http.server 8000` and open `http://localhost:8000/kinematics/workspace.html`. The page imports Three.js and Tailwind from CDNs; an internet connection is needed. Run structural checks with `node --test tests/workspace-lab.test.mjs`, and numerical tests with `node --test tests/workspace-core.test.mjs`.

## Manual review checklist

1. Generate planar 2R (Halton and grid). Orbit, switch views, show/hide samples and fit.
2. Change joint pose only; samples should stay visible. Edit `a` or `alpha`; samples should clear until regenerated.
3. Restrict a sampling domain; regenerate and observe the change.
4. Toggle mechanical limits, change their values, and confirm invalid-domain errors without changing the model silently.
5. Test spatial 3R and RRP; test both dark/light themes and a mobile-width viewport.
6. Change joint type, add/delete DH rows, and confirm bounds are reinitialized with correct units.

## Scope/interpretation

The points are **reachable by construction** within the configured finite sampling domains. Gaps are not evidence that a point is unreachable; denser sampling yields better coverage. Revolute defaults without declared mechanical limits cover one turn only. Prismatics without limits need a finite user-defined exploration range.

W2 does **not** select sampled points or recover their joint configurations in the UI (W3), calculate exact workspace boundaries, solve IK, consider collision, or enforce orientation constraints. These will be separate work.
