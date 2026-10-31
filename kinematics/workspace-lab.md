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

## W3: inspect a reachable sample

After generating the workspace, click or tap a visible point. The sample inspector shows the XYZ position and the original sampled joint vector, then updates the robot's current pose and DH controls without modifying the point cloud or solving IK.

Selection uses a Three.js point raycast with pixel-scaled tolerance and a screen-space nearest-point filter. Repeated clicks at the same location cycle through up to 24 nearly coincident projected samples. As a keyboard-friendly alternative, enter a 1-based sample number and select **Inspect**.

The selected point is highlighted in amber. A subsequent user pose change clears the selection (but not the cloud); changing geometry, topology, limits, domains, sample method or resolution clears both. Hiding the cloud hides the highlight without discarding the inspector values. Clearing the workspace releases point geometry/material and sample buffers.

The reported XYZ values reflect Float32 drawing precision while the recovered joint variables retain Float64 sampling precision. A selected point is reachable by the stored q in the ideal DH model; neither IK nor motion planning is involved.

## Manual review checklist

1. Generate planar 2R (Halton and grid). Orbit, switch views, show/hide samples and fit.
2. Change joint pose only; samples should stay visible. Edit `a` or `alpha`; samples should clear until regenerated.
3. Restrict a sampling domain; regenerate and observe the change.
4. Toggle mechanical limits, change their values, and confirm invalid-domain errors without changing the model silently.
5. Test spatial 3R and RRP; test both dark/light themes and a mobile-width viewport.
6. Change joint type, add/delete DH rows, and confirm bounds are reinitialized with correct units.
7. Click a point to move the robot. Confirm the recovered q, XYZ, sliders and DH table agree without regenerating the workspace.
8. Click repeatedly on dense/overlapping regions; verify cycling, and try a numeric sample index and Enter.
9. Rotate or pan the viewport (no point should be selected by a drag); hide/show the cloud, then change pose or geometry and confirm appropriate selection invalidation.

## Scope/interpretation

The points are **reachable by construction** within the configured finite sampling domains. Gaps are not evidence that a point is unreachable; denser sampling yields better coverage. Revolute defaults without declared mechanical limits cover one turn only. Prismatics without limits need a finite user-defined exploration range.

W3 supports discrete point inspection and recovery. It does **not** calculate exact workspace boundaries, solve IK for arbitrary targets, consider collision, guarantee path feasibility, or enforce orientation constraints. These remain separate work.
