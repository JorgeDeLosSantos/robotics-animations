# Workspace core (W1)

Pure numerical core for **reachable position** workspace of serial standard-DH manipulators. No Three.js dependency or mutation of the existing RobotViewer.

Each joint is `{type:'R'|'P', a, alpha, d, theta, limits?:{min,max}|null}`. Angles are in degrees and distances in model units. Limits apply to theta for R or d for P. Unspecified limits do not imply mechanical constraints. Current poses are not clamped.

`resolveSamplingRanges(robot, samplingRanges?)` returns finite per-joint domains, validating they remain within any declared limits. Defaults: declared limits if present; otherwise [-180,180] for R. An unrestricted P joint requires an explicit finite domain. Equal min/max locks a joint.

`forwardKinematicsPosition(robot, q?)` returns [x,y,z]; `generateWorkspaceSamples(robot, {method:'halton'|'grid', sampleCount?, samplesPerJoint?, samplingRanges?})` returns packed `positions` (Float32Array) and corresponding `jointValues` (Float64Array), plus metadata. Halton is deterministic, grid includes endpoints when samplesPerJoint > 1. Maximum sample budget is 200,000.

Each generated point is reachable in the ideal DH model. An unsampled point is not proof of unreachability. No collision checks, pose/orientation constraints, path feasibility, or IK enforcement.

Run: `node --test tests/workspace-core.test.mjs`.
