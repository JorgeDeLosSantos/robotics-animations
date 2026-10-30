/**
 * ik-solvers.js
 * Pure inverse kinematics solver functions for robot manipulators.
 * These functions operate on a RobotViewer instance and modify its dhParams.
 */
import * as THREE from 'three';

// ---------------------------------------------------------------------------
// Helper: solve a 6x6 linear system via Gaussian elimination with pivoting
// ---------------------------------------------------------------------------
function gaussianElimination6x6(A, b) {
    const n = 6;
    for (let i = 0; i < n; i++) {
        let maxEl = Math.abs(A[i][i]);
        let maxRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(A[k][i]) > maxEl) {
                maxEl = Math.abs(A[k][i]);
                maxRow = k;
            }
        }
        for (let k = i; k < n; k++) {
            const tmp = A[maxRow][k]; A[maxRow][k] = A[i][k]; A[i][k] = tmp;
        }
        let tmp = b[maxRow]; b[maxRow] = b[i]; b[i] = tmp;

        if (Math.abs(A[i][i]) < 1e-6) return null; // singular

        for (let k = i + 1; k < n; k++) {
            const c = -A[k][i] / A[i][i];
            for (let j = i; j < n; j++) {
                if (i === j) A[k][j] = 0;
                else A[k][j] += c * A[i][j];
            }
            b[k] += c * b[i];
        }
    }
    const x = new Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) {
        x[i] = b[i] / A[i][i];
        for (let k = i - 1; k >= 0; k--) b[k] -= A[k][i] * x[i];
    }
    return x;
}

// ---------------------------------------------------------------------------
// IK Solver: Position only using Cyclic Coordinate Descent (CCD)
// ---------------------------------------------------------------------------
/**
 * Position-only CCD solver for serial chains with revolute (R) and
 * prismatic (P) joints, using standard DH parameters.
 *
 * The detailed API reports convergence separately from reachability.
 * Failure to converge does NOT establish that a target is unreachable.
 * The manipulator is left at the best configuration encountered.
 *
 * @param {RobotViewer} viewer - Mutable serial-chain model
 * @param {THREE.Vector3} target - Desired end-effector position
 * @param {Object} [options]
 * @param {number} [options.maxIterations=200] - Maximum full CCD sweeps
 * @param {number} [options.tolerance=0.5] - Position tolerance in model units
 * @param {Function} [options.onStep] - Optional callback after a joint update:
 *   {iteration, jointIndex, jointType, delta, values, error}.
 *   Revolute delta is in degrees; prismatic delta is in model length units.
 *   values contains theta for R and d for P. For backward compatibility,
 *   callbacks also receive angles (all theta values) and deltaDegrees for R.
 *   Each iteration denotes one full sweep from the last joint to the first.
 *   No callbacks are emitted for joints that cannot move or have zero update.
 *   The ideal prismatic joints have unrestricted travel in this version.
 * @returns {{solved:boolean,status:string,iterations:number,error:number,initialError:number}}
 */
export function solveIKPositionCCDDetailed(viewer, target, options = {}) {
    const maxIterations = Number.isFinite(options.maxIterations)
        ? Math.max(1, Math.floor(options.maxIterations)) : 200;
    const tolerance = Number.isFinite(options.tolerance) && options.tolerance > 0
        ? options.tolerance : 0.5;
    const initialError = viewer.getEEPosition().distanceTo(target);
    if (viewer.dhParams.some(joint => joint.type !== 'R' && joint.type !== 'P')) {
        return { solved: false, status: 'unsupported_joint', iterations: 0,
            error: initialError, initialError };
    }

    let bestError = initialError;
    const getValues = () => viewer.dhParams.map(joint =>
        joint.type === 'P' ? joint.d : joint.theta);
    let bestValues = getValues();
    if (initialError <= tolerance) {
        return { solved: true, status: 'converged', iterations: 0,
            error: initialError, initialError };
    }

    let iterations = 0;
    let stallCount = 0;
    let lastError = initialError;
    let status = 'max_iterations';

    for (let iter = 0; iter < maxIterations; iter++) {
        for (let i = viewer.dhParams.length - 1; i >= 0; i--) {
            const joint = viewer.dhParams[i];
            const eePos = viewer.getEEPosition();
            const { zAxes } = viewer.getJointStates(i);
            const axis = zAxes[i];
            let delta = 0;

            if (joint.type === 'P') {
                // Moving d_i translates the end-effector along z_{i-1}.
                // Projecting the current positional error onto this unit
                // axis exactly minimizes the error with other joints fixed.
                const error = new THREE.Vector3().subVectors(target, eePos);
                delta = error.dot(axis); // model length units
                if (Math.abs(delta) < 1e-12) continue;
                joint.d += delta;
            } else {
                const jointPos = viewer.getJointPosition(i);
                // R joint: align the two projected directions by rotating
                // about z_{i-1}, leaving the axial component unchanged.
                const toEE = new THREE.Vector3().subVectors(eePos, jointPos)
                    .projectOnPlane(axis);
                const toTarget = new THREE.Vector3().subVectors(target, jointPos)
                    .projectOnPlane(axis);
                if (toEE.lengthSq() < 1e-12 || toTarget.lengthSq() < 1e-12) continue;

                toEE.normalize();
                toTarget.normalize();
                const sine = new THREE.Vector3().crossVectors(toEE, toTarget).dot(axis);
                const cosine = Math.max(-1, Math.min(1, toEE.dot(toTarget)));
                const angle = Math.atan2(sine, cosine);
                delta = THREE.MathUtils.radToDeg(angle);
                if (Math.abs(angle) < 1e-12) continue;

                const theta = joint.theta + delta;
                joint.theta = ((theta + 180) % 360 + 360) % 360 - 180;
            }

            if (typeof options.onStep === 'function') {
                options.onStep({
                    iteration: iter + 1,
                    jointIndex: i,
                    jointType: joint.type,
                    delta,
                    // A snapshot allows the interface to replay real CCD
                    // updates, including the displacement of prismatic joints.
                    values: getValues(),
                    // Keep existing R-only animation callbacks compatible.
                    angles: viewer.dhParams.map(joint => joint.theta),
                    deltaDegrees: joint.type === 'R' ? delta : null,
                    error: viewer.getEEPosition().distanceTo(target)
                });
            }
        }

        iterations = iter + 1;
        const error = viewer.getEEPosition().distanceTo(target);
        if (error < bestError) {
            bestError = error;
            bestValues = getValues();
        }
        if (bestError <= tolerance) {
            status = 'converged';
            break;
        }
        // Stop when full sweeps cease improving the position appreciably.
        // This may be a stationary or numerically difficult configuration,
        // not necessarily an unreachable target.
        stallCount = Math.abs(lastError - error) < 1e-9 ? stallCount + 1 : 0;
        if (stallCount >= 10) {
            status = 'stalled';
            break;
        }
        lastError = error;
    }

    viewer.dhParams.forEach((joint, index) => {
        if (joint.type === 'P') joint.d = bestValues[index];
        else joint.theta = bestValues[index];
    });
    return { solved: bestError <= tolerance, status, iterations,
        error: bestError, initialError };
}

/**
 * Backward-compatible position IK API returning only the final error.
 * For convergence diagnostics use solveIKPositionCCDDetailed().
 */
export function solveIKPositionCCD(viewer, target, options = {}) {
    return solveIKPositionCCDDetailed(viewer, target, options).error;
}

// ---------------------------------------------------------------------------
// IK Solver: Full pose (position + orientation) using Damped Least Squares
// ---------------------------------------------------------------------------

/**
 * Pose error expressed in the base/world frame. The shortest quaternion
 * difference gives a rotation vector with magnitude in [0, pi], including
 * valid 180-degree rotations (unlike a sum of cross products of axes).
 *
 * The rotation vector is the axis-angle logarithm of
 * q_target * inverse(q_current). Its norm is the angular discrepancy.
 *
 * @returns {{position:THREE.Vector3,rotation:THREE.Vector3,
 *            pErr:number,rErr:number}}
 */
export function measureIKPoseError(eeMatrix, targetPos, targetQuat) {
    const currentPos = new THREE.Vector3().setFromMatrixPosition(eeMatrix);
    const position = new THREE.Vector3().subVectors(targetPos, currentPos);
    const currentQuat = new THREE.Quaternion().setFromRotationMatrix(eeMatrix).normalize();
    const difference = targetQuat.clone().normalize()
        .multiply(currentQuat.invert()).normalize();
    // q and -q represent the same orientation. Choose the shorter rotation.
    if (difference.w < 0) {
        difference.set(-difference.x, -difference.y, -difference.z, -difference.w);
    }
    const sineHalfAngle = Math.hypot(difference.x, difference.y, difference.z);
    const angle = 2 * Math.atan2(sineHalfAngle, Math.max(0, difference.w));
    const rotation = new THREE.Vector3(difference.x, difference.y, difference.z);
    if (sineHalfAngle > 1e-12) rotation.multiplyScalar(angle / sineHalfAngle);
    else rotation.set(0, 0, 0);
    return { position, rotation, pErr: position.length(), rErr: angle };
}

/**
 * Solve full pose IK for a chain of six or more revolute joints.
 *
 * Translation is divided by characteristicLength in both J_v and the
 * position error to compare translation and rotation on a common scale.
 * The damping and step limit are thus dimensionless/angular quantities.
 * Backtracking accepts only improvements in the combined pose residual.
 * No mechanical joint limits or collision avoidance are imposed.
 *
 * The solver updates viewer.dhParams and returns the best pose reached.
 * A nonconverged status does not prove that the pose is unreachable.
 *
 * @param {RobotViewer} viewer
 * @param {THREE.Vector3} targetPos
 * @param {THREE.Quaternion} targetQuat
 * @param {Object} [options]
 * @param {number} [options.maxIterations=500]
 * @param {number} [options.learningRate=0.8]
 * @param {number} [options.damping=0.08]
 * @param {number} [options.posTolerance=0.5] - Model length units
 * @param {number} [options.rotTolerance=0.008726646259971648] - Radians (0.5 deg)
 * @param {number} [options.characteristicLength] - Model length units
 * @param {number} [options.maxStepDegrees=12] - Maximum change per joint per iteration
 * @returns {{solved:boolean,status:string,iterations:number,pErr:number,
 *            rErr:number,initialPErr:number,initialRErr:number}}
 */
export function solveIKFull6DoF(viewer, targetPos, targetQuat, options = {}) {
    const maxIterations = Number.isFinite(options.maxIterations)
        ? Math.max(1, Math.floor(options.maxIterations)) : 500;
    const learningRate = Number.isFinite(options.learningRate) && options.learningRate > 0
        ? options.learningRate : 0.8;
    const damping = Number.isFinite(options.damping) && options.damping > 0
        ? options.damping : 0.08;
    const posTolerance = Number.isFinite(options.posTolerance) && options.posTolerance > 0
        ? options.posTolerance : 0.5;
    const rotTolerance = Number.isFinite(options.rotTolerance) && options.rotTolerance > 0
        ? options.rotTolerance : THREE.MathUtils.degToRad(0.5);
    const characteristicLength = Number.isFinite(options.characteristicLength)
        && options.characteristicLength > 0 ? options.characteristicLength
        : Math.max(100, viewer.dhParams.reduce((sum, joint) =>
            sum + Math.abs(joint.a) + Math.abs(joint.d), 0) / 3);
    const maxStep = THREE.MathUtils.degToRad(
        Number.isFinite(options.maxStepDegrees) && options.maxStepDegrees > 0
            ? options.maxStepDegrees : 12
    );
    const targetOrientation = targetQuat.clone().normalize();
    const evaluation = () => {
        const { matrices, eeMatrix } = viewer.computeForwardKinematics();
        const error = measureIKPoseError(eeMatrix, targetPos, targetOrientation);
        const residual = Math.hypot(error.pErr / characteristicLength, error.rErr);
        return { matrices, ...error, residual };
    };
    const initial = evaluation();
    const baseResult = {
        initialPErr: initial.pErr, initialRErr: initial.rErr
    };
    const N = viewer.dhParams.length;
    if (viewer.dhParams.some(joint => joint.type !== 'R')) {
        return { ...baseResult, solved: false, status: 'unsupported_joint',
            iterations: 0, pErr: initial.pErr, rErr: initial.rErr };
    }
    if (N < 6) {
        return { ...baseResult, solved: false, status: 'insufficient_dof',
            iterations: 0, pErr: initial.pErr, rErr: initial.rErr };
    }
    let current = initial;
    let best = initial;
    let bestAngles = viewer.dhParams.map(joint => joint.theta);
    let status = 'max_iterations';
    let iterations = 0;
    let stalled = 0;

    for (let iter = 0; iter < maxIterations; iter++) {
        if (current.pErr <= posTolerance && current.rErr <= rotTolerance) {
            status = 'converged';
            break;
        }
        // The viewer computes J in base coordinates with all revolute
        // variables measured in radians.
        const J = viewer.computeJacobian(current.matrices).map((row, r) =>
            row.map(value => r < 3 ? value / characteristicLength : value));
        const desired = [
            current.position.x / characteristicLength,
            current.position.y / characteristicLength,
            current.position.z / characteristicLength,
            current.rotation.x, current.rotation.y, current.rotation.z
        ];
        const JJt = Array.from({ length: 6 }, (_, r) =>
            Array.from({ length: 6 }, (_, c) =>
                J[r].reduce((sum, value, k) => sum + value * J[c][k], 0)
                    + (r === c ? damping * damping : 0)
            )
        );
        const weights = gaussianElimination6x6(JJt, desired);
        if (!weights) {
            status = 'numerical_failure';
            break;
        }
        const updates = Array.from({ length: N }, (_, j) =>
            learningRate * J.reduce((sum, row, r) => sum + row[j] * weights[r], 0));
        const largestStep = Math.max(...updates.map(Math.abs));
        if (!Number.isFinite(largestStep)) {
            status = 'numerical_failure';
            break;
        }
        const stepScale = largestStep > maxStep ? maxStep / largestStep : 1;
        const previous = viewer.dhParams.map(joint => joint.theta);
        let accepted = false;
        // Monotone line search: refuse numerical steps that worsen the
        // combined error. A difficult target may still stall.
        for (const fraction of [1, 0.5, 0.25, 0.125, 0.0625]) {
            viewer.dhParams.forEach((joint, j) => {
                const theta = previous[j] + THREE.MathUtils.radToDeg(
                    updates[j] * stepScale * fraction);
                joint.theta = ((theta + 180) % 360 + 360) % 360 - 180;
            });
            const candidate = evaluation();
            if (candidate.residual < current.residual - 1e-11) {
                current = candidate;
                accepted = true;
                break;
            }
        }
        iterations = iter + 1;
        if (!accepted) {
            viewer.dhParams.forEach((joint, j) => { joint.theta = previous[j]; });
            stalled++;
            if (stalled >= 6) { status = 'stalled'; break; }
        } else {
            stalled = 0;
            if (current.residual < best.residual) {
                best = current;
                bestAngles = viewer.dhParams.map(joint => joint.theta);
            }
        }
    }
    viewer.dhParams.forEach((joint, j) => { joint.theta = bestAngles[j]; });
    const final = evaluation();
    const solved = final.pErr <= posTolerance && final.rErr <= rotTolerance;
    if (solved) status = 'converged';
    return { ...baseResult, solved, status, iterations,
        pErr: final.pErr, rErr: final.rErr };
}
