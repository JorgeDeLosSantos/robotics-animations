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
// IK Solver: Full 6-DoF using Damped Least Squares (DLS)
// ---------------------------------------------------------------------------
/**
 * Solves full 6-DoF inverse kinematics (position + orientation) using DLS.
 * @param {RobotViewer} viewer - The robot viewer instance
 * @param {THREE.Vector3} targetPos - Target position in world coordinates
 * @param {THREE.Quaternion} targetQuat - Target orientation as quaternion
 * @param {Object} [options] - Optional parameters
 * @param {number} [options.maxIterations=300] - Maximum DLS iterations
 * @param {number} [options.learningRate=0.25] - Step size factor (alpha)
 * @param {number} [options.damping=2.0] - Damping factor (lambda)
 * @param {number} [options.posTolerance=0.1] - Position convergence tolerance (mm)
 * @param {number} [options.rotTolerance=0.001] - Orientation convergence tolerance (rad)
 * @returns {Object} Result object with solved, pErr, rErr
 */
export function solveIKFull6DoF(viewer, targetPos, targetQuat, options = {}) {
    const maxIterations = options.maxIterations || 300;
    const learningRate = options.learningRate || 0.25;
    const damping = options.damping || 2.0;
    const posTolerance = options.posTolerance || 0.1;
    const rotTolerance = options.rotTolerance || 0.001;

    const T_target = new THREE.Matrix4().makeRotationFromQuaternion(targetQuat).setPosition(targetPos);

    let pErr = 1000, rErr = 1000;
    let solved = false;

    for (let iter = 0; iter < maxIterations; iter++) {
        const { matrices, eeMatrix } = viewer.computeForwardKinematics();

        // Position error
        const currentPos = new THREE.Vector3().setFromMatrixPosition(eeMatrix);
        const posError = new THREE.Vector3().subVectors(targetPos, currentPos);

        // Orientation error using rotation vector cross product
        const R_curr = new THREE.Matrix3().setFromMatrix4(eeMatrix);
        const R_targ = new THREE.Matrix3().setFromMatrix4(T_target);

        const nx = new THREE.Vector3(R_curr.elements[0], R_curr.elements[1], R_curr.elements[2]);
        const ny = new THREE.Vector3(R_curr.elements[3], R_curr.elements[4], R_curr.elements[5]);
        const nz = new THREE.Vector3(R_curr.elements[6], R_curr.elements[7], R_curr.elements[8]);
        const dx = new THREE.Vector3(R_targ.elements[0], R_targ.elements[1], R_targ.elements[2]);
        const dy = new THREE.Vector3(R_targ.elements[3], R_targ.elements[4], R_targ.elements[5]);
        const dz = new THREE.Vector3(R_targ.elements[6], R_targ.elements[7], R_targ.elements[8]);

        const rotError = new THREE.Vector3()
            .crossVectors(nx, dx)
            .add(new THREE.Vector3().crossVectors(ny, dy))
            .add(new THREE.Vector3().crossVectors(nz, dz))
            .multiplyScalar(0.5);

        pErr = posError.length();
        rErr = rotError.length();

        if (pErr < posTolerance && rErr < rotTolerance) {
            solved = true;
            break;
        }

        const dX = [posError.x, posError.y, posError.z, rotError.x, rotError.y, rotError.z];

        // Build Jacobian
        const N = viewer.dhParams.length;
        const J = Array.from({ length: 6 }, () => new Array(N).fill(0));

        for (let j = 0; j < N; j++) {
            const p_j = j === 0 ? new THREE.Vector3(0, 0, 0) : new THREE.Vector3().setFromMatrixPosition(matrices[j - 1]);
            const m = j === 0 ? (new THREE.Matrix4()).elements : matrices[j - 1].elements;
            const z_j = new THREE.Vector3(m[8], m[9], m[10]).normalize();

            const p_ee_minus_pj = new THREE.Vector3().subVectors(currentPos, p_j);
            const linearVel = new THREE.Vector3().crossVectors(z_j, p_ee_minus_pj);

            J[0][j] = linearVel.x; J[1][j] = linearVel.y; J[2][j] = linearVel.z;
            J[3][j] = z_j.x; J[4][j] = z_j.y; J[5][j] = z_j.z;
        }

        // DLS: J * J^T with damping
        const JJt = Array.from({ length: 6 }, () => new Array(6).fill(0));
        for (let r = 0; r < 6; r++) {
            for (let c = 0; c < 6; c++) {
                let sum = 0;
                for (let k = 0; k < N; k++) sum += J[r][k] * J[c][k];
                JJt[r][c] = sum + (r === c ? damping * damping : 0);
            }
        }

        const w = gaussianElimination6x6(JJt, dX);
        if (!w) break;

        for (let j = 0; j < N; j++) {
            let delta = 0;
            for (let r = 0; r < 6; r++) delta += J[r][j] * w[r];

            const step = delta * learningRate;
            let newTheta = viewer.dhParams[j].theta + THREE.MathUtils.radToDeg(step);
            newTheta = ((newTheta + 180) % 360 + 360) % 360 - 180;
            viewer.dhParams[j].theta = parseFloat(newTheta.toFixed(2));
        }
    }

    return { solved, pErr, rErr };
}