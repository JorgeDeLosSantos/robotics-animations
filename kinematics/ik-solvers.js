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
 * Solves inverse kinematics for position only using the CCD algorithm.
 * @param {RobotViewer} viewer - The robot viewer instance
 * @param {THREE.Vector3} target - Target position in world coordinates
 * @param {Object} [options] - Optional parameters
 * @param {number} [options.maxIterations=100] - Maximum CCD iterations
 * @param {number} [options.tolerance=0.1] - Convergence tolerance in mm
 * @returns {number} Final distance error to target
 */
export function solveIKPositionCCD(viewer, target, options = {}) {
    const maxIterations = options.maxIterations || 100;
    const tolerance = options.tolerance || 0.1;
    let solved = false;

    for (let iter = 0; iter < maxIterations; iter++) {
        for (let i = viewer.dhParams.length - 1; i >= 0; i--) {
            const eePos = viewer.getEEPosition();
            const jointPos = viewer.getJointPosition(i);

            if (eePos.distanceTo(target) < tolerance) {
                solved = true;
                break;
            }

            const jointToEE = new THREE.Vector3().subVectors(eePos, jointPos);
            const jointToTarget = new THREE.Vector3().subVectors(target, jointPos);

            const { zAxes } = viewer.getJointStates(i);
            const jointAxis = zAxes[i] || new THREE.Vector3(0, 0, 1);

            jointToEE.projectOnPlane(jointAxis).normalize();
            jointToTarget.projectOnPlane(jointAxis).normalize();

            let dot = jointToEE.dot(jointToTarget);
            dot = Math.max(-1, Math.min(1, dot));
            let deltaTheta = Math.acos(dot);

            const cross = new THREE.Vector3().crossVectors(jointToEE, jointToTarget);
            if (cross.dot(jointAxis) < 0) {
                deltaTheta = -deltaTheta;
            }

            const deltaDeg = THREE.MathUtils.radToDeg(deltaTheta);
            if (!isNaN(deltaDeg)) {
                let newTheta = viewer.dhParams[i].theta + deltaDeg;
                if (newTheta > 360) newTheta -= 360;
                if (newTheta < -360) newTheta += 360;
                viewer.dhParams[i].theta = parseFloat(newTheta.toFixed(2));
            }
        }
        if (solved) break;
    }

    const finalEE = viewer.getEEPosition();
    return finalEE.distanceTo(target);
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