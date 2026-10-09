/**
 * shared-robot-viewer.js
 * Shared module for 3D robot visualization using Denavit-Hartenberg parameters.
 * Provides common Three.js setup, DH table rendering, view controls, and matrix display.
 * Uses modern Three.js ES module imports.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------------------------------------------------------------------------
// Utility: solve a 6x6 linear system via Gaussian elimination with pivoting
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
// RobotViewer class – shared 3D scene, DH table, and visualization
// ---------------------------------------------------------------------------
export class RobotViewer {
    /**
     * @param {Object} options
     * @param {string} options.canvasContainerId - ID of the container element for the 3D viewport
     * @param {string} options.tableBodyId - ID of the tbody element for the DH parameter table
     * @param {string} options.addRowBtnId - ID of the "Add link" button
     * @param {string} options.matrixContainerId - ID of the element to display the homogeneous matrix
     * @param {Array}  options.initialDHParams - Initial DH parameters array
     * @param {number} [options.frustumSize=1200] - Orthographic camera frustum size
     * @param {number} [options.maxLinks=8] - Maximum number of links allowed
     */
    constructor(options) {
        const {
            canvasContainerId = 'canvas-container',
            tableBodyId = 'dh-table-body',
            addRowBtnId = 'add-row-btn',
            matrixContainerId = 'matrix-container',
            initialDHParams = [],
            frustumSize = 1200,
            maxLinks = 8
        } = options;

        this.canvasContainer = document.getElementById(canvasContainerId);
        this.tableBody = document.getElementById(tableBodyId);
        this.addRowBtn = document.getElementById(addRowBtnId);
        this.matrixContainer = document.getElementById(matrixContainerId);
        this.frustumSize = frustumSize;
        this.maxLinks = maxLinks;

        // Three.js objects (initialized in init3D)
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.robotGroup = null;

        // DH parameters
        this.dhParams = initialDHParams.length > 0
            ? JSON.parse(JSON.stringify(initialDHParams))
            : [];

        // Bind event handlers
        this._onInputChange = this._onInputChange.bind(this);
        this._onDeleteRow = this._onDeleteRow.bind(this);
        this._onAddRow = this._onAddRow.bind(this);
        this._onWindowResize = this._onWindowResize.bind(this);
        this._animate = this._animate.bind(this);
        this._tableControlsBound = false;
    }

    // ======================================================================
    // 3D Scene Initialization
    // ======================================================================
    init3D() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f172a);

        const aspect = this.canvasContainer.clientWidth / this.canvasContainer.clientHeight;

        this.camera = new THREE.OrthographicCamera(
            this.frustumSize * aspect / -2,
            this.frustumSize * aspect / 2,
            this.frustumSize / 2,
            this.frustumSize / -2,
            0.1,
            10000
        );
        this.camera.position.set(1500, -1500, 1500);
        this.camera.up.set(0, 0, 1);  // Z-axis pointing up
        this.camera.lookAt(0, 0, 0);

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.canvasContainer.clientWidth, this.canvasContainer.clientHeight);
        this.canvasContainer.appendChild(this.renderer.domElement);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.mouseButtons = {
            LEFT: THREE.MOUSE.PAN,
            MIDDLE: THREE.MOUSE.ROTATE,
            RIGHT: THREE.MOUSE.PAN
        };

        // Reset view button highlights when user orbits freely
        this.controls.addEventListener('start', () => {
            document.querySelectorAll('.view-btn').forEach(btn => {
                btn.classList.remove('bg-indigo-600', 'hover:bg-indigo-500', 'text-white', 'shadow');
                btn.classList.add('bg-slate-800', 'hover:bg-slate-700', 'text-slate-200');
            });
        });

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(10, 20, 15);
        this.scene.add(dirLight);

        // Ground grid lying on the XY plane
        const gridHelper = new THREE.GridHelper(2000, 40, 0x475569, 0x334155);
        gridHelper.rotation.x = Math.PI / 2;
        this.scene.add(gridHelper);

        // Robot group
        this.robotGroup = new THREE.Group();
        this.scene.add(this.robotGroup);

        window.addEventListener('resize', this._onWindowResize);
        requestAnimationFrame(this._animate);
    }

    _animate() {
        requestAnimationFrame(this._animate);
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }

    _onWindowResize() {
        const aspect = this.canvasContainer.clientWidth / this.canvasContainer.clientHeight;
        this.camera.left = this.frustumSize * aspect / -2;
        this.camera.right = this.frustumSize * aspect / 2;
        this.camera.top = this.frustumSize / 2;
        this.camera.bottom = this.frustumSize / -2;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.canvasContainer.clientWidth, this.canvasContainer.clientHeight);
    }

    // ======================================================================
    // Camera View Presets
    // ======================================================================
    changeView(view) {
        const d = 1800;
        this.controls.target.set(0, 0, 0);

        switch (view) {
            case 'top':
                this.camera.position.set(0, 0, d);
                this.camera.up.set(0, 1, 0);
                break;
            case 'front':
                this.camera.position.set(0, -d, 0);
                this.camera.up.set(0, 0, 1);
                break;
            case 'side':
                this.camera.position.set(d, 0, 0);
                this.camera.up.set(0, 0, 1);
                break;
            case 'iso':
                const isoVal = d / Math.sqrt(3);
                this.camera.position.set(isoVal, -isoVal, isoVal);
                this.camera.up.set(0, 0, 1);
                break;
        }
        this.camera.lookAt(0, 0, 0);
        this.controls.update();

        document.querySelectorAll('.view-btn').forEach(btn => {
            btn.classList.remove('bg-indigo-600', 'hover:bg-indigo-500', 'text-white', 'shadow');
            btn.classList.add('bg-slate-800', 'hover:bg-slate-700', 'text-slate-200');
        });

        const activeBtn = document.getElementById(`btn-view-${view}`);
        if (activeBtn) {
            activeBtn.classList.remove('bg-slate-800', 'hover:bg-slate-700', 'text-slate-200');
            activeBtn.classList.add('bg-indigo-600', 'hover:bg-indigo-500', 'text-white', 'shadow');
        }
    }

    // ======================================================================
    // DH Table Rendering
    // ======================================================================
    renderTable() {
        this.tableBody.innerHTML = '';
        this.dhParams.forEach((param, index) => {
            const tr = document.createElement('tr');
            tr.className = 'border-b border-slate-800 hover:bg-slate-800/40 transition-colors text-sm';
            tr.innerHTML = this._buildTableRow(param, index);
            this.tableBody.appendChild(tr);
        });

        this.tableBody.querySelectorAll('input, select').forEach(input =>
            input.addEventListener('input', this._onInputChange)
        );
        this.tableBody.querySelectorAll('.delete-btn').forEach(btn =>
            btn.addEventListener('click', this._onDeleteRow)
        );

        // Static controls must be bound only once. renderTable() is called
        // repeatedly when adding/removing links or changing joint types.
        if (!this._tableControlsBound) {
            this._attachViewButtonListeners();
            this.addRowBtn?.addEventListener('click', this._onAddRow);
            this._tableControlsBound = true;
        }
    }

    /**
     * Builds the HTML for a single DH table row.
     * Override this method to customize the table columns.
     * @param {Object} param - DH parameter { type, a, alpha, d, theta }
     * @param {number} index - Row index
     * @returns {string} HTML string
     */
    _buildTableRow(param, index) {
        const type = param.type || 'R';
        return `
            <td class="p-2 text-center text-slate-500 font-bold">${index + 1}</td>
            <td class="p-1">
                <select data-index="${index}" data-prop="type"
                    class="w-full bg-slate-800 border border-slate-700 rounded px-1 py-1 text-center font-bold text-indigo-400 focus:outline-none cursor-pointer">
                    <option value="R" ${type === 'R' ? 'selected' : ''}>R</option>
                    <option value="P" ${type === 'P' ? 'selected' : ''}>P</option>
                </select>
            </td>
            <td class="p-1">
                <input type="number" step="any" data-index="${index}" data-prop="a" value="${param.a}"
                    class="w-full bg-slate-800 border border-slate-700 rounded px-1.5 py-1 text-center text-amber-300 focus:outline-none">
            </td>
            <td class="p-1">
                <input type="number" min="-360" max="360" step="any" data-index="${index}" data-prop="alpha" value="${param.alpha}"
                    class="w-full bg-slate-800 border border-slate-700 rounded px-1.5 py-1 text-center text-orange-300 focus:outline-none">
            </td>
            <td class="p-1">
                <input type="number" step="any" data-index="${index}" data-prop="d" value="${param.d}"
                    class="w-full bg-slate-800 border border-slate-700 rounded px-1.5 py-1 text-center text-emerald-300 focus:outline-none ${type === 'P' ? 'ring-2 ring-emerald-500/50' : ''}">
            </td>
            <td class="p-1">
                <input type="number" min="-360" max="360" step="any" data-index="${index}" data-prop="theta" value="${param.theta}"
                    class="w-full bg-slate-800 border border-slate-700 rounded px-1.5 py-1 text-center text-indigo-300 focus:outline-none ${type === 'R' ? 'ring-2 ring-indigo-500/50' : ''}">
            </td>
            <td class="p-2 text-center">
                <button data-index="${index}" class="delete-btn text-slate-500 hover:text-red-400 p-1 transition-colors cursor-pointer">❌</button>
            </td>
        `;
    }

    _onInputChange(e) {
        const index = parseInt(e.target.dataset.index);
        const prop = e.target.dataset.prop;
        let value = e.target.value;

        if (prop !== 'type') {
            value = parseFloat(value) || 0;
            if (prop === 'theta' || prop === 'alpha') {
                if (value < -360) value = -360;
                if (value > 360) value = 360;
                e.target.value = value;
            }
            this.dhParams[index][prop] = value;
        } else {
            // If type changes, update param and reload table to highlight active inputs
            this.dhParams[index][prop] = value;
            this.renderTable();
        }

        this.updateRobotGeometry();
    }

    _onDeleteRow(e) {
        if (this.dhParams.length <= 1) return;
        const index = parseInt(e.target.closest('button').dataset.index);
        this.dhParams.splice(index, 1);
        this.renderTable();
        this.updateRobotGeometry();
    }

    _onAddRow() {
        if (this.dhParams.length >= this.maxLinks) return;
        const last = this.dhParams.length > 0
            ? this.dhParams[this.dhParams.length - 1]
            : { type: 'R', theta: 0, d: 0, a: 200, alpha: 0 };
        this.dhParams.push({ type: 'R', theta: 0, d: 0, a: last.a, alpha: 0 });
        this.renderTable();
        this.updateRobotGeometry();
    }

    // ======================================================================
    // View Button Event Listeners
    // ======================================================================
    _attachViewButtonListeners() {
        const views = ['top', 'front', 'side', 'iso'];
        views.forEach(view => {
            const btn = document.getElementById(`btn-view-${view}`);
            if (btn) {
                // Replace inline onclick with addEventListener to work with ES modules
                btn.onclick = null;
                btn.addEventListener('click', () => this.changeView(view));
            }
        });
    }

    // ======================================================================
    // Matrix Display
    // ======================================================================
    updateMatrixDisplay(matrix) {
        if (!this.matrixContainer) return;

        this.matrixContainer.innerHTML = '';
        const te = matrix.elements;
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                const val = te[j * 4 + i];
                const cell = document.createElement('div');
                cell.className = 'bg-slate-900/80 border border-slate-800/50 py-1 rounded font-bold';
                cell.innerText = Math.abs(val) < 0.001 ? '0.00' : val.toFixed(3);
                if (j === 3 && i < 3) {
                    cell.className += ' text-amber-400';
                }
                this.matrixContainer.appendChild(cell);
            }
        }
    }

    // ======================================================================
    // Forward Kinematics – Default Implementation
    // Builds the DH transformation chain and returns the final matrix.
    // ======================================================================
    computeForwardKinematics() {
        const matrices = [];
        let currentMatrix = new THREE.Matrix4();

        this.dhParams.forEach((param) => {
            const thetaRad = THREE.MathUtils.degToRad(param.theta);
            const alphaRad = THREE.MathUtils.degToRad(param.alpha);

            const rTheta = new THREE.Matrix4().makeRotationZ(thetaRad);
            const tD = new THREE.Matrix4().makeTranslation(0, 0, param.d);
            const tA = new THREE.Matrix4().makeTranslation(param.a, 0, 0);
            const rAlpha = new THREE.Matrix4().makeRotationX(alphaRad);

            const linkTransform = new THREE.Matrix4()
                .multiply(rTheta)
                .multiply(tD)
                .multiply(tA)
                .multiply(rAlpha);

            currentMatrix.multiply(linkTransform);
            matrices.push(currentMatrix.clone());
        });

        return { matrices, eeMatrix: currentMatrix };
    }

    // ======================================================================
    // Robot Geometry Rendering – Default (FK style with orthogonal links)
    // Override this method to provide custom rendering.
    // ======================================================================
    updateRobotGeometry() {
        // Clear and dispose GPU resources from the previous robot drawing.
        // Some link meshes share a material, so dispose each resource only once.
        const disposedGeometries = new Set();
        const disposedMaterials = new Set();
        for (const child of [...this.robotGroup.children]) {
            child.traverse(object => {
                if (object.geometry && !disposedGeometries.has(object.geometry)) {
                    object.geometry.dispose();
                    disposedGeometries.add(object.geometry);
                }
                const materials = object.material
                    ? (Array.isArray(object.material) ? object.material : [object.material])
                    : [];
                for (const material of materials) {
                    if (!disposedMaterials.has(material)) {
                        material.dispose();
                        disposedMaterials.add(material);
                    }
                }
            });
            this.robotGroup.remove(child);
        }

        let currentMatrix = new THREE.Matrix4();
        let pStart = new THREE.Vector3(0, 0, 0);

        // Draw base reference frame {0}
        const baseAxes = new THREE.AxesHelper(100);
        this.robotGroup.add(baseAxes);

        this.dhParams.forEach((param) => {
            // --- 1. DRAW JOINT i IN THE PREVIOUS ORIGIN (FRAME i-1) ---
            let jointMesh;
            if (param.type === 'P') {
                const jointGeom = new THREE.BoxGeometry(16, 16, 24);
                const jointMat = new THREE.MeshStandardMaterial({ color: 0x10b981, emissive: 0x064e3b, roughness: 0.2 });
                jointMesh = new THREE.Mesh(jointGeom, jointMat);
            } else {
                const jointGeom = new THREE.CylinderGeometry(12, 12, 30, 16);
                jointGeom.rotateX(Math.PI / 2);
                const jointMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x0369a1, roughness: 0.2 });
                jointMesh = new THREE.Mesh(jointGeom, jointMat);
            }

            jointMesh.position.copy(pStart);

            const currentQuaternion = new THREE.Quaternion();
            const currentScale = new THREE.Vector3();
            const currentPos = new THREE.Vector3();
            currentMatrix.decompose(currentPos, currentQuaternion, currentScale);

            jointMesh.quaternion.copy(currentQuaternion);
            this.robotGroup.add(jointMesh);

            // --- 2. CALCULATE INTERMEDIATE STEPS FOR ORTHOGONAL LINKS ---
            const thetaRad = THREE.MathUtils.degToRad(param.theta);
            const alphaRad = THREE.MathUtils.degToRad(param.alpha);

            // Step A: Rotate by theta, translate by d (along Z_{i-1})
            const rTheta = new THREE.Matrix4().makeRotationZ(thetaRad);
            const tD = new THREE.Matrix4().makeTranslation(0, 0, param.d);

            // Step B: Translate by a (along X_new), rotate by alpha
            const tA = new THREE.Matrix4().makeTranslation(param.a, 0, 0);
            const rAlpha = new THREE.Matrix4().makeRotationX(alphaRad);

            // Intermediate point (after rotating theta and translating d)
            const intermediateMatrix = currentMatrix.clone().multiply(rTheta).multiply(tD);
            const pMid = new THREE.Vector3().setFromMatrixPosition(intermediateMatrix);

            // Final matrix for this link
            const linkTransform = new THREE.Matrix4()
                .multiply(rTheta)
                .multiply(tD)
                .multiply(tA)
                .multiply(rAlpha);

            currentMatrix.multiply(linkTransform);

            const pEnd = new THREE.Vector3().setFromMatrixPosition(currentMatrix);

            // --- 3. DRAW ORTHOGONAL LINKS (d and a separately) ---
            const linkMaterial = new THREE.MeshStandardMaterial({
                color: 0xff8c00,
                roughness: 0.4,
                metalness: 0.3,
                emissive: 0xff8c00,
                emissiveIntensity: 0.2
            });

            // Cylinder for distance 'd' (from pStart to pMid)
            const distD = pStart.distanceTo(pMid);
            if (distD > 0.01) {
                const dGeom = new THREE.CylinderGeometry(5, 5, distD, 8);
                dGeom.translate(0, distD / 2, 0);
                const dMesh = new THREE.Mesh(dGeom, linkMaterial);
                dMesh.position.copy(pStart);
                dMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), pMid.clone().sub(pStart).normalize());
                this.robotGroup.add(dMesh);
            }

            // Cylinder for distance 'a' (from pMid to pEnd)
            const distA = pMid.distanceTo(pEnd);
            if (distA > 0.01) {
                const aGeom = new THREE.CylinderGeometry(5, 5, distA, 8);
                aGeom.translate(0, distA / 2, 0);
                const aMesh = new THREE.Mesh(aGeom, linkMaterial);
                aMesh.position.copy(pMid);
                aMesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), pEnd.clone().sub(pMid).normalize());
                this.robotGroup.add(aMesh);
            }

            // --- 4. DRAW LOCAL FRAME AXES (FRAME i) ---
            const axesHelper = new THREE.AxesHelper(60);
            axesHelper.matrixAutoUpdate = false;
            axesHelper.matrix.copy(currentMatrix);
            this.robotGroup.add(axesHelper);

            pStart.copy(pEnd);
        });

        // Update homogeneous transformation display
        this.updateMatrixDisplay(currentMatrix);
    }

    // ======================================================================
    // Jacobian Computation
    // Returns the 6xN geometric Jacobian matrix.
    // ======================================================================
    computeJacobian(matrices) {
        const N = this.dhParams.length;
        const eePos = new THREE.Vector3().setFromMatrixPosition(matrices[matrices.length - 1]);
        const J = Array.from({ length: 6 }, () => []);

        for (let i = 0; i < N; i++) {
            const param = this.dhParams[i];
            const T_prev = i === 0 ? new THREE.Matrix4() : matrices[i - 1]; // transformation up to frame i-1 (or identity for i=0)
            const elements = T_prev.elements;
            const z_prev = new THREE.Vector3(elements[8], elements[9], elements[10]).normalize();
            const p_prev = new THREE.Vector3().setFromMatrixPosition(T_prev);

            let Jv, Jw;
            if (param.type === 'P') {
                Jv = z_prev.clone();
                Jw = new THREE.Vector3(0, 0, 0);
            } else {
                const r = new THREE.Vector3().subVectors(eePos, p_prev);
                Jv = new THREE.Vector3().crossVectors(z_prev, r);
                Jw = z_prev.clone();
            }

            J[0].push(Jv.x); J[1].push(Jv.y); J[2].push(Jv.z);
            J[3].push(Jw.x); J[4].push(Jw.y); J[5].push(Jw.z);
        }

        return J; // 6 rows x N columns
    }

    // ======================================================================
    // Utility: get joint positions and Z-axes for a given number of joints
    // ======================================================================
    getJointStates(upToIndex) {
        const matrices = [];
        let currentMatrix = new THREE.Matrix4();
        matrices.push(currentMatrix.clone()); // frame {0}

        const count = upToIndex !== undefined ? upToIndex : this.dhParams.length;
        for (let i = 0; i < count; i++) {
            const param = this.dhParams[i];
            const rTheta = new THREE.Matrix4().makeRotationZ(THREE.MathUtils.degToRad(param.theta));
            const tD = new THREE.Matrix4().makeTranslation(0, 0, param.d);
            const tA = new THREE.Matrix4().makeTranslation(param.a, 0, 0);
            const rAlpha = new THREE.Matrix4().makeRotationX(THREE.MathUtils.degToRad(param.alpha));
            currentMatrix.multiply(rTheta).multiply(tD).multiply(tA).multiply(rAlpha);
            matrices.push(currentMatrix.clone());
        }

        const positions = matrices.map(m => new THREE.Vector3().setFromMatrixPosition(m));
        const zAxes = matrices.map(m => {
            const e = m.elements;
            return new THREE.Vector3(e[8], e[9], e[10]).normalize();
        });

        return { positions, zAxes, matrices };
    }

    // ======================================================================
    // IK Solver (CCD) – only position, no orientation
    // ======================================================================
    solveIKPosition(target) {
        const maxIterations = 100;
        const tolerance = 0.1;
        let solved = false;

        for (let iter = 0; iter < maxIterations; iter++) {
            for (let i = this.dhParams.length - 1; i >= 0; i--) {
                const eePos = this._getEEPosition();
                const jointPos = this._getJointPosition(i);

                if (eePos.distanceTo(target) < tolerance) {
                    solved = true;
                    break;
                }

                const jointToEE = new THREE.Vector3().subVectors(eePos, jointPos);
                const jointToTarget = new THREE.Vector3().subVectors(target, jointPos);

                const { zAxes } = this.getJointStates(i);
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
                    let newTheta = this.dhParams[i].theta + deltaDeg;
                    if (newTheta > 360) newTheta -= 360;
                    if (newTheta < -360) newTheta += 360;
                    this.dhParams[i].theta = parseFloat(newTheta.toFixed(2));
                }
            }
            if (solved) break;
        }

        const finalEE = this._getEEPosition();
        return finalEE.distanceTo(target);
    }

    getEEPosition() {
        const { eeMatrix } = this.computeForwardKinematics();
        return new THREE.Vector3().setFromMatrixPosition(eeMatrix);
    }

    getJointPosition(k) {
        const { positions } = this.getJointStates(k);
        return positions[k] || new THREE.Vector3(0, 0, 0);
    }

    // ======================================================================
    // IK Solver (Full 6-DoF) – Damped Least Squares (DLS)
    // ======================================================================
    solveIKFull6DoF(targetPos, targetQuat, options = {}) {
        const maxIterations = options.maxIterations || 300;
        const learningRate = options.learningRate || 0.25;
        const damping = options.damping || 2.0;
        const posTolerance = options.posTolerance || 0.1;
        const rotTolerance = options.rotTolerance || 0.001;

        const T_target = new THREE.Matrix4().makeRotationFromQuaternion(targetQuat).setPosition(targetPos);

        let pErr = 1000, rErr = 1000;
        let solved = false;

        for (let iter = 0; iter < maxIterations; iter++) {
            const { matrices, eeMatrix } = this.computeForwardKinematics();

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
            const N = this.dhParams.length;
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
                let newTheta = this.dhParams[j].theta + THREE.MathUtils.radToDeg(step);
                newTheta = ((newTheta + 180) % 360 + 360) % 360 - 180;
                this.dhParams[j].theta = parseFloat(newTheta.toFixed(2));
            }
        }

        return { solved, pErr, rErr };
    }

    // ======================================================================
    // Lifecycle
    // ======================================================================
    destroy() {
        window.removeEventListener('resize', this._onWindowResize);
        if (this.renderer) {
            this.renderer.dispose();
            this.canvasContainer.removeChild(this.renderer.domElement);
        }
    }
}