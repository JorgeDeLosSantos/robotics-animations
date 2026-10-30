import * as THREE from 'three';

/**
 * Fit an orthographic view around selected world-space objects.
 * The floor grid is intentionally excluded. Existing orbit direction is preserved.
 * Account for sidebar and toolbar overlays on full-viewport transformation labs.
 * Returns false when there is nothing to fit.
 */
export function fitOrthographicObjects({ camera, controls, canvas, objects, points = [], sidebar, toolbar, padding = 1.3 }) {
  const bounds = new THREE.Box3();
  bounds.makeEmpty();
  for (const object of objects) {
    if (!object || object.visible === false) continue;
    object.updateWorldMatrix(true, true);
    bounds.union(new THREE.Box3().setFromObject(object));
  }
  // Explicit frame origins are useful for translated bodies, but reference-axis
  // helpers should not bias the camera toward their positive endpoints.
  for (const point of points) bounds.expandByPoint(point);
  if (bounds.isEmpty()) return false;

  const center = bounds.getCenter(new THREE.Vector3());
  const direction = camera.position.clone().sub(controls.target);
  const distance = Math.max(direction.length(), 1);
  if (direction.lengthSq() < 1e-10) direction.set(1, -1, 1);
  direction.normalize();
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(direction, distance);
  camera.lookAt(center);

  // Remove any offset from a previous fit before measuring camera-space bounds.
  const width = camera.right - camera.left;
  const height = camera.top - camera.bottom;
  camera.left = -width / 2;
  camera.right = width / 2;
  camera.top = height / 2;
  camera.bottom = -height / 2;
  camera.zoom = 1;
  camera.updateProjectionMatrix();
  camera.updateMatrixWorld(true);

  const rect = canvas.getBoundingClientRect();
  const mobile = window.matchMedia('(max-width: 720px)').matches;
  const sidebarRect = sidebar?.getBoundingClientRect();
  const toolbarRect = toolbar?.getBoundingClientRect();
  const margin = 12;

  let left = margin, top = margin, right = rect.width - margin, bottom = rect.height - margin;
  if (mobile) {
    if (sidebarRect) top = Math.max(top, sidebarRect.bottom - rect.top + margin);
    if (toolbarRect) bottom = Math.min(bottom, toolbarRect.top - rect.top - margin);
  } else {
    if (sidebarRect) left = Math.max(left, sidebarRect.right - rect.left + margin);
    if (toolbarRect) top = Math.max(top, toolbarRect.bottom - rect.top + margin);
  }
  // When the page is extremely small, retain a safe centered fallback.
  if (right - left < 60) { left = margin; right = rect.width - margin; }
  if (bottom - top < 60) { top = margin; bottom = rect.height - margin; }

  const min = bounds.min, max = bounds.max;
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (const x of [min.x, max.x]) {
    for (const y of [min.y, max.y]) {
      for (const z of [min.z, max.z]) {
        const p = new THREE.Vector3(x, y, z).applyMatrix4(camera.matrixWorldInverse);
        xmin = Math.min(xmin, p.x); xmax = Math.max(xmax, p.x);
        ymin = Math.min(ymin, p.y); ymax = Math.max(ymax, p.y);
      }
    }
  }

  const availableWidth = Math.max(right - left, 1);
  const availableHeight = Math.max(bottom - top, 1);
  const zoomX = width * availableWidth / (Math.max(xmax - xmin, 1e-6) * Math.max(rect.width, 1) * padding);
  const zoomY = height * availableHeight / (Math.max(ymax - ymin, 1e-6) * Math.max(rect.height, 1) * padding);
  camera.zoom = THREE.MathUtils.clamp(Math.min(zoomX, zoomY), 0.05, 20);

  // Move the projection center into the visible area without changing orbit target.
  const ndcX = (left + right) / Math.max(rect.width, 1) - 1;
  const ndcY = 1 - (top + bottom) / Math.max(rect.height, 1);
  camera.left = -width * (1 + ndcX) / 2;
  camera.right = width * (1 - ndcX) / 2;
  camera.bottom = -height * (1 + ndcY) / 2;
  camera.top = height * (1 - ndcY) / 2;
  camera.updateProjectionMatrix();
  controls.update();
  return true;
}
