import test from 'node:test';
import assert from 'node:assert/strict';
import { orthographicFrustumForCenter } from '../shared/orthographic-frustum.js';

// Three.js OrthographicCamera.updateProjectionMatrix() scales the frustum
// about (left + right)/2 and (top + bottom)/2 by the camera.zoom factor.
function projectedCenter({ left, right, top, bottom }, zoom) {
  return {
    x: -(left + right) * zoom / (right - left),
    y: -(top + bottom) * zoom / (top - bottom),
  };
}

for (const { zoom, ndcX, ndcY } of [
  { zoom: 1, ndcX: 0.25, ndcY: -0.12 },
  { zoom: 6, ndcX: 0.2638888889, ndcY: -0.1177777778 },
  { zoom: 7, ndcX: 0, ndcY: -0.3981042654 },
  { zoom: 0.5, ndcX: -0.18, ndcY: 0.22 },
  { zoom: 20, ndcX: 0.37, ndcY: -0.29 },
]) {
  test(`fit center stays at requested NDC position with zoom ${zoom}`, () => {
    const frustum = orthographicFrustumForCenter(28, 16, zoom, ndcX, ndcY);
    const center = projectedCenter(frustum, zoom);
    assert.ok(Math.abs(center.x - ndcX) < 1e-12);
    assert.ok(Math.abs(center.y - ndcY) < 1e-12);
    assert.ok(Math.abs((frustum.right - frustum.left) - 28) < 1e-12);
    assert.ok(Math.abs((frustum.top - frustum.bottom) - 16) < 1e-12);
  });
}

test('rejects invalid zoom and dimensions', () => {
  assert.throws(() => orthographicFrustumForCenter(28, 16, 0, 0, 0), RangeError);
  assert.throws(() => orthographicFrustumForCenter(0, 16, 2, 0, 0), RangeError);
  assert.throws(() => orthographicFrustumForCenter(28, 16, NaN, 0, 0), RangeError);
});
