/**
 * OrthographicCamera applies zoom around the frustum's midpoint, so moving
 * the midpoint must be divided by zoom to obtain a requested NDC offset.
 *
 * The returned frustum keeps the same unzoomed width and height. Its projected
 * center maps to (ndcX, ndcY), regardless of camera zoom.
 */
export function orthographicFrustumForCenter(width, height, zoom, ndcX, ndcY) {
  if (![width, height, zoom, ndcX, ndcY].every(Number.isFinite)
      || width <= 0 || height <= 0 || zoom <= 0) {
    throw new RangeError('Expected finite dimensions and positive zoom.');
  }
  const shiftX = width * ndcX / (2 * zoom);
  const shiftY = height * ndcY / (2 * zoom);
  return {
    left: -width / 2 - shiftX,
    right: width / 2 - shiftX,
    top: height / 2 - shiftY,
    bottom: -height / 2 - shiftY,
  };
}
