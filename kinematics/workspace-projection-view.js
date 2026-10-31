import { WORKSPACE_PLANES, projectionLayout } from './workspace-projections.js';
import { nearestSectionSamples } from './workspace-sections.js';
import { nextSelectedSample } from './workspace-selection.js';

/**
 * Single Canvas2D section of the existing sample cloud, not a second
 * sampling method. A lightweight HTML marker tracks the selected sample.
 */
export class WorkspaceSectionView {
  constructor({ onPick, getSelectedIndex }) {
    this.onPick = onPick;
    this.getSelectedIndex = getSelectedIndex;
    this.canvas = document.getElementById('section-canvas');
    this.marker = document.getElementById('section-marker');
    this.samples = null;
    this.metrics = null;
    this.slice = null;
    this.layout = null;
    this.lastPick = null;
    this.pointerStart = null;
    this.resizeQueued = false;
    this.destroyed = false;

    this.canvas.addEventListener('pointerdown', e => {
      if (e.button !== 0 || !e.isPrimary) return;
      this.pointerStart = { id: e.pointerId, x: e.clientX, y: e.clientY };
    });
    this.canvas.addEventListener('pointerup', e => this.handlePointerUp(e));
    this.canvas.addEventListener('pointercancel', () => {
      this.pointerStart = null;
      this.lastPick = null;
    });

    this.onResize = () => {
      if (this.resizeQueued || this.destroyed) return;
      this.resizeQueued = true;
      requestAnimationFrame(() => {
        this.resizeQueued = false;
        if (!this.destroyed) this.redraw();
      });
    };
    if (typeof ResizeObserver !== 'undefined') {
      this.observer = new ResizeObserver(this.onResize);
      this.observer.observe(this.canvas);
    } else {
      window.addEventListener('resize', this.onResize);
    }
  }

  show(samples, metrics, slice) {
    this.samples = samples;
    this.metrics = metrics;
    this.slice = slice;
    this.lastPick = null;
    document.getElementById('section-empty').hidden = true;
    document.getElementById('section-plot').hidden = false;
    this.redraw();
  }

  clear() {
    this.samples = null;
    this.metrics = null;
    this.slice = null;
    this.layout = null;
    this.lastPick = null;
    this.marker.hidden = true;
    document.getElementById('section-plot').hidden = true;
    document.getElementById('section-empty').hidden = false;
  }

  redraw() {
    if (!this.samples || !this.metrics || !this.slice) return;
    const rect = this.canvas.getBoundingClientRect();
    const width = rect.width;
    const height = rect.height;
    if (width < 48 || height < 48) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const pw = Math.round(width * ratio);
    const ph = Math.round(height * ratio);
    if (this.canvas.width !== pw) this.canvas.width = pw;
    if (this.canvas.height !== ph) this.canvas.height = ph;
    const ctx = this.canvas.getContext('2d');
    if (!ctx) return;

    const light = window.RoboticsTheme?.getTheme() === 'light';
    ctx.setTransform(pw / width, 0, 0, ph / height, 0, 0);
    ctx.fillStyle = light ? '#f8fafc' : '#0f172a';
    ctx.fillRect(0, 0, width, height);
    const axisPlaneId = { x: 'yz', y: 'xz', z: 'xy' }[this.slice.axisId];
    const plane = WORKSPACE_PLANES.find(item => item.id === axisPlaneId);
    this.layout = projectionLayout(this.metrics, plane, width, height, 16);
    const { midU, midV, scale } = this.layout;

    const axisX = width / 2 - midU * scale;
    const axisY = height / 2 + midV * scale;
    ctx.strokeStyle = light ? '#94a3b8' : '#475569';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    if (axisX >= 0 && axisX <= width) {
      ctx.moveTo(axisX, 0); ctx.lineTo(axisX, height);
    }
    if (axisY >= 0 && axisY <= height) {
      ctx.moveTo(0, axisY); ctx.lineTo(width, axisY);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = light ? '#2563eb' : '#38bdf8';
    ctx.globalAlpha = 0.75;
    const xyz = this.samples.positions;
    for (const index of this.slice.indices) {
      const x = width / 2 + (xyz[3 * index + plane.hIndex] - midU) * scale;
      const y = height / 2 - (xyz[3 * index + plane.vIndex] - midV) * scale;
      ctx.fillRect(x - 1, y - 1, 2, 2);
    }
    ctx.globalAlpha = 1;
    this.highlight(this.getSelectedIndex());
  }

  highlight(index) {
    if (!this.layout || !this.slice || !Number.isSafeInteger(index) ||
        !this.slice.indices.includes(index)) {
      this.marker.hidden = true;
      return;
    }
    const { width, height, scale, midU, midV, plane } = this.layout;
    const xyz = this.samples.positions;
    const x = width / 2 + (xyz[3 * index + plane.hIndex] - midU) * scale;
    const y = height / 2 - (xyz[3 * index + plane.vIndex] - midV) * scale;
    this.marker.style.left = 100 * x / width + '%';
    this.marker.style.top = 100 * y / height + '%';
    this.marker.hidden = false;
  }

  handlePointerUp(event) {
    const start = this.pointerStart;
    this.pointerStart = null;
    if (!start || start.id !== event.pointerId || !this.samples ||
        !this.slice || !this.layout) return;
    if (Math.hypot(start.x - event.clientX, start.y - event.clientY) > 6) {
      this.lastPick = null;
      return;
    }
    const rect = this.canvas.getBoundingClientRect();
    const pointer = { x: event.clientX - rect.left, y: event.clientY - rect.top };
    const candidates = nearestSectionSamples(this.samples, this.slice, this.layout, pointer);
    if (!candidates.length) return;
    const sameSpot = this.lastPick &&
      Math.hypot(this.lastPick.x - pointer.x, this.lastPick.y - pointer.y) <= 6;
    const index = nextSelectedSample(
      candidates, this.getSelectedIndex(), Boolean(sameSpot)
    );
    this.onPick(index, candidates.length);
    this.lastPick = pointer;
  }

  destroy() {
    this.destroyed = true;
    if (this.observer) this.observer.disconnect();
    else window.removeEventListener('resize', this.onResize);
  }
}
