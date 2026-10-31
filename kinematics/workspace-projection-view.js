import {
  WORKSPACE_PLANES, projectionLayout, nearestProjectionSamples
} from './workspace-projections.js';
import { nextSelectedSample } from './workspace-selection.js';

/**
 * Three responsive Canvas2D plots of the SAME 3D sampled position buffer.
 * 2D views are independent of the Three.js camera and 3D cloud visibility.
 * Selection markers are HTML overlays, so selecting a point never repaints
 * thousands of sampled points.
 */
export class WorkspaceProjectionViews {
  constructor({ onPick, getSelectedIndex }) {
    this.onPick = onPick;
    this.getSelectedIndex = getSelectedIndex;
    this.samples = null;
    this.metrics = null;
    this.lastPick = null;
    this.pendingResize = false;
    this.charts = WORKSPACE_PLANES.map(plane => {
      const canvas = document.getElementById('projection-' + plane.id);
      const marker = document.getElementById('projection-marker-' + plane.id);
      if (!canvas || !marker) throw new Error('Missing Cartesian projection: ' + plane.id);
      const chart = { plane, canvas, marker, layout: null, pointerStart: null };
      canvas.addEventListener('pointerdown', event => {
        if (event.button !== 0 || !event.isPrimary) return;
        chart.pointerStart = {
          id: event.pointerId, x: event.clientX, y: event.clientY
        };
      });
      canvas.addEventListener('pointerup', event => this.handlePointerUp(chart, event));
      canvas.addEventListener('pointercancel', () => {
        chart.pointerStart = null;
        this.lastPick = null;
      });
      return chart;
    });

    this.onResize = () => {
      if (this.pendingResize) return;
      this.pendingResize = true;
      requestAnimationFrame(() => {
        this.pendingResize = false;
        this.redraw();
      });
    };
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(this.onResize);
      for (const chart of this.charts) this.resizeObserver.observe(chart.canvas);
    } else {
      window.addEventListener('resize', this.onResize);
    }
  }

  show(samples, metrics) {
    this.samples = samples;
    this.metrics = metrics;
    this.lastPick = null;
    document.getElementById('projections-empty').hidden = true;
    document.getElementById('projections-charts').hidden = false;
    this.redraw();
  }

  clear() {
    this.samples = null;
    this.metrics = null;
    this.lastPick = null;
    for (const chart of this.charts) {
      chart.layout = null;
      chart.marker.hidden = true;
    }
    document.getElementById('projections-charts').hidden = true;
    document.getElementById('projections-empty').hidden = false;
  }

  redraw() {
    if (!this.samples || !this.metrics) return;
    const light = window.RoboticsTheme?.getTheme() === 'light';
    for (const chart of this.charts) {
      const { canvas, plane } = chart;
      const rect = canvas.getBoundingClientRect();
      const width = rect.width;
      const height = rect.height;
      if (width < 48 || height < 48) continue;

      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const pixelWidth = Math.max(1, Math.round(width * dpr));
      const pixelHeight = Math.max(1, Math.round(height * dpr));
      if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
      if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
      const ctx = canvas.getContext('2d');
      if (!ctx) continue;
      ctx.setTransform(pixelWidth / width, 0, 0, pixelHeight / height, 0, 0);
      ctx.fillStyle = light ? '#f8fafc' : '#0f172a';
      ctx.fillRect(0, 0, width, height);

      const layout = projectionLayout(this.metrics, plane, width, height, 16);
      chart.layout = layout;
      const { midU, midV, scale } = layout;

      // Show coordinate-zero axes only if the corresponding zero is in view.
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
      ctx.globalAlpha = 0.46;
      const xyz = this.samples.positions;
      const u = plane.hIndex;
      const v = plane.vIndex;
      for (let k = 0; k < this.samples.sampleCount; k++) {
        const px = width / 2 + (xyz[3 * k + u] - midU) * scale;
        const py = height / 2 - (xyz[3 * k + v] - midV) * scale;
        ctx.fillRect(px - 0.8, py - 0.8, 1.6, 1.6);
      }
      ctx.globalAlpha = 1;
    }
    this.highlight(this.getSelectedIndex());
  }

  highlight(index) {
    for (const chart of this.charts) {
      const { marker, layout } = chart;
      if (!layout || !this.samples || index === null ||
          !Number.isSafeInteger(index) || index < 0 || index >= this.samples.sampleCount) {
        marker.hidden = true;
        continue;
      }
      const { plane, width, height, scale, midU, midV } = layout;
      const coords = this.samples.positions;
      const px = width / 2 + (coords[3 * index + plane.hIndex] - midU) * scale;
      const py = height / 2 - (coords[3 * index + plane.vIndex] - midV) * scale;
      marker.style.left = (100 * px / width) + '%';
      marker.style.top = (100 * py / height) + '%';
      marker.hidden = false;
    }
  }

  handlePointerUp(chart, event) {
    const start = chart.pointerStart;
    chart.pointerStart = null;
    if (!start || start.id !== event.pointerId || !this.samples || !chart.layout) return;
    if (Math.hypot(start.x - event.clientX, start.y - event.clientY) > 6) {
      this.lastPick = null;
      return;
    }
    const rect = chart.canvas.getBoundingClientRect();
    const pointer = { x: event.clientX - rect.left, y: event.clientY - rect.top };
    const candidates = nearestProjectionSamples(this.samples, chart.layout, pointer);
    if (!candidates.length) return;

    const sameSpot = this.lastPick &&
      this.lastPick.plane === chart.plane.id &&
      Math.hypot(this.lastPick.x - pointer.x, this.lastPick.y - pointer.y) <= 6;
    const chosen = nextSelectedSample(
      candidates, this.getSelectedIndex(), Boolean(sameSpot)
    );
    this.onPick(chosen, candidates.length, chart.plane.id);
    this.lastPick = { plane: chart.plane.id, x: pointer.x, y: pointer.y };
  }

  destroy() {
    if (this.resizeObserver) this.resizeObserver.disconnect();
    else window.removeEventListener('resize', this.onResize);
  }
}
