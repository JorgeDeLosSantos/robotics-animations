import { buildJointPlotSeries, JOINT_PLOT_METRICS } from './joint-trajectory-plot-data.js';

const NS = 'http://www.w3.org/2000/svg';
const WIDTH = 370, HEIGHT = 175;
const AREA = { left: 54, right: 13, top: 12, bottom: 29 };
const PLOT_WIDTH = WIDTH - AREA.left - AREA.right;
const PLOT_HEIGHT = HEIGHT - AREA.top - AREA.bottom;
const COLORS = {
  position: '#06b6d4',
  velocity: '#a78bfa',
  acceleration: '#fb923c'
};
const VALUES = { position: 'q', velocity: 'dq', acceleration: 'ddq' };

const svgElement = (name, attrs = {}) => {
  const node = document.createElementNS(NS, name);
  for (const [key, value] of Object.entries(attrs)) node.setAttribute(key, String(value));
  return node;
};
const finite = value => Number.isFinite(value);
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const numberLabel = value => finite(value) ? (Math.abs(value) < 1e-10 ? '0' :
  Number(value).toPrecision(3).replace(/\.0+(?=e|$)/, '')) : '—';

export class JointTrajectoryPlots {
  constructor({ jointSelect, roots, outputs, onSeek }) {
    this.jointSelect = jointSelect;
    this.roots = roots;
    this.outputs = outputs;
    this.onSeek = onSeek;
    this.trajectory = null;
    this.jointTypes = [];
    this.jointIndex = 0;
    this.time = 0;
    this.views = new Map();

    jointSelect.addEventListener('change', () => {
      this.jointIndex = Number(jointSelect.value);
      this.render();
    });
  }

  refresh(trajectory, jointTypes) {
    this.trajectory = trajectory;
    this.jointTypes = Array.isArray(jointTypes) ? [...jointTypes] : [];
    const count = trajectory?.initial.length ?? this.jointTypes.length;
    if (!count || this.jointIndex >= count) this.jointIndex = 0;
    this.jointSelect.replaceChildren();
    for (let i = 0; i < count; i++) {
      const option = document.createElement('option');
      option.value = String(i);
      option.textContent = 'Joint ' + (i + 1) + ' · ' + (this.jointTypes[i] ?? 'R');
      this.jointSelect.append(option);
    }
    this.jointSelect.value = String(this.jointIndex);
    this.jointSelect.disabled = !trajectory;
    this.render();
  }

  unitFor(metric) {
    const unit = this.jointTypes[this.jointIndex] === 'P' ? 'u' : '°';
    return unit + (metric === 'position' ? '' : metric === 'velocity' ? '/s' : '/s²');
  }

  render() {
    this.views.clear();
    for (const metric of JOINT_PLOT_METRICS) {
      const root = this.roots[metric];
      root.replaceChildren();
      if (!this.trajectory) {
        const note = document.createElement('p');
        note.className = 'text-xs text-slate-400 p-4';
        note.textContent = 'Enter a valid trajectory to display this plot.';
        root.append(note);
        this.outputs[metric].textContent = '—';
        continue;
      }
      const series = buildJointPlotSeries(this.trajectory, this.jointIndex)[metric];
      const { min, max } = series;
      const duration = this.trajectory.duration;
      const x = time => AREA.left + PLOT_WIDTH * time / duration;
      const y = value => AREA.top + (max - value) * PLOT_HEIGHT / (max - min);
      const svg = svgElement('svg', {
        viewBox: '0 0 ' + WIDTH + ' ' + HEIGHT, width: '100%',
        role: 'slider', tabindex: '0', 'aria-label': metric +
          ' graph. Drag or use arrow keys to seek trajectory time.',
        'aria-valuemin': 0, 'aria-valuemax': duration,
        'aria-valuenow': clamp(this.time, 0, duration).toFixed(3)
      });
      svg.style.cssText = 'display:block;touch-action:none;user-select:none;cursor:crosshair';
      const grid = svgElement('g');
      for (let i = 0; i <= 4; i++) {
        const gy = AREA.top + PLOT_HEIGHT * i / 4;
        const gx = AREA.left + PLOT_WIDTH * i / 4;
        grid.append(svgElement('line', {
          x1: AREA.left, x2: WIDTH - AREA.right, y1: gy, y2: gy,
          stroke: 'currentColor', 'stroke-opacity': 0.13
        }));
        grid.append(svgElement('line', {
          x1: gx, x2: gx, y1: AREA.top, y2: AREA.top + PLOT_HEIGHT,
          stroke: 'currentColor', 'stroke-opacity': 0.09
        }));
      }
      svg.append(grid);
      const yLabel = (val, py) => {
        const text = svgElement('text', {
          x: AREA.left - 7, y: py + 3, 'text-anchor': 'end', 'font-size': 10,
          fill: 'currentColor', opacity: 0.75
        });
        text.textContent = numberLabel(val);
        svg.append(text);
      };
      yLabel(max, AREA.top + 3);
      yLabel((max + min) / 2, AREA.top + PLOT_HEIGHT / 2);
      yLabel(min, AREA.top + PLOT_HEIGHT);
      for (const [time, anchor] of [[0, 'start'], [duration / 2, 'middle'], [duration, 'end']]) {
        const tick = svgElement('text', {
          x: x(time), y: HEIGHT - 7, 'text-anchor': anchor, 'font-size': 10,
          fill: 'currentColor', opacity: 0.75
        });
        tick.textContent = Number(time.toPrecision(3)) + ' s';
        svg.append(tick);
      }
      for (const segment of series.segments) {
        if (!segment.length) continue;
        const points = segment.map((point, i) =>
          (i ? 'L' : 'M') + x(point.time).toFixed(3) + ',' + y(point.value).toFixed(3));
        svg.append(svgElement('path', {
          d: points.join(' '), fill: 'none', stroke: COLORS[metric],
          'stroke-width': 2.5, 'stroke-linejoin': 'round',
          'stroke-linecap': 'round'
        }));
      }
      const cursor = svgElement('line', {
        y1: AREA.top, y2: AREA.top + PLOT_HEIGHT,
        stroke: 'currentColor', 'stroke-width': 1.2,
        'stroke-dasharray': '4 3', 'stroke-opacity': 0.8
      });
      const dot = svgElement('circle', {
        r: 4.2, fill: COLORS[metric], stroke: 'currentColor', 'stroke-width': 1
      });
      svg.append(cursor, dot);
      // An invisible area makes the plot easy to scrub with mouse or touch.
      const hit = svgElement('rect', {
        x: AREA.left, y: AREA.top, width: PLOT_WIDTH, height: PLOT_HEIGHT,
        fill: 'transparent'
      });
      svg.append(hit);
      const seek = event => {
        const rect = svg.getBoundingClientRect();
        const coordinate = (event.clientX - rect.left) * WIDTH / rect.width;
        this.onSeek(clamp((coordinate - AREA.left) / PLOT_WIDTH, 0, 1) * duration);
      };
      hit.addEventListener('pointerdown', event => {
        event.preventDefault();
        hit.setPointerCapture(event.pointerId);
        seek(event);
      });
      hit.addEventListener('pointermove', event => {
        if (hit.hasPointerCapture(event.pointerId)) seek(event);
      });
      svg.addEventListener('keydown', event => {
        const keys = ['ArrowLeft', 'ArrowRight', 'Home', 'End'];
        if (!keys.includes(event.key)) return;
        event.preventDefault();
        const change = duration / 100;
        const next = event.key === 'Home' ? 0 : event.key === 'End' ? duration
          : this.time + (event.key === 'ArrowLeft' ? -change : change);
        this.onSeek(clamp(next, 0, duration));
      });
      root.append(svg);
      this.views.set(metric, { cursor, dot, svg, x, y, min, max });
    }
    this.setTime(this.time);
  }

  setTime(time) {
    if (!this.trajectory) return;
    this.time = clamp(time, 0, this.trajectory.duration);
    const frame = this.trajectory.evaluate(this.time);
    for (const metric of JOINT_PLOT_METRICS) {
      const view = this.views.get(metric);
      if (!view) continue;
      const value = frame[VALUES[metric]][this.jointIndex];
      const px = view.x(this.time);
      view.cursor.setAttribute('x1', String(px));
      view.cursor.setAttribute('x2', String(px));
      view.dot.style.display = finite(value) ? '' : 'none';
      if (finite(value)) {
        view.dot.setAttribute('cx', String(px));
        view.dot.setAttribute('cy', String(view.y(value)));
      }
      view.svg.setAttribute('aria-valuenow', this.time.toFixed(3));
      view.svg.setAttribute('aria-valuetext', this.time.toFixed(2) + ' seconds');
      this.outputs[metric].textContent = finite(value)
        ? numberLabel(value) + ' ' + this.unitFor(metric)
        : 'Undefined at phase switch';
    }
  }
}
