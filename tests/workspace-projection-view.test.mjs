import test from 'node:test';
import assert from 'node:assert/strict';
import { WorkspaceProjectionViews } from '../kinematics/workspace-projection-view.js';
import { WORKSPACE_PLANES, projectPosition } from '../kinematics/workspace-projections.js';
import { summarizeWorkspaceSamples } from '../kinematics/workspace-metrics.js';

function testEnvironment() {
  const previous = {
    document: globalThis.document,
    window: globalThis.window,
    requestAnimationFrame: globalThis.requestAnimationFrame,
    ResizeObserver: globalThis.ResizeObserver
  };
  const elements = new Map();
  let draws = 0;
  for (const plane of WORKSPACE_PLANES) {
    const canvas = {
      width: 300,
      height: 150,
      handlers: new Map(),
      addEventListener(type, fn) { this.handlers.set(type, fn); },
      getBoundingClientRect() { return { left: 0, top: 0, width: 360, height: 240 }; },
      getContext() {
        return {
          setTransform() {}, fillRect() { draws++; }, setLineDash() {},
          beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}
        };
      }
    };
    elements.set('projection-' + plane.id, canvas);
    elements.set('projection-marker-' + plane.id, { style: {}, hidden: true });
  }
  elements.set('projections-empty', { hidden: false });
  elements.set('projections-charts', { hidden: true });
  globalThis.document = { getElementById: id => elements.get(id) };
  globalThis.window = {
    devicePixelRatio: 1, RoboticsTheme: { getTheme: () => 'dark' },
    addEventListener() {}, removeEventListener() {}
  };
  globalThis.requestAnimationFrame = cb => cb();
  globalThis.ResizeObserver = undefined;
  return {
    elements,
    get draws() { return draws; },
    restore() {
      for (const [key, value] of Object.entries(previous)) {
        if (value === undefined) delete globalThis[key];
        else globalThis[key] = value;
      }
    }
  };
}

test('three plotted canvases and linked markers update without repaint', () => {
  const env = testEnvironment();
  let selected = null;
  const view = new WorkspaceProjectionViews({
    getSelectedIndex: () => selected,
    onPick: index => { selected = index; }
  });
  try {
    const samples = {
      sampleCount: 3,
      positions: new Float32Array([0, 0, 0, 100, 0, 0, 0, 0, 100])
    };
    view.show(samples, summarizeWorkspaceSamples(samples));
    assert.equal(env.elements.get('projections-charts').hidden, false);
    assert.ok(env.draws >= 12);
    const originalDraws = env.draws;
    selected = 1;
    view.highlight(1);
    assert.equal(env.draws, originalDraws, 'highlights must not repaint clouds');
    for (const plane of WORKSPACE_PLANES) {
      const marker = env.elements.get('projection-marker-' + plane.id);
      assert.equal(marker.hidden, false);
      assert.match(marker.style.left, /%$/);
      assert.match(marker.style.top, /%$/);
    }
    const xy = view.charts.find(c => c.plane.id === 'xy');
    const xyPoint = projectPosition([100, 0, 0], xy.layout);
    xy.canvas.handlers.get('pointerdown')({
      button: 0, isPrimary: true, pointerId: 1,
      clientX: xyPoint.x, clientY: xyPoint.y
    });
    xy.canvas.handlers.get('pointerup')({
      pointerId: 1, clientX: xyPoint.x, clientY: xyPoint.y
    });
    assert.equal(selected, 1);
    view.clear();
    assert.equal(env.elements.get('projections-charts').hidden, true);
    for (const plane of WORKSPACE_PLANES) {
      assert.equal(env.elements.get('projection-marker-' + plane.id).hidden, true);
    }
  } finally {
    view.destroy();
    env.restore();
  }
});

test('drag in a projection canvas does not trigger sample selection', () => {
  const env = testEnvironment();
  let picks = 0;
  const view = new WorkspaceProjectionViews({
    getSelectedIndex: () => null,
    onPick: () => { picks++; }
  });
  try {
    const samples = {
      sampleCount: 1, positions: new Float32Array([0, 0, 0])
    };
    view.show(samples, summarizeWorkspaceSamples(samples));
    const canvas = view.charts[0].canvas;
    canvas.handlers.get('pointerdown')({
      button: 0, isPrimary: true, pointerId: 2, clientX: 120, clientY: 100
    });
    canvas.handlers.get('pointerup')({
      pointerId: 2, clientX: 240, clientY: 100
    });
    assert.equal(picks, 0);
  } finally {
    view.destroy();
    env.restore();
  }
});
