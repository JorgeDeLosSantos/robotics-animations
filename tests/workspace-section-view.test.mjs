import test from 'node:test';
import assert from 'node:assert/strict';
import { WorkspaceSectionView } from '../kinematics/workspace-section-view.js';
import { projectionLayout, WORKSPACE_PLANES, projectPosition } from '../kinematics/workspace-projections.js';
import { summarizeWorkspaceSamples } from '../kinematics/workspace-metrics.js';
import { sliceWorkspaceSamples } from '../kinematics/workspace-sections.js';

function testEnvironment() {
  const previous = {
    document: globalThis.document,
    window: globalThis.window,
    requestAnimationFrame: globalThis.requestAnimationFrame,
    ResizeObserver: globalThis.ResizeObserver
  };
  const elements = new Map();
  let draws = 0;
  const canvas = {
    width: 300, height: 150, handlers: new Map(),
    addEventListener(type, fn) { this.handlers.set(type, fn); },
    getBoundingClientRect() { return { left: 0, top: 0, width: 360, height: 240 }; },
    getContext() { return {
      setTransform() {}, fillRect() { draws++; }, setLineDash() {},
      beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}
    }; }
  };
  elements.set('section-canvas', canvas);
  elements.set('section-marker', { style: {}, hidden: true });
  elements.set('section-empty', { hidden: false });
  elements.set('section-plot', { hidden: true });
  globalThis.document = { getElementById: id => elements.get(id) };
  globalThis.window = {
    devicePixelRatio: 1, RoboticsTheme: { getTheme: () => 'dark' },
    addEventListener() {}, removeEventListener() {}
  };
  globalThis.requestAnimationFrame = cb => cb();
  globalThis.ResizeObserver = undefined;
  return {
    elements, canvas,
    get draws() { return draws; },
    restore() {
      for (const [key, value] of Object.entries(previous)) {
        if (value === undefined) delete globalThis[key];
        else globalThis[key] = value;
      }
    }
  };
}

test('one section plot shows only band points, with overlay marker that does not repaint', () => {
  const env = testEnvironment();
  let selected = null;
  const view = new WorkspaceSectionView({
    getSelectedIndex: () => selected, onPick: index => { selected = index; }
  });
  try {
    const samples = {
      sampleCount: 3,
      positions: new Float32Array([0, 0, 0, 100, 0, 0, 0, 0, 100])
    };
    const metrics = summarizeWorkspaceSamples(samples);
    const band = sliceWorkspaceSamples(samples, 'z', 0, 1);
    assert.equal(band.count, 2);
    view.show(samples, metrics, band);
    assert.equal(env.elements.get('section-plot').hidden, false);
    assert.equal(env.draws, 1 + 2); // 1 background + 2 band points
    const before = env.draws;
    selected = 1;
    view.highlight(1);
    assert.equal(env.draws, before);
    assert.equal(env.elements.get('section-marker').hidden, false);
    selected = 2;
    view.highlight(2);
    assert.equal(env.elements.get('section-marker').hidden, true);
    const location = projectPosition([100, 0, 0], view.layout);
    env.canvas.handlers.get('pointerdown')({
      button: 0, isPrimary: true, pointerId: 3, clientX: location.x, clientY: location.y
    });
    env.canvas.handlers.get('pointerup')({
      pointerId: 3, clientX: location.x, clientY: location.y
    });
    assert.equal(selected, 1);
    view.clear();
    assert.equal(env.elements.get('section-plot').hidden, true);
    assert.equal(env.elements.get('section-marker').hidden, true);
  } finally {
    view.destroy();
    env.restore();
  }
});

test('pointer drags in section plot do not pick samples', () => {
  const env = testEnvironment();
  let picks = 0;
  const view = new WorkspaceSectionView({
    getSelectedIndex: () => null, onPick: () => { picks++; }
  });
  try {
    const samples = { sampleCount: 1, positions: new Float32Array([0,0,0]) };
    view.show(samples, summarizeWorkspaceSamples(samples),
      sliceWorkspaceSamples(samples, 'z', 0, 1));
    env.canvas.handlers.get('pointerdown')({
      button: 0, isPrimary: true, pointerId: 2, clientX: 120, clientY: 100
    });
    env.canvas.handlers.get('pointerup')({
      pointerId: 2, clientX: 240, clientY: 100
    });
    assert.equal(picks, 0);
  } finally {
    view.destroy();
    env.restore();
  }
});
