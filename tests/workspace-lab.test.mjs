import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const read = path => readFileSync(new URL('../' + path, import.meta.url), 'utf8');
const page = read('kinematics/workspace.html');
const lab = read('kinematics/workspace-lab.js');
const index = read('index.html');
const nav = read('shared/navigation.js');

test('workspace HTML has unique IDs and all required control roots', () => {
  const ids = [...page.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
  assert.equal(new Set(ids).size, ids.length);
  for (const id of [
    'canvas-container', 'dh-table-body', 'add-row-btn', 'pose-rows',
    'domain-rows', 'robot-preset', 'sampling-method', 'sample-count',
    'grid-steps', 'grid-summary', 'generate-workspace', 'workspace-status', 'sample-summary',
    'sample-index', 'go-to-sample', 'clear-selected', 'selected-position',
    'selected-joint-values', 'selection-empty', 'selection-details', 'selection-feedback',
    'show-workspace', 'show-robot', 'point-size', 'point-size-value',
    'point-opacity', 'point-opacity-value', 'metrics-rows', 'metrics-count',
    'metrics-empty', 'metrics-details', 'section-panel',
    'section-enabled', 'section-axis', 'section-position', 'section-position-value',
    'section-thickness', 'section-thickness-value', 'section-context',
    'section-count', 'section-empty', 'section-plot', 'section-canvas',
    'section-marker', 'section-plane-label'
  ]) assert.ok(ids.includes(id), `Missing #${id}`);
});

test('all literal el() calls reference an element in workspace.html', () => {
  const ids = new Set([...page.matchAll(/\bid="([^"]+)"/g)].map(m => m[1]));
  for (const [,id] of lab.matchAll(/\bel\('([^']+)'\)/g)) {
    assert.ok(ids.has(id), `Missing DOM node #${id}`);
  }
});

test('workspace loads the shared viewer and the pure numerical core', () => {
  assert.match(lab, /from '\.\/shared-robot-viewer\.js'/);
  assert.match(lab, /from '\.\/workspace-core\.js'/);
  assert.match(page, /src="\.\/workspace-lab\.js"/);
  assert.match(page, /type="importmap"/);
});

test('workspace is discoverable in homepage and shared navigation', () => {
  assert.match(index, /kinematics\/workspace\.html/);
  assert.match(nav, /kinematics\/workspace\.html/);
});

test('sample picking retains the generated buffers and does not call IK', () => {
  assert.match(lab, /from '\.\/workspace-selection\.js'/);
  assert.match(lab, /lastGenerated = result/);
  assert.match(lab, /pickRaycaster\.intersectObject\(cloud, false\)/);
  assert.match(lab, /rankProjectedSampleHits\(/);
  assert.match(lab, /selectionMarker\.visible/);
  assert.doesNotMatch(lab, /solveIKPosition|solveIKFull6DoF/);
});

test('grid control accepts an integer, previews the count, and scales with robot DOF', () => {
  assert.match(page, /<input id="grid-steps" type="number"/);
  assert.match(lab, /gridResolutionSettings\(jointCount\)\.recommendedPerJoint/);
  assert.match(lab, /gridPointCount\(jointCount, value\)/);
  assert.match(lab, /el\('grid-steps'\)\.addEventListener\('input', \(\) =>/);
});

test('view presets are registered by RobotViewer only, without duplicate lab handlers', () => {
  assert.doesNotMatch(lab, /el\('btn-view-' \+ view\)\.addEventListener\('click'/);
});

test('sampling resolution edits invalidate stale clouds immediately', () => {
  assert.match(lab, /el\('grid-steps'\)\.addEventListener\('input', \(\) => \{/);
  assert.match(lab, /clearWorkspace\('Sampling resolution changed/);
});

test('W4 display controls and metrics are wired independently of sampling', () => {
  assert.match(lab, /from '\.\/workspace-metrics\.js'/);
  assert.match(lab, /summarizeWorkspaceSamples\(result\)/);
  assert.match(lab, /renderSampleMetrics\(metrics\)/);
  assert.match(lab, /resetSampleMetrics\(\)/);
  assert.match(lab, /el\('show-robot'\)\.addEventListener\('change', applyDisplaySettings\)/);
  assert.match(lab, /cloud\.material\.size = size/);
  assert.match(lab, /cloud\.material\.opacity = opacity/);
  assert.match(lab, /viewer\.robotGroup\.visible = el\('show-robot'\)\.checked/);
  assert.match(page, /id="metrics-panel"/);
  assert.match(page, /id="display-panel"/);
});

test('W4 sampled sections replace redundant 3-view projections', () => {
  assert.match(lab, /from '\.\/workspace-sections\.js'/);
  assert.match(lab, /new WorkspaceSectionView\(/);
  assert.match(lab, /sliceWorkspaceSamples\(lastGenerated, axis, center, thickness\)/);
  assert.match(lab, /sectionView\.show\(lastGenerated, sampleMetrics, slice\)/);
  assert.match(lab, /sectionView\.highlight\(index\)/);
  assert.match(lab, /sectionView\.clear\(\)/);
  assert.match(lab, /sectionCloud\.name = 'sampled-workspace-section'/);
  assert.match(lab, /sectionPlane\.name = 'workspace-section-plane'/);
  assert.match(lab, /const mappedHits = el\('section-enabled'\)\.checked/);
  assert.match(page, /id="section-panel"/);
  assert.match(page, /does not compute a continuous intersection/i);
  assert.doesNotMatch(page, /id="projection-xy"/);
});
