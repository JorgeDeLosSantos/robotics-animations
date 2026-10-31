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
    'grid-steps', 'generate-workspace', 'workspace-status', 'sample-summary'
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
