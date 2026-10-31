import test from 'node:test';
import assert from 'node:assert/strict';
import { validateJointLimits, validateDhModel, resolveSamplingRanges,
  forwardKinematicsPosition, generateWorkspaceSamples } from '../kinematics/workspace-core.js';

const close = (a,b,tol=1e-5) => assert.ok(Math.abs(a-b)<=tol, `${a} != ${b}`);
const twoR = () => [
  { type:'R',a:2,alpha:0,d:0,theta:30 },
  { type:'R',a:1,alpha:0,d:0,theta:-45 },
];

test('validates limits, including locked joints and unspecified limits',()=>{
  assert.equal(validateJointLimits(null),null);
  assert.deepEqual(validateJointLimits({min:4,max:4}),{min:4,max:4});
  assert.throws(()=>validateJointLimits({min:3,max:2}),RangeError);
  assert.throws(()=>validateJointLimits({min:-Infinity,max:2}),RangeError);
  assert.throws(()=>validateJointLimits({min:1}),RangeError);
});
test('model is valid even when current pose exceeds limits',()=>{
  const r=twoR(); r[0].limits={min:-10,max:10};
  assert.doesNotThrow(()=>validateDhModel(r));
  assert.throws(()=>validateDhModel([{...r[0],a:NaN}]),TypeError);
});
test('sampling domain is separate from mechanical limits',()=>{
  const r=twoR(); r[0].limits={min:-170,max:170};
  assert.deepEqual(resolveSamplingRanges(r),[{min:-170,max:170},{min:-180,max:180}]);
  assert.deepEqual(resolveSamplingRanges(r,[{min:0,max:90},null]),
    [{min:0,max:90},{min:-180,max:180}]);
  assert.throws(()=>resolveSamplingRanges(r,[{min:-180,max:10},null]),RangeError);
  assert.throws(()=>resolveSamplingRanges(r,[null]),TypeError);
});
test('unbounded prismatic joint needs an exploration domain',()=>{
  const r=[{type:'P',a:0,alpha:0,d:1,theta:0}];
  assert.throws(()=>resolveSamplingRanges(r),RangeError);
  assert.deepEqual(resolveSamplingRanges(r,[{min:0,max:100}]),[{min:0,max:100}]);
});
test('pure FK agrees with analytic planar 2R and does not mutate model',()=>{
  const r=twoR(), before=structuredClone(r);
  const [x,y,z]=forwardKinematicsPosition(r);
  close(x,2*Math.cos(Math.PI/6)+Math.cos(-Math.PI/12));
  close(y,2*Math.sin(Math.PI/6)+Math.sin(-Math.PI/12));
  close(z,0);
  close(forwardKinematicsPosition(r,[0,0])[0],3);
  assert.deepEqual(r,before);
});
test('prismatic axis follows rotated previous frame',()=>{
  const r=[{type:'R',a:0,alpha:90,d:0,theta:0},
           {type:'P',a:0,alpha:0,d:1,theta:0}];
  const [x,y,z]=forwardKinematicsPosition(r,[0,3]);
  close(x,0); close(y,-3); close(z,0);
});
test('grid samples are reachable and include annulus extrema',()=>{
  const r=twoR(), before=structuredClone(r);
  const s=generateWorkspaceSamples(r,{method:'grid',samplesPerJoint:3});
  assert.equal(s.sampleCount,9);
  let min=Infinity,max=0;
  for(let i=0;i<9;i++){
    const p=s.positions.slice(i*3,i*3+3);
    const q=s.jointValues.slice(i*2,i*2+2);
    const expected=forwardKinematicsPosition(r,q);
    for(let j=0;j<3;j++)close(p[j],expected[j]);
    const radius=Math.hypot(p[0],p[1]);min=Math.min(min,radius);max=Math.max(max,radius);
  }
  close(min,1);close(max,3);assert.deepEqual(r,before);
});
test('halton is deterministic and respects locked domains',()=>{
  const opts={sampleCount:29,samplingRanges:[{min:25,max:25},{min:-90,max:40}]};
  const a=generateWorkspaceSamples(twoR(),opts), b=generateWorkspaceSamples(twoR(),opts);
  assert.deepEqual(a.positions,b.positions);
  assert.deepEqual(a.jointValues,b.jointValues);
  for(let i=0;i<29;i++)assert.equal(a.jointValues[2*i],25);
});
test('rejects bad sampling options and excessive allocations',()=>{
  assert.throws(()=>generateWorkspaceSamples(twoR(),{method:'random'}),RangeError);
  assert.throws(()=>generateWorkspaceSamples(twoR(),{sampleCount:0}),RangeError);
  assert.throws(()=>generateWorkspaceSamples(twoR(),{sampleCount:200001}),RangeError);
  assert.throws(()=>generateWorkspaceSamples(twoR(),{method:'grid',samplesPerJoint:500}),RangeError);
});

test('spatial mixed R/P FK agrees with independently multiplied standard-DH matrices', () => {
  const robot = [
    { type: 'R', a: 90, alpha: 90, d: 110, theta: 40 },
    { type: 'P', a: 140, alpha: -40, d: 20, theta: 30 },
    { type: 'R', a: 70, alpha: 25, d: -30, theta: 80 },
    { type: 'R', a: -80, alpha: 75, d: 200, theta: -75 }
  ];
  const multiply = (a, b) => {
    const out = Array(16).fill(0);
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        for (let k = 0; k < 4; k++) out[4 * i + j] += a[4 * i + k] * b[4 * k + j];
      }
    }
    return out;
  };
  const reference = q => {
    let transform = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
    robot.forEach((joint, i) => {
      const theta = (joint.type === 'R' ? q[i] : joint.theta) * Math.PI / 180;
      const alpha = joint.alpha * Math.PI / 180;
      const d = joint.type === 'P' ? q[i] : joint.d;
      const c = Math.cos(theta), s = Math.sin(theta);
      const ca = Math.cos(alpha), sa = Math.sin(alpha);
      const dh = [
        c,-s*ca,s*sa,joint.a*c,
        s,c*ca,-c*sa,joint.a*s,
        0,sa,ca,d,
        0,0,0,1
      ];
      transform = multiply(transform, dh);
    });
    return [transform[3], transform[7], transform[11]];
  };
  for (let i = 0; i < 60; i++) {
    const q = [-180 + 6 * i, -45 + 1.5 * i, (i * 29) % 360 - 180, (i * 79) % 360 - 180];
    const result = forwardKinematicsPosition(robot, q);
    const expected = reference(q);
    for (let k = 0; k < 3; k++) close(result[k], expected[k], 1e-9);
  }
});
