// Run with: node tests/simulation.cjs
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(require('node:path').join(__dirname, '../index.html'), 'utf8');
const script = html.split("'use strict';")[1].split('const rowNames=')[0];
const context = vm.createContext({document: {}, console});
vm.runInContext(script + `
let view = null;
function updateHUD() {}
resetState();
globalThis.model = {
 step: stepSimulation, reset: resetState,
 noise(a,b) { q=a; r=b; },
 state() { return { elapsed, cycle, nextPhase, x, P, z, innovation, K, truth, prediction }; }
};`, context);
const model = context.model;
model.step();
assert.equal(model.state().elapsed, 0.1);
const truthAfterPrediction = JSON.stringify(model.state().truth);
model.step();
assert.equal(model.state().elapsed, 0.1, 'Correction must not advance the clock');
assert.equal(JSON.stringify(model.state().truth), truthAfterPrediction);
assert.equal(model.state().cycle, 1);
// A Cholesky factorization checks positive definiteness, not just positive diagonal entries.
function checkCovariance(P) {
 const L = Array.from({length:6}, () => Array(6).fill(0));
 for (let i=0;i<6;i++) for (let j=0;j<=i;j++) {
  assert.ok(Math.abs(P[i][j]-P[j][i])<1e-10, 'Covariance must remain symmetric');
  let v=P[i][j];
  for(let k=0;k<j;k++) v-=L[i][k]*L[j][k];
  if(i===j) { assert.ok(v>0, 'Covariance must be positive definite'); L[i][j]=Math.sqrt(v); }
  else L[i][j]=v/L[j][j];
 }
}
for (const [q,r] of [[0.001,0.1],[0.05,1.2],[1,10]]) {
 model.reset(); model.noise(q,r);
 for(let n=0;n<2000;n++) {
  model.step(); model.step();
  const state=model.state();
  for(const a of [state.x,state.P,state.K,state.z]) assert.ok(a.flat().every(Number.isFinite));
  checkCovariance(state.P);
 }
 assert.equal(model.state().elapsed,200);
}
model.reset();
assert.equal(model.state().elapsed,0);
assert.equal(model.state().cycle,0);
assert.ok(model.state().K.flat().every(v=>v===0));
assert.ok(model.state().z.flat().every(v=>v===0));
assert.equal(model.state().nextPhase,'predict');
console.log('PASS: phase timestamps, 6,000 cycles at noise bounds, covariance stability, and reset.');
// Manual commands and recorded GPS must reproduce the entire filter, not just its path.
vm.runInContext(`
Object.assign(model, {
 beginRecording() { driveMode='manual'; recording=true; recordedFrames=[]; resetState(); },
 input(throttle,steer) { keys.clear(); if(throttle>0)keys.add('ArrowUp');if(throttle<0)keys.add('ArrowDown');if(steer>0)keys.add('ArrowRight');if(steer<0)keys.add('ArrowLeft'); },
 beginReplay() { recording=false;driveMode='replay';resetState(); },
 frames() { return recordedFrames; },
 track(t) { return truthAt(t); },
 lap() { return trackLength/6; },
 running() { return running; }
});`, context);
model.beginRecording();
for(let i=0;i<120;i++){
 model.input(i<35?1:i>90?-1:0,i>20&&i<60?1:i>70&&i<90?-1:0);
 model.noise(i<50?0.05:0.2,i<60?1.2:2.4);
 model.step();model.step();
}
assert.equal(model.frames().length,120);
const saved=JSON.stringify(model.state());
assert.ok(model.frames().some(f=>f.input[1]!==0));
model.beginReplay();
for(let i=0;i<120;i++){model.step();model.step();}
assert.equal(JSON.stringify(model.state()),saved,'Replay must exactly reproduce truth, measurements, gain, state and covariance');
assert.equal(model.running(),false,'Replay must stop after its final correction');
const start=model.track(0),end=model.track(model.lap());
assert.ok(Math.hypot(start.x-end.x,start.y-end.y)<1e-10,'Track must close');
let straight=false,turn=false;
for(let t=0;t<model.lap()-0.1;t+=0.1){const a=model.track(t),b=model.track(t+0.1);const delta=Math.atan2(Math.sin(b.angle-a.angle),Math.cos(b.angle-a.angle));if(Math.abs(delta)<1e-8)straight=true;if(Math.abs(delta)>0.01)turn=true;}
assert.ok(straight&&turn,'Track must contain both straight and turning segments');
console.log('PASS: noncircular closed track, manual inputs, and exact 120-sample replay.');
// Braking crosses zero into reverse, retains vehicle heading, and remains replayable.
model.beginRecording();
const initialHeading=model.state().truth.angle;
model.input(-1,0);
for(let i=0;i<45;i++){model.step();model.step();}
assert.equal(model.state().truth.speed,-6,'Reverse speed must reach its bounded limit');
assert.equal(model.state().truth.angle,initialHeading,'Reversing alone must not flip the car heading');
assert.ok(model.state().truth.vx*Math.cos(initialHeading)+model.state().truth.vy*Math.sin(initialHeading)<0);
model.input(-1,1);model.step();model.step();
assert.ok(model.state().truth.angle<initialHeading,'Steering yaw must reverse with signed speed');
const reversingState=JSON.stringify(model.state());
const reverseFrames=model.frames().length;
model.beginReplay();for(let i=0;i<reverseFrames;i++){model.step();model.step();}
assert.equal(JSON.stringify(model.state()),reversingState,'Reverse and steering must replay exactly');
model.beginRecording();model.input(1,0);
for(let i=0;i<60;i++){model.step();model.step();}
assert.equal(model.state().truth.speed,15,'Forward speed must remain bounded');
vm.runInContext('model.offset = mahalanobisOffset;',context);
for(const P of [[[1,0],[0,1]],[[4,1.2],[1.2,0.8]],[[0.2,-0.7],[-0.7,5]]]){
 const det=P[0][0]*P[1][1]-P[0][1]*P[1][0];
 for(const distance of [2,4,6])for(let i=0;i<128;i++){
  const [dx,dy]=model.offset(P,i/128*Math.PI*2,distance);
  const distanceSquared=(P[1][1]*dx*dx-2*P[0][1]*dx*dy+P[0][0]*dy*dy)/det;
  assert.ok(Math.abs(distanceSquared-distance*distance)<1e-10,'Every boundary must have its specified Mahalanobis distance');
 }
}
console.log('PASS: reverse, reverse steering/replay, speed limits, and constant Mahalanobis distance for rotated covariances.');

model.beginRecording();model.input(1,1);model.step();
const priorSnapshot=JSON.stringify(model.state().prediction);
model.step();assert.equal(JSON.stringify(model.state().prediction),priorSnapshot,'Correction must preserve the displayed prediction and its covariance');
assert.notEqual(JSON.stringify(model.state().x),JSON.stringify(model.state().prediction.state),'Corrected state and saved prediction must be independent');
model.reset();assert.equal(JSON.stringify(model.state().prediction.state),JSON.stringify(model.state().x));
console.log('PASS: prediction snapshot stays fixed through correction and resets with the state.');
vm.runInContext('Object.assign(model,{Q:processCovariance,setQ:applyProcessCovariance,QError:processMatrixError});',context);
const correlatedQ=Array.from({length:6},()=>Array(6).fill(0));
for(const i of [0,2,4]){correlatedQ[i][i]=0.02*(i+1);correlatedQ[i+1][i+1]=0.5*(i+1);correlatedQ[i][i+1]=correlatedQ[i+1][i]=0.6*Math.sqrt(correlatedQ[i][i]*correlatedQ[i+1][i+1]);}
assert.equal(model.QError(correlatedQ),null);
model.setQ(correlatedQ);
model.beginRecording();model.input(0,1);
for(let i=0;i<100;i++){model.step();model.step();checkCovariance(model.state().P);}
assert.ok(Math.abs(model.state().prediction.covariance[0][1])>0.001,'Correlated Q must produce tilted uncertainty');
assert.ok(Math.abs(model.state().prediction.covariance[0][0]-model.state().prediction.covariance[1][1])>0.01,'Unequal Q must produce unequal position variances');
const customSnapshot=JSON.stringify(model.state());
model.beginReplay();for(let i=0;i<100;i++){model.step();model.step();}
assert.equal(JSON.stringify(model.state()),customSnapshot,'Replay must restore custom Q at every recorded prediction');
const invalid=correlatedQ.map(row=>row.slice());invalid[0][1]=invalid[1][0]=10;
assert.ok(model.QError(invalid));assert.throws(()=>model.setQ(invalid));
const asymmetric=correlatedQ.map(row=>row.slice());asymmetric[1][0]=0;assert.ok(model.QError(asymmetric));
assert.ok(model.QError([[1]]));assert.ok(model.QError(correlatedQ.map(row=>row.map(()=>NaN))));
assert.equal(model.QError(Array.from({length:6},()=>Array(6).fill(0))),null,'Zero covariance is positive semidefinite');
console.log('PASS: editable correlated Q, elliptical covariance, validation, and exact custom-Q replay.');
vm.runInContext('Object.assign(model,{markerPose:motionMarkerPose,followBlend});',context);
assert.equal(model.markerPose(6,0,false).shape,'dot');
assert.equal(model.markerPose(0,0,true).shape,'dot');
assert.equal(model.markerPose(6,0,true).shape,'triangle');
assert.ok(Math.abs(model.markerPose(0,6,true).yaw)<1e-12);
assert.ok(Math.abs(model.markerPose(-6,0,true).yaw+Math.PI/2)<1e-12,'A reverse-motion triangle follows velocity, not body heading');
assert.equal(model.markerPose(NaN,0,true).shape,'dot');
function remainingError(hz){let error=1;for(let i=0;i<hz;i++)error*=1-model.followBlend(1/hz);return error;}
assert.ok(Math.abs(remainingError(30)-remainingError(120))<1e-12,'Camera smoothing must be frame-rate independent');
assert.ok(model.followBlend(1/60)>0&&model.followBlend(1/60)<1,'Follow must ease rather than snap');
console.log('PASS: model-derived marker direction and frame-rate-independent smooth camera follow.');
