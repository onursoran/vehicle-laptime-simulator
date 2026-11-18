// Browser data-flow and incremental-rendering checks against the actual script.
// This is a DOM contract harness, not a substitute for visual browser QA.
import * as fs from 'node:fs';
import assert from 'node:assert/strict';
import {setTimeout as wait} from 'node:timers/promises';

const root = new URL('./', import.meta.url);
const html = fs.readFileSync(new URL('simulator.html', root), 'utf8');
const script = fs.readFileSync(new URL('simulator.js', root), 'utf8');
const fixture = JSON.parse(fs.readFileSync(new URL('tests/fixtures/simulator.json', root), 'utf8'));
const generatedModules = [];
fs.mkdirSync(new URL('tmp/ui-tests/', root), {recursive:true});
// UI-only rows: the benchmark fixture may contain either two or six results.
// Replicated states test presentation and must never be exported as physics results.
const mockResults = [fixture.results[0], ...Array.from({length:4}, (_,i) => ({
  ...structuredClone(fixture.results[0]), name:'Mock passive configuration ' + (i + 1),
})), fixture.results.at(-1)];
const runId = Date.now();
const previousGlobals = Object.fromEntries(['document','FormData','fetch','ResizeObserver','requestAnimationFrame','setTimeout'].map(name => [name, globalThis[name]]));
const defaults = {...fixture.inputs, track:'oval', profile:'second-example', configurations:'active', width:3, spacing:2, front_track:1.2446, rear_track:1.2192};
const profileCatalog = {
  'second-example': {label:'Second example profile', description:'Mock profile for preset switching.', default_inputs:{front_track:1.2446, rear_track:1.2192}},
  illustrative: {label:'Original illustrative vehicle', description:'Original example inputs.', default_inputs:{front_track:1.2, rear_track:1.2}},
};

async function until(predicate, description) {
  for (let tries = 0; tries < 200; tries++) {if (predicate()) return; await wait(2);}
  throw new Error('Timed out waiting for ' + description);
}

try {
for (const width of [320, 340, 1000]) {
  const elements = new Map();
  class Element {
    constructor() {
      this.events={}; this.dataset={}; this.value=''; this.textContent=''; this.hidden=false; this.disabled=false;
      this.attributes={}; this._html=''; this.htmlWrites=0; this.attributeWrites=0;
      this.classList={toggle:(name,value)=>{this.attributes[name]=value;}};
    }
    addEventListener(name, handler) {this.events[name]=handler;}
    setAttribute(name, value) {this.attributes[name]=String(value); this.attributeWrites++;}
    getBoundingClientRect() {return {left:0,width:this.clientWidth};}
    set innerHTML(value) {
      this._html=value; this.htmlWrites++;
      this.rows=[...value.matchAll(/<tr data-index="(\d+)"/g)].map(match=>{const row=new Element();row.dataset.index=match[1];return row;});
      for (const match of value.matchAll(/\bid="([^"]+)"/g)) elements.set(match[1], new Element());
    }
    get innerHTML() {return this._html;}
    querySelectorAll(selector) {if(selector==='tr')return this.rows||[];throw new Error('Unexpected selector ' + selector);}
  }
  for (const match of html.matchAll(/\bid="([^"]+)"/g)) elements.set(match[1], new Element());
  const form=elements.get('settings'), controls=[], outputs=new Map();
  for(const match of html.matchAll(/<input\b[^>]*name="([^"]+)"[^>]*>/g)) {
    const input=new Element();input.name=match[1];input.value=match[0].match(/\bvalue="([^"]+)"/)?.[1]||'';
    input.step=match[0].match(/\bstep="([^"]+)"/)?.[1]||'1';input.dataset.unit=match[0].match(/data-unit="([^"]*)"/)?.[1]||'';
    controls.push(input);outputs.set(input.name,new Element());
  }
  for(const match of html.matchAll(/<select\b[^>]*name="([^"]+)"[^>]*>([\s\S]*?)<\/select>/g)) {
    const input=new Element();input.name=match[1];const options=[...match[2].matchAll(/<option value="([^"]+)"([^>]*)>/g)];
    input.value=(options.find(option=>option[2].includes('selected'))||options[0])[1];controls.push(input);
  }
  const control = name => controls.find(input => input.name === name);
  form.querySelectorAll=selector=>{assert.equal(selector,'input[type="range"]');return controls.filter(input=>outputs.has(input.name));};
  form.querySelector=selector=>selector.includes('data-for=') ? outputs.get(selector.match(/data-for="([^"]+)"/)[1]) : control(selector.match(/name="([^"]+)"/)[1]);
  let submitPromise;
  form.requestSubmit=()=>{submitPromise=form.events.submit({preventDefault(){}});};
  const requests=[], frames=[], jobs=new Map(), observers=[];
  let nextJob=1, failNextJob=false, holdJobCreation=false, heldJob=null, holdNextPreview=false, heldPreview=null;
  function resultFor(values, count) {
    const source=structuredClone(fixture);
    source.inputs={...defaults,...values};
    source.track.name=values.track + ' test layout';
    source.track.metadata={kind:'real', source_url:'https://github.com/TUMFTM/racetrack-database', layout_note:'2020 layout snapshot <approximate>', width_note:'Uniform width override.'};
    source.track.width_m=source.track.width_m.map(()=>values.width);
    source.results=structuredClone(values.configurations==='all' ? mockResults : [mockResults[0],mockResults.at(-1)]).slice(0,count);
    if (values.configurations === 'fiddle' && source.results.length > 1) {
      source.results[1].name = 'Inside-rear fiddle brake 40%';
      source.results[1].auxiliary_braking_forces_n = source.track.distance_m.map(()=>[0,0,25,0]);
    }
    for(const result of source.results) result.braking={
      segment_category:source.track.distance_m.map((_,i)=>['none','light','medium','hard'][i%4]),
      peak_deceleration_g:1.12, distance_m:{none:100,light:20,medium:30,hard:40}, zones:[{category:'hard'}],
    };
    return source;
  }
  const ok = value => ({ok:true,json:async()=>value});
  globalThis.document={getElementById(id){assert.ok(elements.has(id),'Missing element ' + id);return elements.get(id);}};
  globalThis.FormData=class {constructor(){this.values=controls.map(input=>[input.name,input.value]);}[Symbol.iterator](){return this.values[Symbol.iterator]();}};
  globalThis.fetch=async(url,options={})=>{
    const body=options.body ? JSON.parse(options.body) : null;
    requests.push({url,body});
    if(url==='/api/config') return ok({default_inputs:defaults,profiles:profileCatalog});
    if(url==='/api/preview') {
      const result=resultFor(body,0);delete result.results;
      if(holdNextPreview) {holdNextPreview=false;return await new Promise(resolve=>{heldPreview=()=>resolve(ok(result));});}
      return ok(result);
    }
    if(url==='/api/jobs') {
      const id=String(nextJob++),total=body.configurations==='all'?6:2;
      jobs.set(id,{values:body,total,polls:0,fail:failNextJob}); failNextJob=false;
      const job={job_id:id,status:'queued',version:0,completed:0,total,phase:'Queued'};
      if(holdJobCreation) {holdJobCreation=false;return await new Promise(resolve=>{heldJob={id,release:()=>resolve(ok(job))};});}
      return ok(job);
    }
    const match=url.match(/^\/api\/jobs\/([^/?]+)(\/cancel|\?after=(-?\d+))$/);
    assert.ok(match,'Unexpected endpoint ' + url);
    const job=jobs.get(match[1]);assert.ok(job);
    if(match[2]==='/cancel') {job.cancelled=true;return ok({status:'cancelled'});}
    job.polls++;
    if(job.fail)return ok({job_id:match[1],status:'failed',version:1,completed:0,total:job.total,error:'Synthetic failure'});
    const complete=job.polls>=2;
    const count=complete?job.total:1;
    assert.equal(Number(match[3]),complete?1:0,'Version polling must advance');
    return ok({job_id:match[1],status:complete?'complete':'running',version:job.polls,completed:count,total:job.total,phase:complete?'Complete':'Calculating independent',elapsed_s:job.polls,result:resultFor(job.values,count)});
  };
  globalThis.ResizeObserver=class {constructor(callback){observers.push(callback);}observe(){}};
  globalThis.requestAnimationFrame=callback=>{frames.push(callback);return frames.length;};
  globalThis.setTimeout=(callback,ms,...args)=>previousGlobals.setTimeout(callback,ms===450||ms===120?0:ms,...args);
  elements.get('track-scene').clientWidth=width<600?width-68:520;
  elements.get('car-scene').clientWidth=width<600?width-68:270;
  elements.get('speed-chart').clientWidth=width-68;
  const url=new URL('tmp/ui-tests/.responsive-ui-'+width+'-'+runId+'.mjs',root);
  fs.writeFileSync(url,script);
  generatedModules.push(url);
  await import(url.href);
  await until(()=>/Track ready/.test(elements.get('status').textContent),'initial independent track preview');
  assert.equal(requests.filter(r=>r.url==='/api/jobs').length,0,'Loading the page must not trigger a full solve');
  assert.equal(elements.get('result-panel').hidden,false);
  assert.equal(elements.get('play').disabled,true);
  assert.equal(control('front_track').value,'1.2446');
  assert.match(elements.get('track-scene').innerHTML,/<path/);
  assert.equal(elements.get('lap-time').textContent,'—');
  assert.equal(elements.get('run').disabled,false);

  form.requestSubmit(); await submitPromise;
  assert.match(elements.get('status').textContent,/Comparison complete/);
  assert.equal(requests.find(r=>r.url==='/api/jobs').body.configurations,'active');
  assert.equal(elements.get('comparison').rows.length,2);
  assert.equal(elements.get('run').disabled,false);
  assert.equal(elements.get('play').disabled,false);
  assert.match(elements.get('braking-summary').textContent,/peak 1.12 g/);
  assert.match(elements.get('track-source').innerHTML,/&lt;approximate&gt;/);
  assert.match(elements.get('track-source').innerHTML,/openstreetmap.org\/copyright/);
  for(const id of ['track-scene','car-scene','speed-chart']) assert.ok(!/NaN|undefined/.test(elements.get(id).innerHTML));
  for(const id of ['track-scene','speed-chart']) for(const category of ['light','medium','hard']) assert.ok(elements.get(id).innerHTML.includes('data-braking="'+category+'"'));

  elements.get('strategy').events.change({target:{value:'1'}});
  assert.equal(elements.get('comparison').rows[1].attributes.selected,true);
  elements.get('seek').events.input({target:{value:'500'}});
  assert.equal(Number(elements.get('seek').value),500);
  const staticWriteCounts=['track-scene','car-scene','speed-chart','wheel-details'].map(id=>elements.get(id).htmlWrites);
  elements.get('play').events.click();
  frames.shift()(1000);frames.shift()(1100);frames.shift()(1200);
  assert.ok(Number(elements.get('seek').value)>500);
  assert.deepEqual(['track-scene','car-scene','speed-chart','wheel-details'].map(id=>elements.get(id).htmlWrites),staticWriteCounts,'Playback must not rebuild static paths or tables');
  assert.ok(elements.get('map-car').attributeWrites>0);
  elements.get('play').events.click();
  const beforeResize=elements.get('track-scene').htmlWrites;
  observers[0]();observers[0]();
  assert.equal(elements.get('track-scene').htmlWrites,beforeResize,'Resize observer callbacks with unchanged widths must not rebuild');
  elements.get('track-scene').clientWidth+=10;observers[0]();
  assert.equal(elements.get('track-scene').htmlWrites,beforeResize+1);

  control('front_bias').value='65';control('strength').value='40';
  form.events.input({target:control('front_bias')});
  assert.equal(elements.get('play').disabled,true);
  assert.match(elements.get('result-state').textContent,/Previous result/);
  control('configurations').value='all';form.requestSubmit();await submitPromise;
  const latestJob=requests.filter(r=>r.url==='/api/jobs').at(-1);
  assert.equal(latestJob.body.front_bias,65);assert.equal(latestJob.body.strength,40);
  assert.equal(elements.get('comparison').rows.length,6);

  control('configurations').value='fiddle';form.requestSubmit();await submitPromise;
  assert.equal(requests.filter(r=>r.url==='/api/jobs').at(-1).body.configurations,'fiddle');
  assert.equal(elements.get('comparison').rows.length,2);
  elements.get('strategy').events.change({target:{value:'1'}});
  assert.equal(elements.get('wheel-aux-2').textContent,'25.0');
  assert.equal(elements.get('inset-wheel-2').attributes.stroke,'var(--orange)');
  assert.equal(elements.get('inset-wheel-3').attributes.stroke,'none');
  elements.get('strategy').events.change({target:{value:'0'}});
  assert.equal(elements.get('inset-wheel-2').attributes.stroke,'none');

  // Even a mock transport that ignores abort must not replace a newer preview.
  holdNextPreview=true;
  control('track').value='monza';form.events.change({target:control('track')});
  await until(()=>!!heldPreview,'held preview');
  assert.equal(control('width').value,'12');assert.equal(control('spacing').value,'4');
  control('track').value='catalunya';form.events.change({target:control('track')});
  await until(()=>elements.get('track-caption').textContent.includes('catalunya'),'newest geometry preview');
  heldPreview();await wait(0);
  assert.match(elements.get('track-caption').textContent,/catalunya/);
  assert.equal(elements.get('play').disabled,true);
  assert.equal(elements.get('lap-time').textContent,'—');

  // A changed car profile restores only its track dimensions, preserving bias preferences.
  control('profile').value='illustrative';form.events.change({target:control('profile')});
  await until(()=>requests.filter(r=>r.url==='/api/preview').at(-1).body.profile==='illustrative','profile preview request');
  assert.equal(control('front_track').value,'1.2');assert.equal(control('rear_track').value,'1.2');
  assert.equal(control('front_bias').value,'65');assert.equal(control('strength').value,'40');
  await wait(0);

  // Abandoning an in-flight job creation still cancels its eventual identifier.
  holdJobCreation=true;form.requestSubmit();
  await until(()=>!!heldJob,'held job creation');
  control('track').value='oval';form.events.change({target:control('track')});
  heldJob.release();await submitPromise;
  assert.ok(requests.some(r=>r.url==='/api/jobs/'+heldJob.id+'/cancel'));
  await until(()=>elements.get('track-caption').textContent.includes('oval'),'post-cancel preview');
  assert.equal(control('width').value,'3');assert.equal(control('spacing').value,'2');
  assert.equal(elements.get('run').disabled,false);
  assert.equal(elements.get('lap-time').textContent,'—');

  failNextJob=true;form.requestSubmit();await submitPromise;
  assert.match(elements.get('status').textContent,/Synthetic failure.*track preview remains/);
  assert.equal(elements.get('result-panel').hidden,false);
  assert.equal(elements.get('run').disabled,false);
}
} finally {
  Object.assign(globalThis, previousGlobals);
  for (const url of generatedModules) fs.rmSync(url, {force:true});
}
export const result='PASS: instant preview without solve, async progress and partial results, version polling, stale-request cancellation, profile presets, two/six comparisons, braking display, and no static SVG/table rebuild during playback at 320/340/1000 px. Browser visual layout was not tested.';
