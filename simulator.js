(() => {
  'use strict';
  const byId = id => document.getElementById(id);
  const form = byId('settings'), status = byId('status');
  const control = name => form.querySelector('[name="' + name + '"]');
  const fmt = (number, digits = 1) => Number.isFinite(number) ? number.toFixed(digits) : '—';
  const escape = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const realTracks = new Set(['monza', 'montreal', 'catalunya']);
  const geometryFields = new Set(['track', 'mirrored', 'width', 'spacing', 'profile', 'front_track', 'rear_track']);
  const brakingLabel = {none:'No classified braking', light:'Light braking', medium:'Medium braking', hard:'Hard braking'};
  let view = null, data = null, selected = 0, distance = 0, stale = false, profiles = {};
  let runGeneration = 0, previewGeneration = 0, previewAbort = null, pollAbort = null, activeJob = null;
  let previewTimer = null, busy = false, playing = false, playbackGeneration = 0;
  let staticView = null, staticResult = null, staticWidths = '', mapLayout = null, chartLayout = null, insetLayout = null;
  let snapshotIndex = -1, snapshotResult = null, observedWidths = '';

  function inputs() {
    const result = {};
    for (const [name, value] of new FormData(form)) {
      result[name] = ['track', 'profile', 'configurations'].includes(name) ? value : name === 'mirrored' ? value === 'true' : Number(value);
    }
    return result;
  }
  function geometryKey(values) { return JSON.stringify([...geometryFields].map(name => [name, values[name]])); }
  function setStatus(message, error = false) { status.textContent = message; status.className = error ? 'status error' : 'status'; }
  function updateLabels() {
    for (const input of form.querySelectorAll('input[type="range"]')) {
      form.querySelector('[data-for="' + input.name + '"]').textContent = fmt(Number(input.value), Number(input.step) < 1 ? 2 : 0) + ' ' + input.dataset.unit;
    }
  }
  function updateNotes() {
    const real = realTracks.has(control('track').value);
    byId('track-preset-note').textContent = real ?
      'Full-size historical circuit. The uniform road width and sample spacing remain adjustable.' :
      'Synthetic test geometry. The road width and sample spacing remain adjustable.';
    const profile = profiles[control('profile').value];
    byId('profile-note').textContent = profile ? (profile.description || profile.label || '') : 'Car values will be supplied by the local simulator.';
  }
  function stop() { playing = false; playbackGeneration++; byId('play').textContent = 'Play lap'; }
  function setBusy(value) {
    busy = value;
    byId('run').disabled = value;
    byId('cancel').hidden = !value;
    byId('calculation-progress').hidden = !value;
  }
  async function jsonRequest(url, options) {
    const response = await fetch(url, options);
    const body = await response.json();
    if (!response.ok) throw new Error(body.error || 'The local simulator could not complete this request.');
    return body;
  }
  const postOptions = (body, signal) => ({method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body), ...(signal ? {signal} : {})});
  function cancelJob(id) {
    if (id) jsonRequest('/api/jobs/' + encodeURIComponent(id) + '/cancel', postOptions({})).catch(() => {});
  }
  function abandonCalculation() {
    runGeneration++;
    pollAbort?.abort();
    cancelJob(activeJob);
    activeJob = null;
    setBusy(false);
    stop();
  }
  function markChanged() {
    abandonCalculation();
    stale = !!data;
    byId('play').disabled = true;
    byId('result-state').textContent = data ? 'Previous result — inputs have changed. Run comparison to update it.' : 'Track preview — run a comparison for lap predictions.';
    setStatus('Inputs changed. Run comparison when ready.');
  }
  function clearProfile() {
    data = null; selected = 0; distance = 0; stale = false;
    byId('comparison-panel').hidden = true;
    byId('strategy').innerHTML = ''; byId('strategy').disabled = true;
    byId('play').disabled = true; byId('seek').disabled = true;
    byId('lap-time').textContent = '—'; byId('delta').textContent = '—'; byId('speed').textContent = '—';
    byId('braking-summary').textContent = 'Braking zones appear after the first profile is calculated.';
    byId('position-detail').textContent = 'Geometry preview; no lap result yet.';
    byId('view-title').textContent = 'Track preview';
  }
  async function requestPreview(values = inputs()) {
    const generation = ++previewGeneration;
    previewAbort?.abort();
    previewAbort = new AbortController();
    const key = geometryKey(values);
    if (!view || geometryKey(view.inputs || {}) !== key) {
      clearProfile();
      byId('result-state').textContent = 'Loading selected layout…';
      // Leave the previous drawing visible until replacement, explicitly labelled.
      if (view) byId('track-caption').textContent = 'Previous layout while the selected preview loads…';
    }
    try {
      const preview = await jsonRequest('/api/preview', postOptions(values, previewAbort.signal));
      if (generation !== previewGeneration || key !== geometryKey(inputs())) return;
      view = preview;
      clearProfile();
      byId('result-panel').hidden = false;
      byId('result-state').textContent = 'Track preview — run a comparison for lap predictions.';
      rebuild();
      if (!busy) setStatus('Track ready. Run comparison to calculate the selected brake systems.');
    } catch (error) {
      if (generation !== previewGeneration || error.name === 'AbortError') return;
      byId('result-state').textContent = 'Preview could not load. Previous geometry may still be visible.';
      setStatus('Track preview: ' + error.message, true);
    }
  }
  function schedulePreview() {
    clearTimeout(previewTimer);
    previewTimer = setTimeout(() => requestPreview(), 120);
  }
  form.addEventListener('input', event => {
    updateLabels(); markChanged();
    if (geometryFields.has(event.target?.name) && !['track','profile','mirrored','spacing'].includes(event.target.name)) schedulePreview();
  });
  form.addEventListener('change', event => {
    const name = event.target.name;
    if (name === 'track') {
      const real = realTracks.has(event.target.value);
      control('width').value = real ? '12' : '3';
      control('spacing').value = real ? '4' : '2';
    } else if (name === 'profile') {
      const defaults = profiles[event.target.value]?.default_inputs || {};
      for (const key of ['front_track','rear_track']) if (defaults[key] !== undefined) control(key).value = String(defaults[key]);
    }
    updateLabels(); updateNotes(); markChanged();
    if (geometryFields.has(name)) {
      clearTimeout(previewTimer);
      requestPreview();
    }
  });
  function pollDelay(signal) {
    return new Promise((resolve, reject) => {
      const abort = () => {clearTimeout(timer); reject(new DOMException('Cancelled', 'AbortError'));};
      const timer = setTimeout(() => {signal.removeEventListener('abort', abort); resolve();}, 450);
      signal.addEventListener('abort', abort, {once:true});
    });
  }
  function acceptResult(result) {
    if (!result.results?.length) return;
    previewGeneration++; previewAbort?.abort();
    data = result; view = result; stale = false;
    selected = Math.min(selected, data.results.length - 1);
    distance = Math.min(distance, data.track.length_m);
    byId('result-panel').hidden = false; byId('comparison-panel').hidden = false;
    byId('view-title').textContent = 'Selected profile';
    byId('strategy').disabled = false; byId('seek').disabled = false; byId('play').disabled = false;
    byId('strategy').innerHTML = data.results.map((result, index) => '<option value="' + index + '">' + escape(result.name) + '</option>').join('');
    byId('strategy').value = String(selected);
    byId('comparison').innerHTML = data.results.map((result, index) => '<tr data-index="' + index + '"><td><button type="button" data-select="' + index + '">' + escape(result.name) + '</button></td><td>' + fmt(result.lap_time_s, 3) + '</td><td>' + (result.delta_to_baseline_s >= 0 ? '+' : '') + fmt(result.delta_to_baseline_s, 3) + '</td></tr>').join('');
    const failures = data.failures || [];
    if (failures.length) byId('comparison').innerHTML += failures.map(failure => '<tr><td>' + escape(failure.name) + '</td><td colspan="2">Unavailable</td></tr>').join('');
    byId('comparison-errors').hidden = failures.length === 0;
    byId('comparison-errors').textContent = failures.map(failure => failure.name + ': ' + failure.error).join(' ');
    rebuild();
  }
  form.addEventListener('submit', async event => {
    event.preventDefault();
    clearTimeout(previewTimer);
    abandonCalculation();
    const generation = runGeneration, values = inputs();
    const neededPreview = !view || geometryKey(view.inputs || {}) !== geometryKey(values);
    if (neededPreview) requestPreview(values);
    stop(); setBusy(true); stale = !!data;
    byId('play').disabled = true;
    byId('result-state').textContent = data ? 'Previous result while the new comparison runs.' : 'Calculating the first profile; the track is ready to inspect.';
    byId('calculation-progress').max = values.configurations === 'all' ? 6 : 2;
    byId('calculation-progress').value = 0;
    setStatus('Starting comparison…');
    let version = -1;
    pollAbort = new AbortController();
    const signal = pollAbort.signal;
    try {
      // Keep the short POST alive so an abandoned request can cancel the job
      // returned by the server, instead of losing its identifier.
      let job = await jsonRequest('/api/jobs', postOptions(values));
      if (generation !== runGeneration) {cancelJob(job.job_id); return;}
      activeJob = job.job_id;
      while (generation === runGeneration) {
        if (job.result) acceptResult(job.result);
        version = job.version ?? version;
        byId('calculation-progress').max = job.total || byId('calculation-progress').max;
        byId('calculation-progress').value = job.completed || 0;
        const elapsed = Number.isFinite(job.elapsed_s) ? ' · ' + fmt(job.elapsed_s, 0) + ' s' : '';
        setStatus((job.phase || 'Calculating') + ' · ' + (job.completed || 0) + '/' + (job.total || '?') + ' profiles' + elapsed);
        byId('result-state').textContent = data && !stale ? 'Available profiles shown; remaining profiles are still calculating.' : 'Track preview — calculation in progress.';
        if (job.status === 'complete') {
          if (!data || stale) throw new Error('The calculation completed without a current profile. Run the comparison again.');
          setStatus('Comparison complete · ' + data.track.distance_m.length + ' segments · ' + values.spacing + ' m requested spacing.');
          byId('result-state').textContent = data.failures?.length ? 'Calculated with the displayed inputs; ' + data.failures.length + ' configurations unavailable (details below).' : 'Calculated with the displayed inputs.';
          break;
        }
        if (job.status === 'failed') throw new Error(job.error || 'Calculation failed.');
        if (job.status === 'cancelled') {
          setStatus('Calculation cancelled. The track remains available.');
          byId('result-state').textContent = data && !stale ? 'Partial comparison — available calculated profiles.' : 'Track preview — calculation cancelled.';
          break;
        }
        await pollDelay(signal);
        job = await jsonRequest('/api/jobs/' + encodeURIComponent(activeJob) + '?after=' + version, {signal});
        if (generation !== runGeneration) return;
      }
    } catch (error) {
      if (generation !== runGeneration || error.name === 'AbortError') return;
      setStatus(error.message + (data ? ' Available profiles remain displayed.' : ' The track preview remains available.'), true);
      byId('result-state').textContent = data && !stale ? 'Available calculated profiles; the full comparison did not finish.' : 'No current lap prediction. Check the inputs and retry.';
    } finally {
      if (generation === runGeneration) {activeJob = null; setBusy(false);}
    }
  });
  byId('cancel').addEventListener('click', () => {
    abandonCalculation();
    setStatus('Calculation cancelled. Available profiles and the track remain visible.');
    byId('result-state').textContent = data && !stale ? 'Partial comparison — available calculated profiles.' : 'Track preview — calculation cancelled.';
  });
  byId('strategy').addEventListener('change', event => {stop(); selected = Number(event.target.value); rebuild();});
  byId('comparison').addEventListener('click', event => {
    const button = event.target.closest('[data-select]');
    if (button) {stop(); selected = Number(button.dataset.select); byId('strategy').value = String(selected); rebuild();}
  });
  byId('seek').addEventListener('input', event => {stop(); if (data) {distance = Number(event.target.value) / 1000 * data.track.length_m; updateSnapshot();}});
  byId('play').addEventListener('click', () => {
    if (!data || stale) return;
    if (playing) {stop(); return;}
    if (distance >= data.track.length_m - .01) distance = 0;
    playing = true; const generation = ++playbackGeneration;
    byId('play').textContent = 'Pause';
    let last = null, lastPaint = -Infinity;
    function frame(timestamp) {
      if (!playing || generation !== playbackGeneration) return;
      const dt = last === null ? 0 : Math.min(.1, (timestamp - last) / 1000); last = timestamp;
      const point = location(), result = data.results[selected];
      const speed = result.speed_mps[point.index] * (1 - point.fraction) + result.speed_mps[point.next] * point.fraction;
      distance = Math.min(data.track.length_m, distance + speed * dt);
      if (timestamp - lastPaint >= 1000 / 30 || distance >= data.track.length_m) {updateSnapshot(); lastPaint = timestamp;}
      if (distance >= data.track.length_m) stop(); else requestAnimationFrame(frame);
    }
    requestAnimationFrame(frame);
  });
  function location() {
    const track = view.track, s = track.distance_m;
    let low = 0, high = s.length;
    while (low < high) {const middle = (low + high) >>> 1; if (s[middle] <= distance) low = middle + 1; else high = middle;}
    const index = Math.max(0, low - 1), next = (index + 1) % s.length;
    const end = next === 0 ? track.length_m : s[next];
    return {index, next, fraction:Math.min(1, Math.max(0, (distance - s[index]) / (end - s[index])))};
  }
  function path(points) {return points.map((p, index) => (index ? 'L' : 'M') + p.map(value => value.toFixed(2)).join(',')).join(' ');}
  function brakingPaths(points, result, strokeWidth) {
    const categories = result?.braking?.segment_category || [], marks = {light:[], medium:[], hard:[]};
    for (let index = 0; index < points.length - 1; index++) if (marks[categories[index]]) marks[categories[index]].push(path([points[index], points[index + 1]]));
    return Object.entries(marks).filter(([, parts]) => parts.length).map(([category, parts]) => '<path data-braking="' + category + '" d="' + parts.join(' ') + '" fill="none" stroke="var(--brake-' + category + ')" stroke-width="' + strokeWidth + '" stroke-linecap="round"/>').join('');
  }
  function widths() {return ['track-scene','car-scene','speed-chart'].map(id => Math.round(byId(id).clientWidth)).join(',');}
  function rebuild() {
    if (!view) return;
    const result = data?.results[selected] || null, size = widths();
    if (view !== staticView || result !== staticResult || size !== staticWidths) {
      buildTrack(result); buildInset(); buildSpeed(result); buildWheelTable();
      staticView = view; staticResult = result; staticWidths = size;
      snapshotIndex = -1; snapshotResult = null;
    }
    const track = view.track, metadata = track.metadata || {}, source = metadata.source_url;
    const vehicle = view.vehicle.wheels.vehicle;
    byId('profile-values').textContent = fmt(vehicle.vehicle_mass_kg, 0) + ' kg total mass · ' + fmt(vehicle.center_of_gravity_to_front_axle_m + vehicle.center_of_gravity_to_rear_axle_m, 3) + ' m wheelbase · ' + fmt(view.vehicle.maximum_speed_mps * 3.6, 0) + ' km/h model speed cap. Values and remaining assumptions are documented in LAP_TIME_MODEL.md.';
    byId('track-caption').textContent = track.name + ' · ' + fmt(track.length_m) + ' m · ' + fmt(track.width_m[0]) + ' m road · car shown to scale';
    byId('track-source').innerHTML = metadata.kind === 'real' ?
      escape(metadata.layout_note || 'Historical approximate circuit centerline.') + ' ' + escape(metadata.width_note || 'Uniform width override.') + ' ' +
      (typeof source === 'string' && /^https?:\/\//i.test(source) ? '<a href="' + escape(source) + '" target="_blank" rel="noopener noreferrer">TUM racetrack database</a>' : 'TUM racetrack database') +
      ' · <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener noreferrer">© OpenStreetMap contributors</a>' : '';
    if (result) {
      byId('lap-time').textContent = fmt(result.lap_time_s, 3) + ' s';
      byId('delta').textContent = (result.delta_to_baseline_s >= 0 ? '+' : '') + fmt(result.delta_to_baseline_s, 3) + ' s';
      const braking = result.braking;
      byId('braking-summary').textContent = braking ? braking.zones.length + ' braking zones · peak ' + fmt(braking.peak_deceleration_g, 2) + ' g · light ' + fmt(braking.distance_m.light, 0) + ' m · medium ' + fmt(braking.distance_m.medium, 0) + ' m · hard ' + fmt(braking.distance_m.hard, 0) + ' m' : 'Braking-zone analysis is unavailable for this saved profile.';
      for (const row of byId('comparison').querySelectorAll('tr')) row.classList.toggle('selected', Number(row.dataset.index) === selected);
    }
    updateSnapshot();
  }
  function buildTrack(result) {
    const svg = byId('track-scene'), track = view.track, width = Math.max(220, svg.clientWidth), height = 340;
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, roadWidth = 0;
    for (let index = 0; index < track.x_m.length; index++) {
      xmin = Math.min(xmin, track.x_m[index]); xmax = Math.max(xmax, track.x_m[index]);
      ymin = Math.min(ymin, track.y_m[index]); ymax = Math.max(ymax, track.y_m[index]); roadWidth = Math.max(roadWidth, track.width_m[index]);
    }
    const padding = roadWidth / 2 + 3; xmin -= padding; xmax += padding; ymin -= padding; ymax += padding;
    const scale = Math.min((width - 20) / (xmax - xmin), (height - 20) / (ymax - ymin));
    const ox = (width - (xmax - xmin) * scale) / 2, oy = (height - (ymax - ymin) * scale) / 2;
    const project = (x, y) => [ox + (x - xmin) * scale, height - oy - (y - ymin) * scale];
    mapLayout = {project, scale};
    const points = track.x_m.map((x, index) => project(x, track.y_m[index])); points.push(points[0]);
    const vehicle = view.vehicle.wheels.vehicle;
    const front = vehicle.center_of_gravity_to_front_axle_m + view.vehicle.front_overhang_m;
    const rear = -vehicle.center_of_gravity_to_rear_axle_m - view.vehicle.rear_overhang_m;
    const tires = [];
    for (let index = 0; index < 4; index++) {
      const wheelX = index < 2 ? vehicle.center_of_gravity_to_front_axle_m : -vehicle.center_of_gravity_to_rear_axle_m;
      const wheelY = (index % 2 === 0 ? -1 : 1) * (index < 2 ? view.vehicle.wheels.front_track_m : view.vehicle.wheels.rear_track_m) / 2;
      tires.push('<rect id="map-wheel-' + index + '" x="' + (wheelX - view.vehicle.tire_plan_length_m / 2) + '" y="' + (wheelY - view.vehicle.tire_plan_width_m / 2) + '" width="' + view.vehicle.tire_plan_length_m + '" height="' + view.vehicle.tire_plan_width_m + '" fill="var(--ink)"/>');
    }
    svg.setAttribute('viewBox', '0 0 ' + width + ' ' + height);
    svg.innerHTML = '<title>Car and calculated braking intensity on ' + escape(track.name) + '</title><path d="' + path(points) + '" fill="none" stroke="var(--road)" stroke-width="' + Math.max(2, track.width_m[0] * scale) + '" stroke-linejoin="round"/><path d="' + path(points) + '" fill="none" stroke="var(--muted)" stroke-width="1" stroke-dasharray="5 6"/>' + brakingPaths(points, result, 3) +
      '<g id="map-car"><rect x="' + rear + '" y="' + (-view.vehicle.body_width_m / 2) + '" width="' + (front - rear) + '" height="' + view.vehicle.body_width_m + '" fill="var(--blue)"/><path d="M' + front + ',0 L' + (front - .3) + ',-.3 L' + (front - .3) + ',.3 Z" fill="var(--ink)"/>' + tires.join('') + '</g><circle id="map-location" r="12" fill="none" stroke="var(--blue)" stroke-width="1"/>';
  }
  function buildInset() {
    const svg = byId('car-scene'), width = Math.max(230, svg.clientWidth), height = 340, scale = 55, cx = width / 2, cy = 155;
    const params = view.vehicle.wheels, vehicle = params.vehicle;
    insetLayout = {wheels:[]};
    const parts = ['<title>Four-wheel forces at the selected segment midpoint</title><defs>'];
    for (const [name, color] of [['brake','orange'],['drive','green'],['lateral','blue']]) parts.push('<marker id="force-' + name + '" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6 Z" fill="var(--' + color + ')"/></marker>');
    parts.push('</defs><path d="M' + (cx - 10) + ',' + (cy - (vehicle.center_of_gravity_to_front_axle_m + .4) * scale) + ' H' + (cx + 10) + ' L' + (cx + 20) + ',' + cy + ' V' + (cy + (vehicle.center_of_gravity_to_rear_axle_m + .2) * scale) + ' H' + (cx - 20) + ' V' + cy + ' Z" fill="var(--road)" stroke="var(--ink)"/><ellipse cx="' + cx + '" cy="' + cy + '" rx="12" ry="21" fill="var(--surface)" stroke="var(--ink)"/>');
    for (let index = 0; index < 4; index++) {
      const front = index < 2, side = index % 2 === 0 ? 1 : -1;
      const x = cx - side * (front ? params.front_track_m : params.rear_track_m) / 2 * scale;
      const y = cy - (front ? vehicle.center_of_gravity_to_front_axle_m : -vehicle.center_of_gravity_to_rear_axle_m) * scale;
      insetLayout.wheels.push({x, y});
      parts.push('<path d="M' + cx + ',' + y + ' H' + x + '" stroke="var(--ink)"/><rect id="inset-wheel-' + index + '" x="' + (x - 6) + '" y="' + (y - 13) + '" width="12" height="26" rx="2" fill="var(--ink)"/><text x="' + (x - side * 18) + '" y="' + (y - 18) + '" text-anchor="middle">' + ['FL','FR','RL','RR'][index] + '</text><path id="force-x-' + index + '" stroke-width="2" fill="none"/><path id="force-y-' + index + '" stroke-width="2" fill="none"/>');
    }
    svg.setAttribute('viewBox', '0 0 ' + width + ' ' + height); svg.innerHTML = parts.join('');
  }
  function buildWheelTable() {
    byId('wheel-details').innerHTML = ['FL','FR','RL','RR'].map((name, index) => '<tr><th>' + name + '</th>' + ['load','fx','fy','aux','grip'].map(quantity => '<td id="wheel-' + quantity + '-' + index + '">—</td>').join('') + '</tr>').join('');
  }
  function buildSpeed(result) {
    chartLayout = null;
    if (!result) return;
    const svg = byId('speed-chart'), width = Math.max(220, svg.clientWidth), height = 205, left = 45, right = 15, top = 22, bottom = 38;
    let ymax = 10;
    for (const profile of data.results) for (const speed of profile.speed_mps) ymax = Math.max(ymax, Math.ceil(speed * 3.6 / 10) * 10);
    const x = value => left + value / view.track.length_m * (width - left - right);
    const y = value => height - bottom - value / ymax * (height - top - bottom);
    chartLayout = {x, top, bottom:height - bottom};
    const points = view.track.distance_m.map((distance, index) => [x(distance), y(result.speed_mps[index] * 3.6)]); points.push([x(view.track.length_m), y(result.speed_mps[0] * 3.6)]);
    const parts = ['<title>Selected speed profile versus distance</title><path d="M' + left + ',' + top + ' V' + (height - bottom) + ' H' + (width - right) + '" stroke="var(--line)" fill="none"/>'];
    for (let i = 0; i <= 4; i++) {const distance = view.track.length_m * i / 4; parts.push('<text x="' + x(distance) + '" y="' + (height - bottom + 17) + '" text-anchor="' + (i === 0 ? 'start' : i === 4 ? 'end' : 'middle') + '">' + fmt(distance, 0) + '</text>');}
    for (let speed = 0; speed <= ymax; speed += 20) parts.push('<text x="' + (left - 7) + '" y="' + (y(speed) + 4) + '" text-anchor="end">' + speed + '</text>');
    parts.push('<text x="' + left + '" y="14">km/h</text><text x="' + (width - right) + '" y="' + (height - 3) + '" text-anchor="end">Distance (m)</text><path d="' + path(points) + '" fill="none" stroke="var(--muted)" stroke-width="2"/>' + brakingPaths(points, result, 3) + '<path id="speed-cursor" stroke="var(--orange)" stroke-width="1"/><rect x="' + left + '" y="' + top + '" width="' + (width - left - right) + '" height="' + (height - top - bottom) + '" fill="transparent" data-hit="profile"/>');
    svg.setAttribute('viewBox', '0 0 ' + width + ' ' + height); svg.innerHTML = parts.join('');
    svg.onclick = event => {stop(); const rect = svg.getBoundingClientRect(); distance = Math.max(0, Math.min(view.track.length_m, (event.clientX - rect.left - left) / (width - left - right) * view.track.length_m)); updateSnapshot();};
  }
  function updateSnapshot() {
    if (!view || !mapLayout) return;
    const point = location(), track = view.track, result = data?.results[selected] || null;
    const x = track.x_m[point.index] * (1 - point.fraction) + track.x_m[point.next] * point.fraction;
    const y = track.y_m[point.index] * (1 - point.fraction) + track.y_m[point.next] * point.fraction;
    const deltaHeading = Math.atan2(Math.sin(track.heading_rad[point.next] - track.heading_rad[point.index]), Math.cos(track.heading_rad[point.next] - track.heading_rad[point.index]));
    const heading = track.heading_rad[point.index] + deltaHeading * point.fraction - (result?.sideslip_angle_rad[point.index] || 0);
    const car = mapLayout.project(x, y);
    byId('map-car').setAttribute('transform', 'translate(' + car + ') rotate(' + (-heading * 180 / Math.PI) + ') scale(' + mapLayout.scale + ')');
    byId('map-location').setAttribute('cx', car[0]); byId('map-location').setAttribute('cy', car[1]);
    byId('seek').value = String(distance / track.length_m * 1000);
    if (chartLayout) byId('speed-cursor').setAttribute('d', 'M' + chartLayout.x(distance) + ',' + chartLayout.top + ' V' + chartLayout.bottom);
    if (result) {
      const speed = result.speed_mps[point.index] * (1 - point.fraction) + result.speed_mps[point.next] * point.fraction;
      byId('speed').textContent = fmt(speed * 3.6) + ' km/h';
      const category = result.braking?.segment_category[point.index];
      byId('position-detail').textContent = fmt(distance) + ' / ' + fmt(track.length_m) + ' m · ' + fmt(result.acceleration_mps2[point.index] / 9.81, 2) + ' g longitudinal' + (category ? ' · ' + (brakingLabel[category] || 'Unclassified') : '') + ' · ' + fmt(result.steering_angle_rad[point.index] * 180 / Math.PI, 1) + '° road-wheel steering · profile playback';
    }
    if (snapshotIndex === point.index && snapshotResult === result) return;
    snapshotIndex = point.index; snapshotResult = result;
    const steering = result?.steering_angle_rad[point.index] || 0, params = view.vehicle.wheels;
    for (let index = 0; index < 4; index++) {
      const {x, y} = insetLayout.wheels[index], angle = index < 2 ? steering : 0;
      byId('inset-wheel-' + index).setAttribute('transform', 'rotate(' + (-angle * 180 / Math.PI) + ',' + x + ',' + y + ')');
      const wheelX = index < 2 ? params.vehicle.center_of_gravity_to_front_axle_m : -params.vehicle.center_of_gravity_to_rear_axle_m;
      const wheelY = (index % 2 === 0 ? -1 : 1) * (index < 2 ? params.front_track_m : params.rear_track_m) / 2;
      byId('map-wheel-' + index).setAttribute('transform', 'rotate(' + (-angle * 180 / Math.PI) + ',' + wheelX + ',' + wheelY + ')');
      const fx = result?.tire_longitudinal_forces_n[point.index][index] || 0, fy = result?.tire_lateral_forces_n[point.index][index] || 0;
      const auxiliary = result?.auxiliary_braking_forces_n?.[point.index]?.[index] || 0;
      for (const prefix of ['inset-wheel-', 'map-wheel-']) {
        byId(prefix + index).setAttribute('stroke', auxiliary > .01 ? 'var(--orange)' : 'none');
        byId(prefix + index).setAttribute('stroke-width', prefix === 'map-wheel-' ? '.05' : '3');
      }
      for (const [force, lateral] of [[fx, false],[fy, true]]) {
        const node = byId('force-' + (lateral ? 'y-' : 'x-') + index), length = Math.min(95, Math.abs(force) * .045);
        const kind = lateral ? 'lateral' : force < 0 ? 'brake' : 'drive', color = lateral ? 'blue' : force < 0 ? 'orange' : 'green';
        const endX = x - Math.sign(force) * length * (lateral ? Math.cos(angle) : Math.sin(angle));
        const endY = y + Math.sign(force) * length * (lateral ? Math.sin(angle) : -Math.cos(angle));
        node.setAttribute('d', Math.abs(force) < 1 ? '' : 'M' + x + ',' + y + ' L' + endX + ',' + endY);
        node.setAttribute('stroke', 'var(--' + color + ')'); node.setAttribute('marker-end', 'url(#force-' + kind + ')');
      }
      if (result) {
        const load = result.wheel_loads_n[point.index][index], mu = params.vehicle.friction_coefficient;
        byId('wheel-load-' + index).textContent = fmt(load, 0); byId('wheel-fx-' + index).textContent = fmt(fx, 0); byId('wheel-fy-' + index).textContent = fmt(fy, 0);
        byId('wheel-grip-' + index).textContent = fmt(load > 0 ? Math.hypot(fx, fy) / (mu * load) * 100 : 0, 0) + '%';
        byId('wheel-aux-' + index).textContent = fmt(auxiliary, 1);
      }
    }
  }
  new ResizeObserver(() => {
    const size = widths();
    if (size === observedWidths) return;
    observedWidths = size;
    if (view) rebuild();
  }).observe(byId('result-panel'));
  async function initialize() {
    updateLabels(); updateNotes();
    try {
      const config = await jsonRequest('/api/config');
      profiles = Array.isArray(config.profiles) ? Object.fromEntries(config.profiles.map(profile => [profile.id, profile])) : config.profiles || {};
      if (Object.keys(profiles).length) control('profile').innerHTML = Object.entries(profiles).map(([id, profile]) => '<option value="' + escape(id) + '">' + escape(profile.label || id) + '</option>').join('');
      for (const [name, value] of Object.entries(config.default_inputs || {})) {const input = control(name); if (input) input.value = String(value);}
      updateLabels(); updateNotes(); setBusy(false);
      await requestPreview();
    } catch (error) {
      setStatus('Could not connect to the updated simulator: ' + error.message + ' Restart simulator.py and refresh this page.', true);
    }
  }
  initialize();
})();
