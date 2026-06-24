/* ============================================================================
   ros-bindings.js — turns the SCUTTLE operator console (index.html) from a
   simulation into a LIVE ROS 2 client.

   Design contract (see DATA-CONTRACT.md): the render layer (draw(), updateHUD(),
   renderNodes(), renderWaypoints()) and the State object `S` are reused verbatim.
   This file only changes the DATA SOURCE: sim generators → ROS 2 topics.

   How it engages:
     - On load it tries to reach rosbridge at ws://localhost:9090 (the laptop, which
       is zenoh-sourced so it also sees the Pi's topics over the link).
     - If reachable  → S.live = true, the simulation is halted (S.paused = true so
       step() and the 1 Hz health tick no-op), and every panel is fed from ROS.
     - If NOT reachable (or roslib isn't vendored) → the page silently stays in SIM
       mode so it still opens for design review. A small badge shows which mode.

   Single-robot deployment. Topic/type/rate reference: DATA-CONTRACT.md.
   ========================================================================== */
(function () {
  'use strict';

  const URL = (new URLSearchParams(location.search)).get('bridge') || 'ws://localhost:9090';
  const badge = makeBadge();

  if (window.__noRoslib || typeof ROSLIB === 'undefined') {
    badge('SIM', 'roslib not vendored — see web/vendor/README.md');
    return;                                   // stay in simulation
  }

  const ros = new ROSLIB.Ros({ url: URL });
  let bootDriven = false;

  ros.on('error',  () => { if (!S.live) badge('SIM', 'rosbridge unreachable @ ' + URL); });
  ros.on('close',  () => { if (S.live) injectLinkLoss(); });   // reuse the frozen-link flow
  ros.on('connection', onBridgeUp);

  /* ─────────────────────────── boot / discovery ─────────────────────────── */
  async function onBridgeUp() {
    badge('LIVE', URL);
    goLive();
    setBoot('15%', '● websocket up · discovering ROS 2 graph…');
    let nodes = [];
    try { nodes = await new Promise(r => ros.getNodes(r)); } catch (_) {}
    const have = n => nodes.some(x => x.includes(n));
    const core = ['slam_toolbox'];                 // base mode: SLAM only; Nav2/explore arrive on frontier
    if (nodes.length && !core.every(have)) {
      setBoot('45%', '⚠ base stack incomplete · waiting on SLAM…');
    } else {
      setBoot('70%', '● core stack alive · waiting on /gdm/gas_1 + /odom…');
    }
    subscribeAll();
  }

  // Flip the app into live mode and freeze the simulation engine.
  function goLive() {
    if (S.live) return;
    S.live = true;
    S.paused = true;                          // halts step() motion/gas AND the 1 Hz sim tick
    S.traj = [];                              // drop sim trajectory; real one rebuilds from /odom
    const sim = document.getElementById('simbar'); if (sim) sim.style.display = 'none';
    clearFog();                               // occupancy comes from /map now, not the fog reveal
  }

  function setBoot(w, msg) {
    if (bootDriven === false) { bootDriven = true; window.bootT && window.bootT.forEach(clearTimeout); }
    const bar = document.getElementById('bootBar'), st = document.getElementById('bootStatus');
    if (bar) bar.style.width = w; if (st) st.textContent = msg;
  }
  function enableEnter() {
    const b = document.getElementById('enterBtn');
    if (b && b.disabled) {
      b.disabled = false;
      document.getElementById('bootPi').className = 'nbox ok';
      document.getElementById('bootFeather').className = 'nbox ok';
      setBoot('100%', '✓ live · /odom + /gdm/gas_1 flowing — cockpit unlocked');
      document.getElementById('bootStatus').style.color = 'var(--color-safe)';
    }
  }

  /* ─────────────────────────── subscriptions ─────────────────────────── */
  let gasSeen = false, odomSeen = false;
  function bothSeen() { if (gasSeen && odomSeen) enableEnter(); }

  function subscribeAll() {
    /* Gas — /gdm/gas_1 (olfaction_msgs/GasSensor). PLACEHOLDER source: fake_gas_sensor.
       Value range 0–110 (GMRF max_sensor_val). Thresholds caution 60 / hazard 90 (locked
       placeholders — re-tune to the real sensor's units/LEL when the module lands). */
    topic('/gdm/gas_1', 'olfaction_msgs/msg/GasSensor').subscribe(m => {
      S.ch4 = (m.raw != null ? m.raw : m.concentration);     // GasSensor.raw (raw_units = ppm/volt/ohm)
      S.ch4Hist.push(S.ch4); if (S.ch4Hist.length > 60) S.ch4Hist.shift();
      if (S.ch4 > 90 && !S.alarm) fireHazard();
      if (S.alarm && S.alarm.type === 'gas' && S.alarm.acked) {
        if (S.ch4 < 60 && !S.alarm.safe) markSafe();
        if (S.ch4 >= 60 && S.alarm.safe) markUnsafe();
      }
      gasSeen = true; bothSeen(); updateHUD();
    });

    /* Pose — composed map→base_link from /tf (correct frame for a map-aligned marker).
       A bare /odom pose is in the odom frame and drifts vs the map; we compose through TF. */
    topic('/tf', 'tf2_msgs/msg/TFMessage').subscribe(m => ingestTF(m, false));
    topic('/tf_static', 'tf2_msgs/msg/TFMessage', { throttle_rate: 0 }).subscribe(m => ingestTF(m, true));
    // /odom still drives velocity readout + the "encoder /odom OK" health line.
    topic('/odom', 'nav_msgs/msg/Odometry', { throttle_rate: 100 }).subscribe(m => {
      S.vel.v = m.twist.twist.linear.x;
      S.vel.w = m.twist.twist.angular.z;
      odomSeen = true; bothSeen();
    });

    /* Occupancy grid — /map → paints `base`, replaces the fog reveal. */
    topic('/map', 'nav_msgs/msg/OccupancyGrid', { throttle_rate: 1000 }).subscribe(paintOccupancy);

    /* Lidar — /scan → S.scan; draw() transforms hits map→canvas via paintScan(). */
    topic('/scan', 'sensor_msgs/msg/LaserScan', { throttle_rate: 100 }).subscribe(m => { S.scan = m; });

    /* GMRF gas grid — render as the heatmap. NOTE: confirm gmrf_node's output topic name;
       it publishes an OccupancyGrid-like mean field. Update the name below when verified. */
    topic('/gdm/gmrf_gas_map', 'nav_msgs/msg/OccupancyGrid', { throttle_rate: 1000 })
      .subscribe(g => { S.gasGrid = g; });

    /* Diagnostics — /diagnostics → System / Network / Nodes drawer. */
    topic('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray').subscribe(arr => {
      for (const st of arr.status) {
        const kv = Object.fromEntries(st.values.map(v => [v.key, v.value]));
        const n = st.name.toLowerCase();
        if (n.includes('cpu'))     { if (kv.load != null) S.cpu = +kv.load; if (kv.temp_c != null) S.temp = +kv.temp_c; }
        if (n.includes('battery')) { if (kv.voltage != null) S.volt = +kv.voltage; if (kv.percentage != null) S.battery = +kv.percentage / 100; }
        if (n.includes('motor'))   { if (kv.current_left != null) S.motorL = +kv.current_left; if (kv.current_right != null) S.motorR = +kv.current_right; }
        if (n.includes('wifi') || n.includes('link')) { if (kv.rssi != null) S.rssi = +kv.rssi; if (kv.rtt_ms != null) S.latency = +kv.rtt_ms; }
      }
      S.latHist.push(S.latency); if (S.latHist.length > 40) S.latHist.shift();
      updateHUD(); renderNodes();
    });
  }

  function topic(name, messageType, extra) {
    return new ROSLIB.Topic(Object.assign({ ros, name, messageType }, extra || {}));
  }

  /* ─────────────────────── publishers / commands ─────────────────────── */

  // Teleop → /cmd_vel_teleop (twist_mux pri 100), clamped to SCUTTLE limits, only when live+manual.
  const cmdVel = topic('/cmd_vel_teleop', 'geometry_msgs/msg/Twist');
  setInterval(() => {
    if (!S.live || S.frozen || S.estop || S.mode !== 'manual') return;
    cmdVel.publish(new ROSLIB.Message({
      linear:  { x: clamp(S.manual.v, -0.5, 0.5), y: 0, z: 0 },   // LIN_MAX 0.5 m/s
      angular: { x: 0, y: 0, z: clamp(S.manual.w, -1.0, 1.0) }    // ANG_MAX 1.0 rad/s
    }));
  }, 50);

  // E-STOP → /estop (std_msgs/Bool, latched, twist_mux lock pri 255). Wrap the UI's fireEstop so
  // the button + Space×2 + Ctrl+. all publish; subscribe back so a watchdog-asserted stop shows.
  const estopPub = new ROSLIB.Topic({ ros, name: '/estop', messageType: 'std_msgs/msg/Bool', latch: true });
  const _fire = window.fireEstop;
  window.fireEstop = function () { _fire(); if (S.live) estopPub.publish(new ROSLIB.Message({ data: !!S.estop })); };
  topic('/estop', 'std_msgs/msg/Bool').subscribe(m => {
    if (m.data && !S.estop) { S.estop = true; toast('/estop asserted (watchdog or operator)'); updateHUD(); }
    if (!m.data && S.estop) { S.estop = false; updateHUD(); }
  });

  // ---- Mission control: mode toggle drives the supervisors on both machines. ----
  const latched = { latch: true };
  const modePub = topic('/mission/mode', 'std_msgs/msg/String', latched);
  const savePathPub = topic('/mission/save_path', 'std_msgs/msg/String', latched);

  // Wrap the existing client-side setMode so the toggle also publishes the real mode.
  const _setMode = window.setMode;
  window.setMode = function (m) {
    _setMode(m);
    if (S.live) modePub.publish(new ROSLIB.Message({ data: m === 'manual' ? 'manual' : 'frontier' }));
  };

  // Per-machine state strings -> merge for the autonomy chip.
  const missionState = { pi: '', laptop: '' };
  function showMissionState() {
    const txt = `pi:${missionState.pi || '?'} · laptop:${missionState.laptop || '?'}`;
    const el = document.getElementById('sacState');
    if (el) el.title = txt;
  }
  topic('/mission/state/pi', 'std_msgs/msg/String', { throttle_rate: 0 })
    .subscribe(m => { missionState.pi = m.data; showMissionState(); });
  topic('/mission/state/laptop', 'std_msgs/msg/String', { throttle_rate: 0 })
    .subscribe(m => { missionState.laptop = m.data; showMissionState(); });

  // Save maps: publish the chosen path, then call the Trigger; toast the returned folder.
  const saveSrv = new ROSLIB.Service({
    ros, name: '/mission/save_maps', serviceType: 'std_srvs/srv/Trigger'
  });
  window.saveMaps = function (path) {
    if (!S.live) { toast('Save needs a live connection'); return; }
    if (path) savePathPub.publish(new ROSLIB.Message({ data: path }));
    setTimeout(() => saveSrv.callService(new ROSLIB.ServiceRequest({}), res => {
      toast(res.success ? ('Maps saved → ' + res.message) : ('Save failed: ' + res.message));
    }, err => toast('Save error: ' + err)), 200);
  };

  // Heatmap layer is client-side only; gmrf_node keeps running regardless.
  window.toggleHeatmap = function (on) { S.showHeatmap = !!on; };

  // Goals → navigate_to_pose (the action server frontier also uses). Wrap the map click:
  // the UI already pushes a canvas-space waypoint + renders it; we additionally send the goal.
  const navClient = new ROSLIB.ActionClient({ ros, serverName: '/navigate_to_pose', actionName: 'nav2_msgs/action/NavigateToPose' });
  const cv = document.getElementById('mapCanvas');
  cv.addEventListener('click', e => {
    if (!S.live || S.mode === 'manual') return;
    const r = cv.getBoundingClientRect();
    const cx = (e.clientX - r.left) / r.width * 960, cy = (e.clientY - r.top) / r.height * 600;
    const w = canvasToWorld(cx, cy); if (!w) return;
    new ROSLIB.Goal({ actionClient: navClient, goalMessage: {
      pose: { header: { frame_id: 'map' }, pose: {
        position: { x: w.x, y: w.y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } } }
    }).send();
  });

  /* ─────────────────────── map ↔ canvas transform ─────────────────────── */
  // Captured from the live /map OccupancyGrid (origin + resolution). Fits the grid into the
  // 960×600 canvas with a margin and flips Y (ROS +y up → canvas +y down).
  let MX = null;   // {ox,oy,res,scale,offX,offY,pxH}
  function paintOccupancy(grid) {
    const info = grid.info, gw = info.width, gh = info.height, res = info.resolution;
    const ox = info.origin.position.x, oy = info.origin.position.y;
    const scale = Math.min(960 / (gw * res), 600 / (gh * res)) * 0.96;
    const pxW = gw * res * scale, pxH = gh * res * scale;
    MX = { ox, oy, res, scale, offX: (960 - pxW) / 2, offY: (600 - pxH) / 2, pxH };

    // Render the grid into a gw×gh offscreen, then blit scaled (with Y flip) into `base`.
    const tmp = document.createElement('canvas'); tmp.width = gw; tmp.height = gh;
    const tctx = tmp.getContext('2d'); const img = tctx.createImageData(gw, gh);
    const free = hexRGB(css('--map-free')), occ = hexRGB(css('--map-occupied')), unk = hexRGB(css('--map-unknown'));
    for (let i = 0; i < grid.data.length; i++) {
      const v = grid.data[i], c = v < 0 ? unk : v >= 65 ? occ : free, j = i * 4;
      img.data[j] = c[0]; img.data[j + 1] = c[1]; img.data[j + 2] = c[2]; img.data[j + 3] = 255;
    }
    tctx.putImageData(img, 0, 0);
    bctx.fillStyle = css('--map-bg'); bctx.fillRect(0, 0, 960, 600);
    bctx.save();
    bctx.translate(MX.offX, MX.offY + pxH); bctx.scale(1, -1);   // flip Y so row 0 = bottom
    bctx.imageSmoothingEnabled = false;
    bctx.drawImage(tmp, 0, 0, pxW, pxH);
    bctx.restore();
  }
  window.worldToCanvas = (x, y) => MX ? { x: MX.offX + (x - MX.ox) * MX.scale, y: MX.offY + MX.pxH - (y - MX.oy) * MX.scale } : null;
  window.canvasToWorld = (cx, cy) => MX ? { x: MX.ox + (cx - MX.offX) / MX.scale, y: MX.oy + (MX.offY + MX.pxH - cy) / MX.scale } : null;

  /* ─────────────────────── TF compositor (map ↔ base_link) ─────────────────────── */
  const TF = new Map();   // "parent>child" → {x,y,yaw}
  function ingestTF(msg, isStatic) {
    for (const t of msg.transforms) {
      const tr = t.transform.translation, q = t.transform.rotation;
      TF.set(t.header.frame_id + '>' + t.child_frame_id,
        { x: tr.x, y: tr.y, yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) });
    }
    if (!isStatic) refreshPoseFromTF();
  }
  // BFS over TF edges (and their inverses) to compose `from`→`to`.
  function compose(from, to) {
    if (from === to) return { x: 0, y: 0, yaw: 0 };
    const seen = new Set([from]), q = [[from, { x: 0, y: 0, yaw: 0 }]];
    while (q.length) {
      const [node, acc] = q.shift();
      for (const key of TF.keys()) {
        const [p, c] = key.split('>'); let nxt = null, e = TF.get(key);
        if (p === node && !seen.has(c)) nxt = [c, apply(acc, e)];
        else if (c === node && !seen.has(p)) nxt = [p, apply(acc, inv(e))];
        if (nxt) { if (nxt[0] === to) return nxt[1]; seen.add(nxt[0]); q.push(nxt); }
      }
    }
    return null;
  }
  const apply = (a, b) => ({                  // a ∘ b  (apply b in a's frame)
    x: a.x + Math.cos(a.yaw) * b.x - Math.sin(a.yaw) * b.y,
    y: a.y + Math.sin(a.yaw) * b.x + Math.cos(a.yaw) * b.y,
    yaw: a.yaw + b.yaw
  });
  const inv = e => ({ x: -(Math.cos(-e.yaw) * e.x - Math.sin(-e.yaw) * e.y), y: -(Math.sin(-e.yaw) * e.x + Math.cos(-e.yaw) * e.y), yaw: -e.yaw });

  const BASE = ['base_link', 'base_footprint'];
  function refreshPoseFromTF() {
    if (!MX) return;
    let m = null;
    for (const b of BASE) { m = compose('map', b); if (m) break; }
    if (!m) return;
    const c = worldToCanvas(m.x, m.y); if (!c) return;
    S.pose.x = c.x; S.pose.y = c.y; S.pose.th = -m.yaw;        // canvas Y is flipped → negate yaw
    S.poseWorld = m;
    if (!S.traj.length || Math.hypot(c.x - S.traj[S.traj.length - 1].x, c.y - S.traj[S.traj.length - 1].y) > 4) {
      S.traj.push({ x: c.x, y: c.y }); if (S.traj.length > 600) S.traj.shift();
    }
  }

  /* ─────────────────────── live render helpers (called from draw()) ─────────────────────── */
  // Real /scan: transform each hit into the map frame via map→<scan frame>, then to canvas.
  window.paintScan = function (ctx, scan) {
    if (!MX || !scan) return;
    const frame = (scan.header && scan.header.frame_id) || 'base_link';
    const T = compose('map', frame) || S.poseWorld; if (!T) return;
    ctx.fillStyle = 'rgba(88,166,255,.55)';
    const n = scan.ranges.length, stepN = Math.max(1, Math.floor(n / 240));
    for (let i = 0; i < n; i += stepN) {
      const r = scan.ranges[i];
      if (!isFinite(r) || r < scan.range_min || r > scan.range_max) continue;
      const a = scan.angle_min + i * scan.angle_increment;
      const lx = r * Math.cos(a), ly = r * Math.sin(a);
      const wx = T.x + Math.cos(T.yaw) * lx - Math.sin(T.yaw) * ly;
      const wy = T.y + Math.sin(T.yaw) * lx + Math.cos(T.yaw) * ly;
      const c = worldToCanvas(wx, wy); if (!c) continue;
      ctx.fillRect(c.x - 1, c.y - 1, 2, 2);
    }
  };
  // Real heatmap: GMRF mean grid (OccupancyGrid-like) painted cell-by-cell, inferno-ish ramp.
  window.paintGasGrid = function (ctx, grid) {
    if (S.showHeatmap === false) return;          // operator hid the layer
    if (!MX || !grid || !grid.info) return;
    const info = grid.info, gw = info.width, gh = info.height, res = info.resolution;
    const ox = info.origin.position.x, oy = info.origin.position.y, cell = res * MX.scale;
    for (let gy = 0; gy < gh; gy++) for (let gx = 0; gx < gw; gx++) {
      const v = grid.data[gy * gw + gx]; if (v < 0) continue;        // unknown
      const t = Math.min(1, v / 100);                                 // 0..100 (GMRF normalised to 0–100)
      if (t < 0.04) continue;
      const c = worldToCanvas(ox + (gx + 0.5) * res, oy + (gy + 0.5) * res); if (!c) continue;
      ctx.fillStyle = inferno(t, 0.15 + t * 0.55);
      ctx.fillRect(c.x - cell / 2, c.y - cell / 2, cell + 1, cell + 1);
    }
  };

  /* ─────────────────────────── utilities ─────────────────────────── */
  function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }
  function css(v) { return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }
  function clearFog() { try { fctx.clearRect(0, 0, 960, 600); } catch (_) {} }   // fog → transparent so /map shows
  function hexRGB(h) {
    h = h.replace('#', ''); if (h.length === 3) h = h.split('').map(c => c + c).join('');
    return [parseInt(h.slice(0, 2), 16) || 20, parseInt(h.slice(2, 4), 16) || 24, parseInt(h.slice(4, 6), 16) || 30];
  }
  function inferno(t, a) {   // cheap perceptual-ish gas ramp: purple → orange → yellow
    const r = Math.round(255 * Math.min(1, t * 1.6)), g = Math.round(255 * Math.max(0, t * t)), b = Math.round(120 * (1 - t) + 20);
    return `rgba(${r},${g},${b},${a})`;
  }
  function makeBadge() {
    const el = document.createElement('div');
    el.style.cssText = 'position:fixed;top:8px;right:10px;z-index:400;font:600 11px/1 IBM Plex Mono,monospace;' +
      'padding:5px 9px;border-radius:6px;letter-spacing:.06em;border:1px solid;background:rgba(0,0,0,.6)';
    document.body.appendChild(el);
    return (mode, detail) => {
      const live = mode === 'LIVE';
      el.style.color = live ? 'var(--color-safe)' : 'var(--color-caution)';
      el.style.borderColor = live ? 'rgba(63,185,80,.5)' : 'rgba(210,153,34,.5)';
      el.textContent = (live ? '● LIVE ROS 2 · ' : '○ SIM · ') + (detail || '');
      el.title = detail || '';
    };
  }
})();
