/*
 * sim.js — dohyo simulation + canvas UI for the TAXI strategy designer.
 *
 * Classic script (no ES modules). Exposes a single global: window.SimUI
 *
 *   SimUI.init({ canvas, hud, readout, controls })  // HTMLElements
 *   SimUI.render()   // recompute the simulation from TAXI.state and redraw
 *   SimUI.play()     // start the animation
 *   SimUI.pause()    // stop the animation
 *   SimUI.reset()    // rewind to t = 0
 *
 * Reads window.TAXI.state (see state.js) and uses TAXI.simulate / corners /
 * robotOut / straightTimeToEdge. Guards against window.TAXI being absent.
 *
 * Coordinate convention (matches state.js): world +x = right, +y = up,
 * heading 0 = up, heading increases clockwise. Canvas y is flipped on draw.
 */
(function () {
  "use strict";

  var TAU = Math.PI * 2;
  var DEG = Math.PI / 180;

  /* Distinct colour per step; golden-angle fallback beyond the palette. */
  var PALETTE = [
    "#4da3ff", "#3ddc84", "#ffb020", "#ff5d73",
    "#b07cff", "#00d4d4", "#ff8a3d", "#9ad34f",
  ];
  function stepColor(i) {
    if (i < PALETTE.length) return PALETTE[i];
    return "hsl(" + ((i * 137.508) % 360).toFixed(0) + " 70% 58%)";
  }

  /* ------------------------------------------------------------- module state */
  var els = { canvas: null, hud: null, readout: null, controls: null };
  var ctx = null;
  var TAXI = null;              // captured window.TAXI (may be null)
  var view = null;              // last computed view transform
  var sim = null;               // last TAXI.simulate() result
  var stepPaths = [];           // [{ step, pts:[{x,y}], endTh }]
  var timelineMark = null;      // DOM marker inside the timeline bar
  var anim = { playing: false, t: 0, speed: 1, last: 0 };
  var probeHeading = 0;         // deg
  var drag = null;              // { mode:'move'|'rotate', offX, offY }
  var dirty = true;
  var raf = 0;
  var btnPlay = null;

  /* ------------------------------------------------------------------ helpers */
  function snap(v, step) { return Math.round(v / step) * step; }
  function normDeg(d) { return ((d % 360) + 360) % 360; }

  function cssVar(name, fallback) {
    var v = getComputedStyle(document.documentElement).getPropertyValue(name);
    v = (v || "").trim();
    return v || fallback;
  }
  function theme() {
    return {
      panel:  cssVar("--panel",  "#1b1e24"),
      panel2: cssVar("--panel2", "#23272f"),
      line:   cssVar("--line",   "#3a3f4a"),
      txt:    cssVar("--txt",    "#e6e8ec"),
      dim:    cssVar("--dim",    "#8b93a1"),
      accent: cssVar("--accent", "#4da3ff"),
      warn:   cssVar("--warn",   "#ffb020"),
      ok:     cssVar("--ok",     "#3ddc84"),
    };
  }

  function activeSteps() {
    if (!TAXI) return [];
    var st = TAXI.state;
    var slot = st.slots && st.slots[st.active];
    return (slot && slot.steps) ? slot.steps : [];
  }

  /* --------------------------------------------------------------- style block */
  function injectStyle() {
    if (document.getElementById("sim-style")) return;
    var s = document.createElement("style");
    s.id = "sim-style";
    s.textContent = [
      ".sim-ctl{display:flex;gap:10px;align-items:center;flex-wrap:wrap}",
      ".sim-btn{background:var(--panel2);color:var(--txt);border:1px solid var(--line);",
      "  border-radius:6px;padding:6px 12px;font:600 13px ui-sans-serif,system-ui,sans-serif;",
      "  cursor:pointer;transition:border-color .15s,color .15s}",
      ".sim-btn:hover{border-color:var(--accent);color:var(--accent)}",
      ".sim-slider{display:flex;align-items:center;gap:6px;font:12px ui-sans-serif,system-ui,sans-serif;color:var(--dim)}",
      ".sim-slider input[type=range]{width:110px;accent-color:var(--accent)}",
      ".sim-slider span{min-width:34px;color:var(--txt);font-variant-numeric:tabular-nums}",
      ".sim-ro-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:6px}",
      ".sim-ro-grid>div{background:var(--panel2);border:1px solid var(--line);border-radius:6px;padding:6px 8px}",
      ".sim-ro-grid span{display:block;font-size:11px;color:var(--dim);margin-bottom:2px}",
      ".sim-ro-grid b{font-size:14px;color:var(--txt);font-variant-numeric:tabular-nums}",
      ".sim-tl{margin-top:10px}",
      ".sim-tl-track{position:relative;height:14px;background:var(--panel2);border:1px solid var(--line);",
      "  border-radius:4px;overflow:hidden;display:flex}",
      ".sim-tl-seg{height:100%}",
      ".sim-tl-mark{position:absolute;top:-2px;bottom:-2px;width:2px;background:var(--accent);",
      "  box-shadow:0 0 5px var(--accent);pointer-events:none}",
      ".sim-table{width:100%;border-collapse:collapse;font:12px ui-monospace,SFMono-Regular,Menlo,monospace;",
      "  margin-top:10px;font-variant-numeric:tabular-nums}",
      ".sim-table th,.sim-table td{border-bottom:1px solid var(--line);padding:3px 6px;text-align:right;color:var(--txt)}",
      ".sim-table th{color:var(--dim);font-weight:600}",
      ".sim-empty{color:var(--warn);font:13px ui-sans-serif,system-ui,sans-serif;padding:8px 0}",
      ".sim-hud{font:12px ui-monospace,SFMono-Regular,Menlo,monospace;color:var(--dim);",
      "  font-variant-numeric:tabular-nums}",
    ].join("\n");
    document.head.appendChild(s);
  }

  /* ------------------------------------------------------------ canvas / view */
  function sizeCanvas() {
    var c = els.canvas;
    var w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) { w = 600; h = 600; }   // fallback; never read back c.width (would grow)
    var dpr = window.devicePixelRatio || 1;
    var pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (c.width !== pw || c.height !== ph) { c.width = pw; c.height = ph; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { w: w, h: h };
  }

  function computeView(w, h) {
    var R = TAXI.state.dohyo.diameter / 2;
    var outer = R + TAXI.state.dohyo.tawara;
    var scale = Math.min(w, h) / (2 * outer * 1.14);
    return { scale: scale, cx: w / 2, cy: h / 2, R: R };
  }

  function w2s(x, y) { return [view.cx + x * view.scale, view.cy - y * view.scale]; }
  function s2w(px, py) { return { x: (px - view.cx) / view.scale, y: (view.cy - py) / view.scale }; }

  function pointInPoly(px, py, poly) {
    var inside = false;
    for (var i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      var xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
      if (((yi > py) !== (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) inside = !inside;
    }
    return inside;
  }

  /* ------------------------------------------------------------- pose sampling */
  function poseAt(T) {
    var st = TAXI.activeStart();
    if (!sim || !sim.pts || !sim.pts.length) {
      return { x: st.x, y: st.y, th: st.heading * DEG, step: -1 };
    }
    var pts = sim.pts;
    if (T <= pts[0].t) return pts[0];
    var last = pts[pts.length - 1];
    if (T >= last.t) return last;
    var lo = 0, hi = pts.length - 1;
    while (hi - lo > 1) {
      var mid = (lo + hi) >> 1;
      if (pts[mid].t <= T) lo = mid; else hi = mid;
    }
    var a = pts[lo], b = pts[hi];
    var f = (T - a.t) / ((b.t - a.t) || 1);
    return {
      x: a.x + (b.x - a.x) * f,
      y: a.y + (b.y - a.y) * f,
      th: a.th + (b.th - a.th) * f,
      step: a.step,
    };
  }

  function handleWorld(p) {
    var hl = TAXI.state.robot.length / 2;
    return { x: p.x + Math.sin(p.th) * (hl + 2.5), y: p.y + Math.cos(p.th) * (hl + 2.5) };
  }

  /* --------------------------------------------------------------- path build */
  function buildStepPaths() {
    stepPaths = [];
    if (!sim) return;
    var pts = sim.pts;
    var map = new Map();
    for (var i = 1; i < pts.length; i++) {
      var s = pts[i].step;
      if (s < 0) continue;
      var arr = map.get(s);
      if (!arr) { arr = []; map.set(s, arr); arr.push(pts[i - 1]); } // seed with previous point
      arr.push(pts[i]);
    }
    map.forEach(function (arr, s) {
      stepPaths.push({ step: s, pts: arr, endTh: arr[arr.length - 1].th });
    });
    stepPaths.sort(function (a, b) { return a.step - b.step; });
  }

  /* ----------------------------------------------------------------- readouts */
  function cell(label, val) {
    return "<div><span>" + label + "</span><b>" + val + "</b></div>";
  }

  function buildReadout() {
    if (!els.readout) return;
    timelineMark = null;
    var sp = TAXI.activeStart();
    var steps = activeSteps();

    if (!steps.length) {
      els.readout.innerHTML =
        '<div class="sim-empty">No steps in this opening — add steps in the editor to simulate a path.</div>';
      return;
    }

    var exitTxt = sim.exit ? (sim.exit.t * 1000).toFixed(0) + " ms" : "stays on ring";
    var ste = TAXI.straightTimeToEdge(sp.heading, sp);
    var steTxt = ste == null ? "—" : ste.toFixed(0) + " ms";
    var endTh = normDeg(sim.endTh / DEG);
    var totalMs = steps.reduce(function (a, s) { return a + Math.max(0, s.ms); }, 0);

    // heading after each step (carry forward across zero-ms steps)
    var lastTh = {};
    for (var i = 0; i < sim.pts.length; i++) {
      if (sim.pts[i].step >= 0) lastTh[sim.pts[i].step] = sim.pts[i].th;
    }

    var rows = "";
    var cum = 0;
    var prevTh = sp.heading * DEG;
    steps.forEach(function (s, si) {
      cum += Math.max(0, s.ms);
      var th = (lastTh[si] !== undefined) ? lastTh[si] : prevTh;
      prevTh = th;
      rows += "<tr>" +
        "<td>" + si + "</td>" +
        "<td>" + s.left + "</td>" +
        "<td>" + s.right + "</td>" +
        "<td>" + s.ms + "</td>" +
        "<td>" + (cum / 1000).toFixed(2) + " s</td>" +
        "<td>" + normDeg(th / DEG).toFixed(0) + "°</td>" +
        "</tr>";
    });

    var segs = "";
    steps.forEach(function (s, si) {
      var w = totalMs > 0 ? (Math.max(0, s.ms) / totalMs * 100) : (100 / steps.length);
      segs += '<div class="sim-tl-seg" style="width:' + w + "%;background:" + stepColor(si) + '"></div>';
    });

    els.readout.innerHTML =
      '<div class="sim-ro-grid">' +
        cell("Opening time", sim.totalTime.toFixed(2) + " s") +
        cell("Travel distance", sim.distance.toFixed(1) + " cm") +
        cell("Edge-exit time", exitTxt) +
        cell("Final heading", endTh.toFixed(0) + "°") +
        cell("Final position", "(" + sim.endX.toFixed(1) + ", " + sim.endY.toFixed(1) + ") cm") +
        cell("Straight to edge", steTxt) +
      "</div>" +
      '<div class="sim-tl"><div class="sim-tl-track">' + segs +
        '<div class="sim-tl-mark"></div></div></div>' +
      '<table class="sim-table"><thead><tr>' +
        "<th>#</th><th>L</th><th>R</th><th>ms</th><th>Cum</th><th>Heading</th>" +
      "</tr></thead><tbody>" + rows + "</tbody></table>";

    timelineMark = els.readout.querySelector(".sim-tl-mark");
  }

  /* ------------------------------------------------------------------ drawing */
  function drawDohyo(th) {
    var R = view.R, tw = TAXI.state.dohyo.tawara;
    var cx = view.cx, cy = view.cy, s = view.scale;

    // tawara ring (white band) then clay surface on top
    ctx.beginPath(); ctx.arc(cx, cy, (R + tw) * s, 0, TAU); ctx.fillStyle = th.txt; ctx.fill();
    ctx.beginPath(); ctx.arc(cx, cy, R * s, 0, TAU); ctx.fillStyle = th.panel2; ctx.fill();
    ctx.lineWidth = 1; ctx.strokeStyle = th.line;
    ctx.beginPath(); ctx.arc(cx, cy, R * s, 0, TAU); ctx.stroke();

    // shikiri-sen: 1x10 cm, 5 cm either side of centre, perpendicular to facing (heading 0 = up)
    ctx.fillStyle = th.txt;
    [5, -5].forEach(function (yy) {
      var a = w2s(-5, yy + 0.5), b = w2s(5, yy - 0.5);
      ctx.fillRect(a[0], a[1], b[0] - a[0], b[1] - a[1]);
    });

    // centre cross
    ctx.strokeStyle = th.dim; ctx.lineWidth = 1;
    var c1 = w2s(-2, 0), c2 = w2s(2, 0), c3 = w2s(0, -2), c4 = w2s(0, 2);
    ctx.beginPath();
    ctx.moveTo(c1[0], c1[1]); ctx.lineTo(c2[0], c2[1]);
    ctx.moveTo(c3[0], c3[1]); ctx.lineTo(c4[0], c4[1]);
    ctx.stroke();
  }

  function drawProbe(th) {
    var st = TAXI.activeStart();
    var rad = probeHeading * DEG;
    var dx = Math.sin(rad), dy = Math.cos(rad);
    var R = view.R;

    // distance from start to the dohyo edge along the probe heading
    var b = 2 * (st.x * dx + st.y * dy);
    var c = st.x * st.x + st.y * st.y - R * R;
    var disc = b * b - 4 * c;
    var len = R;
    if (disc > 0) { var t = (-b + Math.sqrt(disc)) / 2; if (t > 0) len = t; }

    var a = w2s(st.x, st.y);
    var e = w2s(st.x + dx * len, st.y + dy * len);

    ctx.save();
    ctx.setLineDash([6, 5]);
    ctx.strokeStyle = th.warn; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(e[0], e[1]); ctx.stroke();
    ctx.restore();

    var time = TAXI.straightTimeToEdge(probeHeading, st);
    var label = time == null ? "no edge" : time.toFixed(0) + " ms";
    var mx = a[0] + (e[0] - a[0]) * 0.62, my = a[1] + (e[1] - a[1]) * 0.62;
    ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, monospace";
    var tw2 = ctx.measureText(label).width;
    ctx.globalAlpha = 0.85; ctx.fillStyle = th.panel;
    ctx.fillRect(mx - 4, my - 9, tw2 + 8, 16);
    ctx.globalAlpha = 1; ctx.fillStyle = th.warn;
    ctx.fillText(label, mx, my + 3);
  }

  function drawPath(th) {
    if (!stepPaths.length) return;
    ctx.lineWidth = 2.5; ctx.lineJoin = "round"; ctx.lineCap = "round";
    stepPaths.forEach(function (sp) {
      ctx.strokeStyle = stepColor(sp.step);
      ctx.beginPath();
      var s0 = w2s(sp.pts[0].x, sp.pts[0].y);
      ctx.moveTo(s0[0], s0[1]);
      for (var i = 1; i < sp.pts.length; i++) {
        var s = w2s(sp.pts[i].x, sp.pts[i].y);
        ctx.lineTo(s[0], s[1]);
      }
      ctx.stroke();
    });

    // edge-exit marker
    if (sim && sim.exit) {
      var e = w2s(sim.exit.x, sim.exit.y);
      ctx.save();
      ctx.strokeStyle = th.warn; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(e[0], e[1], 7, 0, TAU); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(e[0] - 5, e[1] - 5); ctx.lineTo(e[0] + 5, e[1] + 5);
      ctx.moveTo(e[0] + 5, e[1] - 5); ctx.lineTo(e[0] - 5, e[1] + 5);
      ctx.stroke();
      ctx.restore();
    }
  }

  function drawRobot(th) {
    var p = poseAt(anim.t);
    var cs = TAXI.corners(p.x, p.y, p.th).map(function (c) { return w2s(c.x, c.y); });

    ctx.beginPath();
    ctx.moveTo(cs[0][0], cs[0][1]);
    for (var i = 1; i < cs.length; i++) ctx.lineTo(cs[i][0], cs[i][1]);
    ctx.closePath();
    ctx.globalAlpha = 0.28; ctx.fillStyle = th.accent; ctx.fill();
    ctx.globalAlpha = 1; ctx.strokeStyle = th.accent; ctx.lineWidth = 2; ctx.stroke();

    // heading arrow
    var hl = TAXI.state.robot.length / 2;
    var c = w2s(p.x, p.y);
    var front = w2s(p.x + Math.sin(p.th) * hl, p.y + Math.cos(p.th) * hl);
    ctx.strokeStyle = th.txt; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(front[0], front[1]); ctx.stroke();

    var ang = Math.atan2(front[1] - c[1], front[0] - c[0]);
    ctx.beginPath();
    ctx.moveTo(front[0], front[1]);
    ctx.lineTo(front[0] - 8 * Math.cos(ang - 0.4), front[1] - 8 * Math.sin(ang - 0.4));
    ctx.lineTo(front[0] - 8 * Math.cos(ang + 0.4), front[1] - 8 * Math.sin(ang + 0.4));
    ctx.closePath(); ctx.fillStyle = th.txt; ctx.fill();

    // round rotation handle in front
    var hw = handleWorld(p);
    var hs = w2s(hw.x, hw.y);
    ctx.beginPath(); ctx.arc(hs[0], hs[1], 7, 0, TAU);
    ctx.fillStyle = th.accent; ctx.fill();
    ctx.strokeStyle = th.txt; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath(); ctx.arc(hs[0], hs[1], 3.2, 0.6, 0.6 + 4.6);
    ctx.strokeStyle = th.panel; ctx.lineWidth = 1.5; ctx.stroke();
  }

  function updateHud() {
    var steps = activeSteps();
    if (els.hud) {
      if (!sim || !steps.length) {
        els.hud.textContent = "no steps";
      } else {
        var p = poseAt(anim.t);
        var idx = p.step >= 0 ? p.step : 0;
        els.hud.textContent =
          "t " + anim.t.toFixed(2) + " / " + sim.totalTime.toFixed(2) + " s · step " +
          (idx + 1) + "/" + steps.length + " · " + anim.speed.toFixed(1) + "×";
      }
    }
    if (timelineMark && sim && sim.totalTime > 0) {
      timelineMark.style.left = (Math.min(1, anim.t / sim.totalTime) * 100).toFixed(2) + "%";
    }
  }

  function draw() {
    if (!els.canvas || !ctx) return;
    var size = sizeCanvas();
    var th = theme();
    ctx.clearRect(0, 0, size.w, size.h);

    if (!TAXI) {
      ctx.fillStyle = th.dim;
      ctx.font = "14px ui-sans-serif, system-ui, sans-serif";
      ctx.fillText("TAXI state not loaded", 16, 26);
      return;
    }

    view = computeView(size.w, size.h);
    drawDohyo(th);
    drawProbe(th);
    drawPath(th);
    drawRobot(th);
    updateHud();
  }

  /* --------------------------------------------------------------- public API */
  function render() {
    if (!TAXI) { dirty = true; return; }
    sim = TAXI.simulate(activeSteps(), TAXI.activeStart());
    buildStepPaths();
    buildReadout();
    dirty = true;
  }

  function setPlaying(v) {
    anim.playing = v;
    if (btnPlay) btnPlay.textContent = v ? "❚❚ Pause" : "▶ Play";
  }
  function play() {
    if (!TAXI || !sim || !sim.totalTime) return;
    if (anim.t >= sim.totalTime) anim.t = 0;
    setPlaying(true);
  }
  function pause() { setPlaying(false); }
  function reset() { setPlaying(false); anim.t = 0; dirty = true; }

  /* ------------------------------------------------------------- interaction */
  function onDown(e) {
    if (!TAXI || !view || !els.canvas) return;
    var rect = els.canvas.getBoundingClientRect();
    var px = e.clientX - rect.left, py = e.clientY - rect.top;

    // editing always happens at the start pose
    setPlaying(false); anim.t = 0;
    var p = poseAt(0);

    var hs = w2s(handleWorld(p).x, handleWorld(p).y);
    if (Math.hypot(px - hs[0], py - hs[1]) <= 11) {
      drag = { mode: "rotate" };
    } else {
      var poly = TAXI.corners(p.x, p.y, p.th).map(function (c) { return w2s(c.x, c.y); });
      if (pointInPoly(px, py, poly)) {
        var w = s2w(px, py);
        drag = { mode: "move", offX: w.x - p.x, offY: w.y - p.y };
      } else {
        return;
      }
    }
    try { els.canvas.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
    els.canvas.style.cursor = "grabbing";
    dirty = true;
    e.preventDefault();
  }

  function onMove(e) {
    if (!drag || !TAXI || !view || !els.canvas) return;
    var rect = els.canvas.getBoundingClientRect();
    var w = s2w(e.clientX - rect.left, e.clientY - rect.top);
    var st = TAXI.activeStart();

    if (drag.mode === "move") {
      var nx = snap(w.x - drag.offX, 0.5), ny = snap(w.y - drag.offY, 0.5);
      var R = TAXI.state.dohyo.diameter / 2;
      var r = Math.hypot(nx, ny);
      if (r > R) { nx = nx * R / r; ny = ny * R / r; }   // keep the centre on the ring
      st.x = nx; st.y = ny;
    } else {
      var p = poseAt(0);
      var h = normDeg(snap(Math.atan2(w.x - p.x, w.y - p.y) / DEG, 1));
      st.heading = h;
    }

    TAXI.emit();
    render();
    e.preventDefault();
  }

  function onUp(e) {
    if (drag && els.canvas) {
      try { els.canvas.releasePointerCapture(e.pointerId); } catch (err) { /* ignore */ }
    }
    drag = null;
    if (els.canvas) els.canvas.style.cursor = "grab";
  }

  /* ------------------------------------------------------------------ controls */
  function buildControls() {
    if (!els.controls) return;
    els.controls.innerHTML = "";
    els.controls.classList.add("sim-ctl");

    btnPlay = document.createElement("button");
    btnPlay.type = "button"; btnPlay.className = "sim-btn"; btnPlay.textContent = "▶ Play";
    btnPlay.addEventListener("click", function () { anim.playing ? pause() : play(); });
    els.controls.appendChild(btnPlay);

    var btnReset = document.createElement("button");
    btnReset.type = "button"; btnReset.className = "sim-btn"; btnReset.textContent = "⟲ Reset";
    btnReset.addEventListener("click", reset);
    els.controls.appendChild(btnReset);

    // speed multiplier
    var speedWrap = document.createElement("label");
    speedWrap.className = "sim-slider";
    speedWrap.appendChild(document.createTextNode("Speed"));
    var speed = document.createElement("input");
    speed.type = "range"; speed.min = "0.1"; speed.max = "4"; speed.step = "0.1"; speed.value = "1";
    var speedTxt = document.createElement("span");
    speedTxt.textContent = "1.0×";
    speed.addEventListener("input", function () {
      anim.speed = parseFloat(speed.value) || 1;
      speedTxt.textContent = anim.speed.toFixed(1) + "×";
      dirty = true;
    });
    speedWrap.appendChild(speed); speedWrap.appendChild(speedTxt);
    els.controls.appendChild(speedWrap);

    // probe heading
    var probeWrap = document.createElement("label");
    probeWrap.className = "sim-slider";
    probeWrap.appendChild(document.createTextNode("Probe"));
    var probe = document.createElement("input");
    probe.type = "range"; probe.min = "0"; probe.max = "359"; probe.step = "1"; probe.value = "0";
    var probeTxt = document.createElement("span");
    probeTxt.textContent = "0°";
    probe.addEventListener("input", function () {
      probeHeading = parseFloat(probe.value) || 0;
      probeTxt.textContent = probeHeading.toFixed(0) + "°";
      dirty = true;
    });
    probeWrap.appendChild(probe); probeWrap.appendChild(probeTxt);
    els.controls.appendChild(probeWrap);
  }

  /* --------------------------------------------------------------- animation */
  function frame(ts) {
    raf = requestAnimationFrame(frame);
    var dt = anim.last ? Math.min(0.1, (ts - anim.last) / 1000) : 0;
    anim.last = ts;

    if (anim.playing && sim && sim.totalTime > 0) {
      anim.t += dt * anim.speed;
      if (anim.t >= sim.totalTime) { anim.t = sim.totalTime; setPlaying(false); }
      dirty = true;
    }
    if (dirty) { draw(); dirty = false; }
  }

  /* --------------------------------------------------------------------- init */
  function init(opts) {
    opts = opts || {};
    els.canvas = opts.canvas || null;
    els.hud = opts.hud || null;
    els.readout = opts.readout || null;
    els.controls = opts.controls || null;
    TAXI = window.TAXI || null;

    injectStyle();

    if (els.canvas) {
      ctx = els.canvas.getContext("2d");
      els.canvas.style.touchAction = "none";
      els.canvas.style.cursor = "grab";
      els.canvas.addEventListener("pointerdown", onDown);
      els.canvas.addEventListener("pointermove", onMove);
      els.canvas.addEventListener("pointerup", onUp);
      els.canvas.addEventListener("pointercancel", onUp);
    }

    buildControls();

    window.addEventListener("resize", function () { dirty = true; });
    if (window.ResizeObserver && els.canvas) {
      try { new ResizeObserver(function () { dirty = true; }).observe(els.canvas); }
      catch (err) { /* ignore */ }
    }

    if (TAXI) render(); else dirty = true;
    if (!raf) raf = requestAnimationFrame(frame);
  }

  window.SimUI = { init: init, render: render, play: play, pause: pause, reset: reset };
})();
