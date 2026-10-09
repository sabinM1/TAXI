/*
 * state.js — shared state, geometry, kinematics and C++ (StrategyData.h) I/O
 * for the TAXI strategy designer.
 *
 * Exposes a single global: window.TAXI
 *
 *   TAXI.state      { dohyo, robot, calib, start, active, slots, loaded }
 *   TAXI.on(fn)     subscribe to change notifications
 *   TAXI.emit()     notify subscribers (call after mutating state)
 *   TAXI.corners(x,y,th)          -> 4 footprint corners (cm)
 *   TAXI.robotOut(x,y,th)         -> true if any corner is off the dohyo
 *   TAXI.simulate(steps, start)   -> { pts, exit, totalTime, distance, endX, endY, endTh }
 *   TAXI.straightTimeToEdge(deg, start) -> ms for the robot to reach the edge driving straight
 *   TAXI.activeStart()            -> the active slot's start pose (mutable)
 *   TAXI.parseStrategyData(text)  -> { slots }  (throws on bad input)
 *   TAXI.serializeStrategyData()  -> string (a complete StrategyData.h)
 *   TAXI.PRESETS                  -> built-in openings
 *   TAXI.newSlot()                -> empty slot
 *   TAXI.ensureSlots()            -> pad slots to exactly 8
 *
 * Each slot is { name, search, steps:[{left,right,ms}], start:{x,y,heading} }.
 * state.start is only the DEFAULT placement handed to new slots; the pose that
 * actually drives the simulation is the active slot's own start.
 */
window.TAXI = (function () {
  "use strict";
  const DEG = Math.PI / 180;
  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

  /* ------------------------------------------------------------- state */
  const state = {
    dohyo: { diameter: 77, tawara: 2.5 },
    robot: { length: 10, width: 10 },
    calib: { straight: 60, turn: 750 },   // cm/s @100% duty ; deg/s @full differential
    start: { x: 0, y: -25, heading: 0 },  // DEFAULT placement for new slots (cm ; deg)
    active: 0,
    slots: [],                            // 8 x { name, search, steps, start }
    loaded: false,                        // true once a StrategyData.h has been imported
  };

  function cloneStart(s) { return { x: s.x, y: s.y, heading: s.heading }; }
  function newSlot() {
    return { name: "", search: "SEARCH_SPIN", steps: [], start: cloneStart(state.start) };
  }
  function ensureSlots() {
    while (state.slots.length < 8) state.slots.push(newSlot());
    if (state.slots.length > 8) state.slots.length = 8;
    // Backfill a start pose on slots that predate per-slot placement.
    state.slots.forEach(function (sl) { if (!sl.start) sl.start = cloneStart(state.start); });
  }
  ensureSlots();

  function activeStart() {
    const sl = state.slots[state.active];
    return (sl && sl.start) ? sl.start : state.start;
  }

  /* --------------------------------------------------------- listeners */
  const listeners = [];
  function on(fn) { listeners.push(fn); }
  function emit() { for (const fn of listeners) fn(); }

  /* ---------------------------------------------------------- geometry */
  function corners(x, y, th) {
    const hl = state.robot.length / 2, hw = state.robot.width / 2;
    const ct = Math.cos(th), st = Math.sin(th);
    // local: +y forward, +x right  ->  FL, FR, RR, RL
    return [[hw, hl], [-hw, hl], [-hw, -hl], [hw, -hl]].map(([lx, ly]) => ({
      x: x + lx * ct + ly * st,
      y: y - lx * st + ly * ct,
    }));
  }
  function robotOut(x, y, th) {
    const R = state.dohyo.diameter / 2;
    return corners(x, y, th).some(c => Math.hypot(c.x, c.y) > R);
  }

  /* -------------------------------------------------------- kinematics */
  const DT = 0.001;   // s
  function simulate(steps, start) {
    const sp = start || state.start;
    const V = state.calib.straight;
    let x = sp.x, y = sp.y, th = sp.heading * DEG;
    let t = 0, dist = 0, exit = null;
    const pts = [{ x, y, th, t, step: -1 }];
    (steps || []).forEach((st, si) => {
      const v = ((st.left + st.right) / 2) / 100 * V;                 // cm/s, fwd +
      const w = ((st.left - st.right) / 200) * state.calib.turn * DEG; // rad/s, cw +
      const dur = Math.max(0, st.ms) / 1000;
      let el = 0;
      while (el < dur - 1e-9) {
        const h = Math.min(DT, dur - el);
        th += w * h;
        x += v * Math.sin(th) * h;
        y += v * Math.cos(th) * h;
        dist += Math.abs(v) * h;
        t += h; el += h;
        pts.push({ x, y, th, t, step: si });
        if (!exit && robotOut(x, y, th)) exit = { x, y, t, step: si };
      }
    });
    return { pts, exit, totalTime: t, distance: dist, endX: x, endY: y, endTh: th };
  }

  function straightTimeToEdge(headingDeg, start) {
    const sp = start || state.start;
    const V = state.calib.straight;
    let x = sp.x, y = sp.y, th = headingDeg * DEG;
    for (let t = 0; t < 10; t += DT) {
      x += V * Math.sin(th) * DT; y += V * Math.cos(th) * DT;
      if (robotOut(x, y, th)) return t * 1000;
    }
    return null;
  }

  /* ------------------------------------------------------- C++ parsing */
  function parseStrategyData(text) {
    if (typeof text !== "string" || !text.length) throw new Error("empty file");

    const arrays = {};
    const arrRe = /static\s+const\s+TacticStep\s+s(\d+)\s*\[\s*\]\s*=\s*\{([\s\S]*?)\}\s*;/g;
    let m;
    while ((m = arrRe.exec(text))) {
      const steps = [];
      const stepRe = /\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(\d+)\s*\}/g;
      let sm;
      while ((sm = stepRe.exec(m[2]))) {
        steps.push({
          left: clamp(+sm[1], -100, 100),
          right: clamp(+sm[2], -100, 100),
          ms: Math.max(0, +sm[3]),
        });
      }
      arrays[+m[1]] = steps;
    }

    const rows = {};
    const tm = text.match(/static\s+const\s+Strategy\s+STRATEGIES\s*\[\s*8\s*\]\s*=\s*\{([\s\S]*?)\}\s*;/);
    if (tm) {
      // start pose is optional so older files without it still parse; the
      // trailing handoffMs is optional too (recomputed on export).
      const rowRe = /\{\s*s(\d+)\s*,\s*(\d+)\s*,\s*(SEARCH_\w+)\s*(?:,\s*\{\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\})?\s*(?:,\s*(\d+)\s*)?\}/g;
      let rm;
      while ((rm = rowRe.exec(tm[1]))) {
        rows[+rm[1]] = {
          search: rm[3],
          start: (rm[4] !== undefined)
            ? { x: +rm[4], y: +rm[5], heading: +rm[6] }
            : null,
        };
      }
    }

    const names = {};
    const nameRe = /\/\/\s*slot\s+(\d+)\s*:\s*(.*)/g;
    let nm;
    while ((nm = nameRe.exec(text))) names[+nm[1]] = nm[2].trim();

    if (!Object.keys(arrays).length && !Object.keys(rows).length)
      throw new Error("no TacticStep arrays or STRATEGIES table found");

    const slots = [];
    for (let i = 0; i < 8; i++) {
      slots.push({
        name: names[i] || ("slot " + i),
        search: (rows[i] && rows[i].search) || "SEARCH_SPIN",
        steps: arrays[i] || [],
        start: (rows[i] && rows[i].start) ? rows[i].start : cloneStart(state.start),
      });
    }
    return { slots };
  }

  /* --------------------------------------------------- C++ serializing */
  function cstr(s) {
    return '"' + String(s).replace(/\\/g, "\\\\").replace(/"/g, '\\"') + '"';
  }
  // Compact number for C++ output: trims trailing zeros, caps at 2 decimals.
  function num(v) {
    const n = Number(v);
    if (!isFinite(n)) return "0";
    return String(Math.round(n * 100) / 100);
  }
  // 65% of an opening's total time: after this much has run, a close opponent
  // outranks the committed move (see ST_TACTIC in StateMachine.h). Computed here,
  // at export, so the firmware never has to add the steps up.
  const HANDOFF_FRACTION = 0.65;
  function slotTotalMs(sl) {
    return (sl.steps || []).reduce((a, st) => a + (st.ms || 0), 0);
  }
  function slotHandoffMs(sl) {
    return Math.round(slotTotalMs(sl) * HANDOFF_FRACTION);
  }

  function serializeStrategyData() {
    const s = state;
    const L = [];
    L.push("/*");
    L.push(" * StrategyData.h — opening tactic data, managed by tools/strategy-designer.html");
    L.push(" *");
    L.push(" * Included by Strategies.h AFTER the TacticStep / Strategy / SearchStyle /");
    L.push(" * StartPose types are defined. Do NOT include this file directly.");
    L.push(" *");
    L.push(" * The strategy designer reads and writes THIS file: import it, edit the");
    L.push(" * openings on the dohyo, then export it back over this one. There is no other");
    L.push(" * copy of the strategies anywhere — this file is the single source of truth.");
    L.push(" *");
    L.push(` * Dohyo ${s.dohyo.diameter} cm | robot ${s.robot.length}x${s.robot.width} cm`);
    L.push(` * Calibration: ${s.calib.straight} cm/s straight @ 100% duty, ${s.calib.turn} deg/s @ full differential.`);
    L.push(" *");
    L.push(" * Each STRATEGIES row carries the start pose that opening assumes:");
    L.push(" *   { x, y, heading }  — x/y in cm from centre (+x right, +y toward the");
    L.push(" *   opponent), heading in degrees (0 = up, + = clockwise). The opening is");
    L.push(" *   open-loop, so this is placement metadata, not a steering input.");
    L.push(" *");
    L.push(" * The trailing number is handoffMs: 65% of the opening's total time. After");
    L.push(" * that point a close opponent outranks the committed move (see ST_TACTIC).");
    L.push(" */");
    L.push("#ifndef STRATEGY_DATA_H");
    L.push("#define STRATEGY_DATA_H");
    L.push("");
    s.slots.forEach((sl, i) => {
      const steps = sl.steps.length ? sl.steps : [{ left: 0, right: 0, ms: 0 }];
      const body = steps.map(x => `{${x.left},${x.right},${x.ms}}`).join(", ");
      L.push(`// slot ${i}: ${sl.name || "unnamed"}`);
      L.push(`static const TacticStep s${i}[] = { ${body} };`);
    });
    L.push("");
    L.push("static const Strategy STRATEGIES[8] = {");
    s.slots.forEach((sl, i) => {
      const n = sl.steps.length ? sl.steps.length : 1;
      const sp = sl.start || s.start;
      L.push(`  { s${i}, ${n}, ${sl.search}, { ${num(sp.x)}, ${num(sp.y)}, ${num(sp.heading)} }, ${slotHandoffMs(sl)} },    // ${i}: ${sl.name || "unnamed"}`);
    });
    L.push("};");
    L.push("");
    L.push("#if DEBUG_VERBOSE");
    L.push("// Human-readable opening names, index-aligned with STRATEGIES[].");
    L.push("static const char* const STRATEGY_NAMES[8] = {");
    s.slots.forEach((sl, i) => {
      L.push(`  ${cstr(sl.name || ("slot " + i))}${i < 7 ? "," : ""}`);
    });
    L.push("};");
    L.push("#endif");
    L.push("");
    L.push("#endif // STRATEGY_DATA_H");
    return L.join("\n") + "\n";
  }

  /* ------------------------------------------------------------ presets */
  // Start poses live in the BOTTOM half of the dohyo (the operator's side,
  // negative y), pushed back toward the outer edge. The shikiri-sen sits at
  // y = -5 cm, so y = -10 cm is the most forward legal centre for a 10 cm robot;
  // the tawara counts as part of the dohyo, so y = -33.5 cm is the most rearward
  // legal centre. The openings start deep to leave runway to accelerate, turn or
  // flank. These mirror the preset list in strategy-designer.html so the two
  // stay in step.
  const MID     = { x: 0, y: -25, heading: 0 };    // centred, facing up
  const ANG_R   = { x: 0, y: -25, heading: 30 };   // placed already angled right
  const ANG_L   = { x: 0, y: -25, heading: -30 };  // placed already angled left
  const FLANK_R = { x: 8, y: -25, heading: -90 };  // right lane, facing the centre
  const FLANK_L = { x: -8, y: -25, heading: 90 };  // left lane, facing the centre
  const BAIT_R  = { x: 0, y: -25, heading: 30 };   // centred, angled right
  const WAIT    = { x: 8, y: -25, heading: -90 };  // on the right flank lane
  const PRESETS = [
    { name: "Rush (dash)",                search: "SEARCH_SPIN",  steps: [[100, 100, 600]], start: MID },
    { name: "Angled rush (right)",        search: "SEARCH_ARC",   steps: [[100, 100, 550]], start: ANG_R },
    { name: "Angled rush (left)",         search: "SEARCH_ARC",   steps: [[100, 100, 550]], start: ANG_L },
    { name: "Feint (rush, pause, rush)",  search: "SEARCH_SPIN",  steps: [[100, 100, 200], [0, 0, 200], [100, 100, 500]], start: MID },
    { name: "Flank (right)",              search: "SEARCH_ARC",   steps: [[100, 100, 300]], start: FLANK_R },
    { name: "Flank (left)",               search: "SEARCH_ARC",   steps: [[100, 100, 300]], start: FLANK_L },
    { name: "Bait-and-flank (right)",     search: "SEARCH_ARC",   steps: [[-100, -100, 80], [100, 100, 600]], start: BAIT_R },
    { name: "Wait-then-rush (deceptive)", search: "SEARCH_SPIN",  steps: [[0, 0, 300], [100, 100, 300]], start: WAIT },
  ];

  return {
    DEG, state, on, emit,
    corners, robotOut, simulate, straightTimeToEdge, activeStart,
    parseStrategyData, serializeStrategyData,
    PRESETS, newSlot, ensureSlots,
    HANDOFF_FRACTION, slotTotalMs, slotHandoffMs,
  };
})();
