// Logic test for the strategy designer: loads the three modules with a minimal
// stub, then exercises the C++ parse/serialize round-trip and the kinematics.
const fs = require("fs");
const vm = require("vm");

// --- minimal globals so the IIFEs can load ---
global.window = {};
global.document = {
  createElement: () => ({
    style: {}, dataset: {}, classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
    appendChild() {}, removeChild() {}, insertBefore() {}, addEventListener() {},
    setAttribute() {}, getAttribute() { return null; }, querySelector() { return null; },
    querySelectorAll() { return []; }, getContext() { return null; },
  }),
  getElementById: () => null,
  head: { appendChild() {} },
  body: { appendChild() {} },
  addEventListener() {},
};
global.requestAnimationFrame = () => 0;
global.ResizeObserver = undefined;

function load(f) { vm.runInThisContext(fs.readFileSync(f, "utf8"), { filename: f }); }

load("TAXI_Firmware/tools/js/state.js");
load("TAXI_Firmware/tools/js/slots.js");
load("TAXI_Firmware/tools/js/sim.js");

const T = global.window.TAXI;
let fails = 0;
const check = (name, cond, detail) => {
  console.log((cond ? "PASS" : "FAIL") + "  " + name + (detail ? "  [" + detail + "]" : ""));
  if (!cond) fails++;
};

check("state.js exposes TAXI", !!T);
check("slots.js exposes SlotsUI", !!global.window.SlotsUI);
check("sim.js exposes SimUI", !!global.window.SimUI);
check("SlotsUI has init/render", global.window.SlotsUI && typeof global.window.SlotsUI.init === "function" && typeof global.window.SlotsUI.render === "function");
check("SimUI has init/render/play/pause/reset",
  global.window.SimUI && ["init", "render", "play", "pause", "reset"].every(k => typeof global.window.SimUI[k] === "function"));
check("default state has 8 empty slots", T.state.slots.length === 8 && T.state.slots.every(s => s.steps.length === 0));
check("default loaded=false", T.state.loaded === false);
check("default start sits in the bottom half", T.state.start.y < 0, "y=" + T.state.start.y);
check("every preset start sits in the bottom half", T.PRESETS.every(p => p.start.y < 0));
check("most presets start near the outer edge", T.PRESETS.filter(p => p.start.y <= -25).length >= 6,
  T.PRESETS.map(p => p.start.y).join(","));
check("every preset start stays inside the dohyo",
  T.PRESETS.every(p => !T.robotOut(p.start.x, p.start.y, p.start.heading)));

// --- parse the real StrategyData.h ---
const src = fs.readFileSync("TAXI_Firmware/StrategyData.h", "utf8");
const parsed = T.parseStrategyData(src);
check("parsed 8 slots", parsed.slots.length === 8);
check("slot 0 = Rush {100,100,600}",
  parsed.slots[0].name === "Rush (dash)" && parsed.slots[0].search === "SEARCH_SPIN" &&
  JSON.stringify(parsed.slots[0].steps) === JSON.stringify([{ left: 100, right: 100, ms: 600 }]),
  JSON.stringify(parsed.slots[0]));
check("slot 6 = Tornado SEARCH_SWEEP, 2 steps",
  parsed.slots[6].search === "SEARCH_SWEEP" && parsed.slots[6].steps.length === 2,
  JSON.stringify(parsed.slots[6]));
check("slot 7 = Wait {0,0,300},{100,100,600}",
  JSON.stringify(parsed.slots[7].steps) === JSON.stringify([{ left: 0, right: 0, ms: 300 }, { left: 100, right: 100, ms: 600 }]),
  JSON.stringify(parsed.slots[7].steps));
check("slot 0 start = (0,-25,0)",
  JSON.stringify(parsed.slots[0].start) === JSON.stringify({ x: 0, y: -25, heading: 0 }),
  JSON.stringify(parsed.slots[0].start));
check("every slot has a start pose",
  parsed.slots.every(s => s.start && typeof s.start.x === "number" && typeof s.start.heading === "number"));
check("no opening drives itself off the dohyo",
  parsed.slots.every(s => {
    const sim = T.simulate(s.steps, s.start);
    return !sim.exit && !T.robotOut(sim.endX, sim.endY, sim.endTh);
  }),
  parsed.slots.map(s => { const sim = T.simulate(s.steps, s.start); return sim.exit ? "EXIT" : "ok"; }).join(","));
check("handoff is 65% of each opening's total time",
  parsed.slots.every(s => T.slotHandoffMs(s) === Math.round(T.slotTotalMs(s) * 0.65)),
  parsed.slots.map(s => T.slotHandoffMs(s)).join(","));
check("turning openings start already angled",
  parsed.slots[1].start.heading === 30 && parsed.slots[2].start.heading === -30 &&
  parsed.slots[4].start.heading === 45 && parsed.slots[5].start.heading === -45 &&
  parsed.slots[1].steps.length === 1 && parsed.slots[4].steps.length === 1);

// --- round-trip: serialize then re-parse ---
T.state.slots = parsed.slots;
T.state.loaded = true;
const out = T.serializeStrategyData();
const parsed2 = T.parseStrategyData(out);
let rt = true;
for (let i = 0; i < 8; i++) {
  const a = parsed.slots[i], b = parsed2.slots[i];
  if (a.name !== b.name || a.search !== b.search || JSON.stringify(a.steps) !== JSON.stringify(b.steps) ||
      JSON.stringify(a.start) !== JSON.stringify(b.start)) {
    rt = false;
    console.log("  MISMATCH slot " + i + ": " + JSON.stringify(a) + " vs " + JSON.stringify(b));
  }
}
check("serialize -> parse round-trip", rt);

// --- the generated file must be valid C++ shape ---
check("output has STRATEGIES[8]", /static const Strategy STRATEGIES\[8\] = \{/.test(out));
check("output has 8 TacticStep arrays", (out.match(/static const TacticStep s\d\[\]/g) || []).length === 8);
check("output has STRATEGY_NAMES", /STRATEGY_NAMES\[8\]/.test(out));
check("output carries start poses", /\{ s0, 1, SEARCH_SPIN, \{ 0, -25, 0 \}, 390 \}/.test(out));

// --- kinematics sanity ---
const sim = T.simulate(parsed.slots[1].steps, parsed.slots[1].start);   // Angled rush (right)
check("slot1 rushes straight from its 30deg start",
  Math.abs(sim.endTh / T.DEG - 30) < 0.5 && sim.exit === null,
  "endTh " + (sim.endTh / T.DEG).toFixed(1) + "deg, exit " + (sim.exit ? "yes" : "no"));
check("straightTimeToEdge(0) ~720ms from the shikiri-sen",
  Math.abs(T.straightTimeToEdge(0, { x: 0, y: -10, heading: 0 }) - 720) < 3,
  T.straightTimeToEdge(0, { x: 0, y: -10, heading: 0 }).toFixed(0) + "ms");

// --- per-slot start pose drives the kinematics ---
const nearEdge = { x: 0, y: 30, heading: 0 };   // close to the top edge, facing it
const tNear = T.straightTimeToEdge(0, nearEdge);
check("custom start changes straight-to-edge time", tNear != null && tNear < 200, tNear && tNear.toFixed(0) + "ms");
const simNear = T.simulate([{ left: 100, right: 100, ms: 1000 }], nearEdge);
check("simulate honours a custom start", simNear.exit !== null && simNear.exit.t < 0.2,
  simNear.exit ? (simNear.exit.t * 1000).toFixed(0) + "ms" : "no exit");
check("activeStart returns the active slot's pose",
  T.activeStart() === T.state.slots[T.state.active].start);

// --- empty-state safety ---
T.state.slots = [];
T.ensureSlots();
const emptySim = T.simulate(T.state.slots[0].steps);
check("empty opening simulates without throwing", emptySim.totalTime === 0 && emptySim.pts.length === 1);
const emptyOut = T.serializeStrategyData();
check("empty slots still serialize to valid shape",
  /static const TacticStep s0\[\] = \{ \{0,0,0\} \};/.test(emptyOut) && /STRATEGIES\[8\]/.test(emptyOut));

// --- HTML shell inline script syntax ---
const html = fs.readFileSync("TAXI_Firmware/tools/strategy-designer.html", "utf8");
const scripts = [...html.matchAll(/<script(?![^>]*\bsrc=)[^>]*>([\s\S]*?)<\/script>/g)].map(m => m[1]);
check("HTML has an inline script", scripts.length >= 1, scripts.length + " block(s)");
let jsOk = true;
for (const s of scripts) {
  try { new Function(s); } catch (e) { jsOk = false; console.log("  inline script error: " + e.message); }
}
check("HTML inline script parses", jsOk);
check("HTML loads the three modules",
  /js\/state\.js/.test(html) && /js\/slots\.js/.test(html) && /js\/sim\.js/.test(html));

console.log(fails ? "\n" + fails + " FAILED" : "\nall checks passed");
process.exit(fails ? 1 : 0);
