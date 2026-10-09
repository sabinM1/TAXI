/*
 * slots.js — "strategy slots" editor for the TAXI strategy designer.
 *
 * Classic script (no modules). Exposes:
 *
 *   window.SlotsUI = {
 *     init(containerEl),   // build the UI inside containerEl (an HTMLElement)
 *     render(),            // re-render everything from TAXI.state
 *   };
 *
 * Reads/writes window.TAXI.state.slots (8 x { name, search, steps:[{left,right,ms}] }),
 * window.TAXI.state.active (0..7) and window.TAXI.state.loaded.
 * Calls window.TAXI.emit() after every mutation.
 *
 * The UI is rebuilt from scratch on structural changes (select slot, add/remove/move
 * step, load preset, paste). Live edits (typing a name, nudging a number) mutate state
 * and refresh only the affected card so the input keeps focus.
 */
(function () {
  "use strict";

  /* ------------------------------------------------------------ constants */
  var SEARCHES = ["SEARCH_SPIN", "SEARCH_ARC", "SEARCH_SWEEP"];
  var SEARCH_ABBR = { SEARCH_SPIN: "SPIN", SEARCH_ARC: "ARC", SEARCH_SWEEP: "SWEEP" };

  /* ------------------------------------------------------- module state */
  var root = null;        // wrapper element inside the host container
  var clipboard = null;   // copied slot (deep copy), module-scoped
  var startInputs = null; // [x, y, heading] inputs of the active editor, for live sync

  /* ------------------------------------------------------------ helpers */
  function T() { return window.TAXI; }

  function clamp(v, a, b) {
    if (a != null && v < a) v = a;
    if (b != null && v > b) v = b;
    return v;
  }

  // Escape user-provided text before it ever touches innerHTML.
  function esc(s) {
    return String(s == null ? "" : s).replace(/[&<>"']/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
    });
  }

  function dipCode(i) {
    return (i & 7).toString(2).padStart(3, "0");
  }

  function abbr(search) {
    return SEARCH_ABBR[search] || "SPIN";
  }

  function metaText(slot) {
    var n = slot.steps.length;
    var sp = slot.start || { x: 0, y: 0, heading: 0 };
    return abbr(slot.search) + " \u00b7 " + n + (n === 1 ? " step" : " steps") +
      " \u00b7 @" + fmtNum(sp.x) + "," + fmtNum(sp.y) + " " + fmtNum(sp.heading) + "\u00b0";
  }

  function fmtNum(v) {
    var n = Number(v);
    if (!isFinite(n)) return "0";
    return String(Math.round(n * 100) / 100);
  }

  function deepCopy(slot) {
    return JSON.parse(JSON.stringify(slot));
  }

  // Coloured bar preview. Bars are flex-grow proportional to ms so widths track
  // duration; a 0 ms step collapses to the CSS min-width.
  function previewHTML(slot) {
    var steps = slot.steps;
    if (!steps.length) {
      return '<div class="sl-bar sl-bar--pause" style="flex:1 1 0;opacity:.25"></div>';
    }
    return steps.map(function (s) {
      var sum = (s.left || 0) + (s.right || 0);
      var cls = sum > 0 ? "sl-bar--fwd" : sum < 0 ? "sl-bar--rev" : "sl-bar--pause";
      var grow = Math.max(0, s.ms || 0);
      return '<div class="sl-bar ' + cls + '" style="flex:' + grow + ' 1 0"></div>';
    }).join("");
  }

  /* --------------------------------------------------------- DOM builders */
  function field(labelText) {
    var f = document.createElement("label");
    f.className = "sl-field";
    var s = document.createElement("span");
    s.textContent = labelText;
    f.appendChild(s);
    return f;
  }

  function button(label, cls) {
    var b = document.createElement("button");
    b.type = "button";
    b.className = "sl-btn" + (cls ? " " + cls : "");
    b.textContent = label;
    return b;
  }

  function iconBtn(glyph, title, disabled, onClick) {
    var b = document.createElement("button");
    b.type = "button";
    b.className = "sl-icon-btn";
    b.textContent = glyph;
    b.title = title;
    b.setAttribute("aria-label", title);
    b.disabled = !!disabled;
    b.addEventListener("click", onClick);
    return b;
  }

  // Number input that clamps on commit and updates state live while typing.
  function numField(value, min, max, onSet) {
    var inp = document.createElement("input");
    inp.type = "number";
    inp.className = "sl-num";
    inp.value = value;
    if (min != null) inp.min = min;
    if (max != null) inp.max = max;

    inp.addEventListener("input", function () {
      var raw = inp.value.trim();
      // Let the user type "-", "+" or clear the field without snapping to 0.
      if (raw === "" || raw === "-" || raw === "+") return;
      var v = Number(raw);
      if (!isFinite(v)) return;
      onSet(clamp(v, min, max));
      T().emit();
      refreshCards();
    });
    inp.addEventListener("change", function () {
      var v = Number(inp.value);
      if (!isFinite(v)) v = min != null ? min : 0;
      v = clamp(v, min, max);
      inp.value = v;
      onSet(v);
      T().emit();
      refreshCards();
    });
    return inp;
  }

  /* ------------------------------------------------------------- cards */
  function buildCard(i) {
    var state = T().state;
    var slot = state.slots[i];

    var card = document.createElement("div");
    card.className = "sl-card" + (i === state.active ? " sl-card--active" : "");
    card.dataset.i = i;
    card.setAttribute("role", "button");
    card.tabIndex = 0;

    var top = document.createElement("div");
    top.className = "sl-card-top";
    var dip = document.createElement("span");
    dip.className = "sl-dip";
    dip.textContent = dipCode(i);
    top.appendChild(dip);
    card.appendChild(top);

    var name = document.createElement("div");
    name.className = "sl-name" + (slot.name ? "" : " sl-name--empty");
    name.textContent = slot.name || "empty";
    card.appendChild(name);

    var meta = document.createElement("div");
    meta.className = "sl-meta";
    meta.textContent = metaText(slot);
    card.appendChild(meta);

    var prev = document.createElement("div");
    prev.className = "sl-preview";
    prev.innerHTML = previewHTML(slot);
    card.appendChild(prev);

    function select() {
      T().state.active = i;
      T().emit();
      render();
    }
    card.addEventListener("click", select);
    card.addEventListener("keydown", function (e) {
      if (e.key === "Enter" || e.key === " ") { e.preventDefault(); select(); }
    });
    return card;
  }

  function buildGrid() {
    var grid = document.createElement("div");
    grid.className = "sl-grid";
    for (var i = 0; i < 8; i++) grid.appendChild(buildCard(i));
    return grid;
  }

  // Update card labels/previews in place (used for live edits, keeps focus).
  function refreshCards() {
    if (!root || !T()) return;
    var cards = root.querySelectorAll(".sl-card");
    for (var c = 0; c < cards.length; c++) {
      var card = cards[c];
      var slot = T().state.slots[+card.dataset.i];
      if (!slot) continue;
      var nameEl = card.querySelector(".sl-name");
      nameEl.textContent = slot.name || "empty";
      nameEl.classList.toggle("sl-name--empty", !slot.name);
      card.querySelector(".sl-meta").textContent = metaText(slot);
      card.querySelector(".sl-preview").innerHTML = previewHTML(slot);
    }
  }

  /* ------------------------------------------------------------ editor */
  function buildStepRow(st, j, total) {
    var row = document.createElement("div");
    row.className = "sl-step";

    var idx = document.createElement("span");
    idx.className = "sl-step-idx";
    idx.textContent = j + 1;
    row.appendChild(idx);

    row.appendChild(numField(st.left, -100, 100, function (v) { st.left = v; }));
    row.appendChild(numField(st.right, -100, 100, function (v) { st.right = v; }));
    row.appendChild(numField(st.ms, 0, null, function (v) { st.ms = v; }));

    var btns = document.createElement("div");
    btns.className = "sl-step-btns";
    btns.appendChild(iconBtn("\u2191", "Move up", j === 0, function () { moveStep(j, -1); }));
    btns.appendChild(iconBtn("\u2193", "Move down", j === total - 1, function () { moveStep(j, 1); }));
    btns.appendChild(iconBtn("\u29c9", "Duplicate", false, function () { dupStep(j); }));
    btns.appendChild(iconBtn("\u2715", "Delete", false, function () { delStep(j); }));
    row.appendChild(btns);
    return row;
  }

  function activeSteps() {
    return T().state.slots[T().state.active].steps;
  }

  function moveStep(j, d) {
    var steps = activeSteps();
    var k = j + d;
    if (k < 0 || k >= steps.length) return;
    var tmp = steps[j]; steps[j] = steps[k]; steps[k] = tmp;
    T().emit();
    render();
  }

  function dupStep(j) {
    var steps = activeSteps();
    var s = steps[j];
    steps.splice(j + 1, 0, { left: s.left, right: s.right, ms: s.ms });
    T().emit();
    render();
  }

  function delStep(j) {
    activeSteps().splice(j, 1);
    T().emit();
    render();
  }

  function buildEditor() {
    var state = T().state;
    var i = state.active;
    var slot = state.slots[i];

    var ed = document.createElement("div");
    ed.className = "sl-editor";

    var head = document.createElement("div");
    head.className = "sl-editor-head";
    head.textContent = "Editing slot " + i + " \u00b7 DIP " + dipCode(i) +
      (slot.steps.length ? "" : " \u00b7 empty");
    ed.appendChild(head);

    /* name + search */
    var row1 = document.createElement("div");
    row1.className = "sl-row";

    var nameField = field("Name");
    var nameInput = document.createElement("input");
    nameInput.type = "text";
    nameInput.className = "sl-input";
    nameInput.placeholder = "opening name";
    nameInput.value = slot.name;
    nameInput.addEventListener("input", function () {
      slot.name = nameInput.value;
      T().emit();
      refreshCards();
    });
    nameField.appendChild(nameInput);
    row1.appendChild(nameField);

    var searchField = field("Search style");
    var searchSel = document.createElement("select");
    searchSel.className = "sl-select";
    SEARCHES.forEach(function (s) {
      var o = document.createElement("option");
      o.value = s;
      o.textContent = s;
      if (s === slot.search) o.selected = true;
      searchSel.appendChild(o);
    });
    searchSel.addEventListener("change", function () {
      slot.search = searchSel.value;
      T().emit();
      refreshCards();
    });
    searchField.appendChild(searchSel);
    row1.appendChild(searchField);
    ed.appendChild(row1);

    /* start pose (per-slot placement) */
    var srow = document.createElement("div");
    srow.className = "sl-row";

    var sxInp = numField(slot.start.x, -40, 40, function (v) { slot.start.x = v; });
    var sxF = field("Start X (cm)");
    sxF.appendChild(sxInp);
    srow.appendChild(sxF);

    var syInp = numField(slot.start.y, -40, 40, function (v) { slot.start.y = v; });
    var syF = field("Start Y (cm)");
    syF.appendChild(syInp);
    srow.appendChild(syF);

    var shInp = numField(slot.start.heading, -180, 180, function (v) { slot.start.heading = v; });
    var shF = field("Heading (\u00b0)");
    shF.appendChild(shInp);
    srow.appendChild(shF);
    ed.appendChild(srow);
    startInputs = [sxInp, syInp, shInp];

    /* preset loader */
    var prow = document.createElement("div");
    prow.className = "sl-row";
    var presetField = field("Load preset");
    var presetSel = document.createElement("select");
    presetSel.className = "sl-select";
    T().PRESETS.forEach(function (p, idx) {
      var o = document.createElement("option");
      o.value = idx;
      o.textContent = p.name;
      presetSel.appendChild(o);
    });
    presetField.appendChild(presetSel);
    prow.appendChild(presetField);

    var loadBtn = button("Load preset");
    loadBtn.addEventListener("click", function () {
      var p = T().PRESETS[+presetSel.value];
      if (!p) return;
      slot.name = p.name;
      slot.search = p.search;
      slot.steps = p.steps.map(function (s) { return { left: s[0], right: s[1], ms: s[2] }; });
      if (p.start) slot.start = { x: p.start.x, y: p.start.y, heading: p.start.heading };
      T().emit();
      render();
    });
    prow.appendChild(loadBtn);
    ed.appendChild(prow);

    /* step list */
    var stepsWrap = document.createElement("div");
    stepsWrap.className = "sl-steps";

    var header = document.createElement("div");
    header.className = "sl-step sl-step--head";
    ["#", "Left %", "Right %", "ms", ""].forEach(function (t) {
      var s = document.createElement("span");
      s.textContent = t;
      header.appendChild(s);
    });
    stepsWrap.appendChild(header);

    if (!slot.steps.length) {
      var empty = document.createElement("div");
      empty.className = "sl-step-empty";
      empty.textContent = "No steps \u2014 this slot is empty. Add a step or load a preset.";
      stepsWrap.appendChild(empty);
    } else {
      slot.steps.forEach(function (st, j) {
        stepsWrap.appendChild(buildStepRow(st, j, slot.steps.length));
      });
    }
    ed.appendChild(stepsWrap);

    /* actions */
    var arow = document.createElement("div");
    arow.className = "sl-row sl-actions";

    var addBtn = button("+ Add step");
    addBtn.addEventListener("click", function () {
      slot.steps.push({ left: 0, right: 0, ms: 0 });
      T().emit();
      render();
    });
    arow.appendChild(addBtn);

    var clearBtn = button("Clear slot", "sl-btn--danger");
    clearBtn.addEventListener("click", function () {
      slot.steps = [];
      T().emit();
      render();
    });
    arow.appendChild(clearBtn);

    var copyBtn = button("Copy slot");
    copyBtn.addEventListener("click", function () {
      clipboard = deepCopy(slot);
      pasteBtn.disabled = false;
    });
    arow.appendChild(copyBtn);

    var pasteBtn = button("Paste slot");
    pasteBtn.disabled = !clipboard;
    pasteBtn.addEventListener("click", function () {
      if (!clipboard) return;
      var c = deepCopy(clipboard);
      slot.name = c.name;
      slot.search = c.search;
      slot.steps = c.steps;
      if (c.start) slot.start = { x: c.start.x, y: c.start.y, heading: c.start.heading };
      T().emit();
      render();
    });
    arow.appendChild(pasteBtn);

    ed.appendChild(arow);
    return ed;
  }

  /* -------------------------------------------------------------- render */
  // Keep the editor's start fields in step with state when the pose is changed
  // elsewhere (the sidebar panel, or dragging the robot on the dohyo). Skips
  // whichever field currently has focus so typing is never interrupted.
  function syncStartFields() {
    if (!startInputs || !T() || !T().state) return;
    var slot = T().state.slots[T().state.active];
    if (!slot || !slot.start) return;
    var vals = [slot.start.x, slot.start.y, slot.start.heading];
    for (var i = 0; i < 3; i++) {
      if (document.activeElement !== startInputs[i]) startInputs[i].value = vals[i];
    }
  }

  function render() {
    if (!root) return;
    root.innerHTML = "";

    if (!T() || !T().state) {
      var msg = document.createElement("div");
      msg.className = "sl-hint";
      msg.textContent = "TAXI core (state.js) is not loaded.";
      root.appendChild(msg);
      return;
    }

    var state = T().state;
    T().ensureSlots();
    if (typeof state.active !== "number" || state.active < 0 || state.active > 7) {
      state.active = 0;
    }

    if (!state.loaded) {
      var hint = document.createElement("div");
      hint.className = "sl-hint";
      hint.textContent = "No strategies loaded \u2014 import StrategyData.h to begin.";
      root.appendChild(hint);
    }

    root.appendChild(buildGrid());
    root.appendChild(buildEditor());
  }

  /* ---------------------------------------------------------------- init */
  function init(containerEl) {
    if (!containerEl || !containerEl.nodeType) return;
    injectStyle();
    root = document.createElement("div");
    root.className = "sl-root";
    containerEl.innerHTML = "";
    containerEl.appendChild(root);
    if (T() && typeof T().on === "function") T().on(syncStartFields);
    render();
  }

  /* --------------------------------------------------------------- style */
  function injectStyle() {
    if (document.getElementById("sl-styles")) return;
    var st = document.createElement("style");
    st.id = "sl-styles";
    st.textContent = [
      ".sl-root{color:var(--txt);font:13px/1.4 system-ui,-apple-system,Segoe UI,sans-serif;}",
      ".sl-hint{background:var(--panel2);border:1px solid var(--warn);color:var(--warn);",
      "  padding:8px 10px;border-radius:6px;margin-bottom:10px;}",
      ".sl-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-bottom:14px;}",
      "@media (max-width:720px){.sl-grid{grid-template-columns:repeat(2,minmax(0,1fr));}}",
      ".sl-card{background:var(--panel2);border:1px solid var(--line);border-radius:8px;",
      "  padding:8px;cursor:pointer;transition:border-color .12s,box-shadow .12s;}",
      ".sl-card:hover{border-color:var(--accent);}",
      ".sl-card:focus-visible{outline:2px solid var(--accent);outline-offset:1px;}",
      ".sl-card--active{border-color:var(--accent);box-shadow:0 0 0 2px var(--accent) inset;}",
      ".sl-card-top{display:flex;justify-content:space-between;align-items:center;}",
      ".sl-dip{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:12px;",
      "  letter-spacing:1px;color:var(--dim);}",
      ".sl-card--active .sl-dip{color:var(--accent);}",
      ".sl-name{font-weight:600;margin:4px 0 2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}",
      ".sl-name--empty{color:var(--dim);font-weight:400;font-style:italic;}",
      ".sl-meta{font-size:11px;color:var(--dim);margin-bottom:6px;}",
      ".sl-preview{display:flex;height:10px;gap:2px;background:var(--panel);border-radius:3px;overflow:hidden;}",
      ".sl-bar{height:100%;border-radius:2px;min-width:3px;}",
      ".sl-bar--fwd{background:var(--ok);}",
      ".sl-bar--rev{background:var(--warn);}",
      ".sl-bar--pause{background:var(--dim);opacity:.5;}",
      ".sl-editor{background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:12px;}",
      ".sl-editor-head{font-weight:600;margin-bottom:10px;}",
      ".sl-row{display:flex;gap:10px;flex-wrap:wrap;align-items:flex-end;margin-bottom:10px;}",
      ".sl-field{display:flex;flex-direction:column;gap:3px;font-size:11px;color:var(--dim);}",
      ".sl-input,.sl-select,.sl-num{background:var(--panel);border:1px solid var(--line);",
      "  color:var(--txt);border-radius:5px;padding:5px 7px;font:inherit;min-width:0;}",
      ".sl-input:focus,.sl-select:focus,.sl-num:focus{outline:none;border-color:var(--accent);}",
      ".sl-input{min-width:200px;}",
      ".sl-steps{border:1px solid var(--line);border-radius:6px;overflow:hidden;margin-bottom:10px;}",
      ".sl-step{display:grid;grid-template-columns:26px minmax(0,1fr) minmax(0,1fr) minmax(0,1fr) auto;",
      "  gap:6px;align-items:center;padding:6px 8px;border-bottom:1px solid var(--line);}",
      ".sl-step:last-child{border-bottom:none;}",
      ".sl-step--head{font-size:11px;color:var(--dim);background:var(--panel);}",
      ".sl-step-idx{color:var(--dim);font-family:ui-monospace,Consolas,monospace;text-align:center;}",
      ".sl-num{width:100%;}",
      ".sl-step-btns{display:flex;gap:4px;}",
      ".sl-step-empty{padding:12px;color:var(--dim);font-style:italic;}",
      ".sl-btn{background:var(--panel);border:1px solid var(--line);color:var(--txt);",
      "  border-radius:5px;padding:5px 10px;cursor:pointer;font:inherit;}",
      ".sl-btn:hover:not(:disabled){border-color:var(--accent);}",
      ".sl-btn:disabled{opacity:.4;cursor:default;}",
      ".sl-btn--danger:hover:not(:disabled){border-color:var(--warn);color:var(--warn);}",
      ".sl-icon-btn{background:var(--panel);border:1px solid var(--line);color:var(--txt);",
      "  border-radius:5px;width:26px;height:26px;line-height:1;cursor:pointer;font:inherit;padding:0;}",
      ".sl-icon-btn:hover:not(:disabled){border-color:var(--accent);}",
      ".sl-icon-btn:disabled{opacity:.35;cursor:default;}",
      ".sl-actions{margin-bottom:0;}",
    ].join("\n");
    document.head.appendChild(st);
  }

  /* --------------------------------------------------------------- export */
  window.SlotsUI = { init: init, render: render };
})();
