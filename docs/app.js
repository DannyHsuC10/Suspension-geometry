import {LabChart} from "./chart.js";
import {setupGeometryNotes} from "./geometry-notes.js";
import {downloadFile, setupProjectSettings} from "./settings.js";

const $ = id => document.getElementById(id);
const chart = new LabChart($("chart"), $("chart-legend"), $("chart-help"));
const showGeometryNotes = setupGeometryNotes($("geometry-notes"), $("geometry-note-title"), $("geometry-note-content"));
const titles = {front: "Front FV", rear: "Rear FV", sv: "Side View (SV)", we: "Wheel End (WE3D)", static: "Static Full Car"};
const formats = {state: "Pose CSV", points: "Hardpoints CSV", mapping: "Sweep CSV", svg: "SVG chart",
  png: "PNG chart", report: "HTML report / Print to PDF", step: "STEP (AP214)"};
const specialMetrics = {
  rc_y_prime_mm: "Roll center height y′ (mm)", rc_x_prime_mm: "Roll center lateral position x′ (mm)",
  pc_y_prime_mm: "Pitch center height y′ (mm)", pc_x_prime_mm: "Pitch center longitudinal position x′ (mm)",
  cg_y_prime_mm: "CG height y′ (mm)", cg_x_prime_mm: "CG position x′ (mm)",
  fvsa_left_mm: "Left FVSA (mm)", fvsa_right_mm: "Right FVSA (mm)",
  camber_left_deg: "Left camber (deg)", camber_right_deg: "Right camber (deg)",
  travel_left_mm: "Left wheel travel (mm)", travel_right_mm: "Right wheel travel (mm)",
  travel_front_mm: "Front wheel travel (mm)", travel_rear_mm: "Rear wheel travel (mm)",
};
const number = value => value == null ? "N/A" : typeof value === "number" ?
  Number(value.toPrecision(7)).toString() : String(value);

class Solver {
  constructor() {
    this.worker = new Worker(new URL("./py-worker.js", import.meta.url));
    this.requests = new Map();
    this.next = 0;
    this.worker.addEventListener("message", ({data}) => {
      const call = this.requests.get(data.id);
      if (!call) return;
      if (data.progress) {
        call.progress?.(data.progress);
        return;
      }
      this.requests.delete(data.id);
      if (data.ok) call.resolve(data.data);
      else call.reject(new Error(data.error));
    });
    this.worker.addEventListener("error", event => {
      this.fail(event.message || "The browser could not start the geometry solver.");
    });
  }

  request(action, payload, progress) {
    if (this.error) return Promise.reject(new Error(this.error));
    return new Promise((resolve, reject) => {
      const id = ++this.next;
      this.requests.set(id, {resolve, reject, progress});
      this.worker.postMessage({id, action, payload});
    });
  }

  fail(message) {
    this.error = message;
    for (const call of this.requests.values()) call.reject(new Error(message));
    this.requests.clear();
  }

  close() {
    this.worker.terminate();
    this.fail("Search canceled.");
  }
}

const solver = new Solver();
const request = solver.request.bind(solver);
const states = Object.fromEntries(Object.keys(titles).map(page => [page, {
  plot: page === "static" ? "iso" : "motion", mode: "travel", axle: "front", values: [0, 0],
  construction: true, staticReferences: true, groups: {front: true, rear: true, sv: true},
  hiddenSeries: [], showNormalCurves: true,
}]));
const state = () => states[page];
let page = "front";
let renderedPage;
let project, views, fields, derived, metricLabels, camberPlot, hardpointNote, lap, lapFile;
let showLap = true;
let designInputs = [];
let latest, requested = 0, completed = 0, running = false, scheduled = false;
let projectRevision = 0;
let exporting = false;
let searchTask;

function showStatus(message, error = false) {
  $("status").textContent = message;
  $("status").classList.toggle("status-error", Boolean(error));
}

function option(value, label) {
  return new Option(label, value);
}

function checkbox(text, checked, change) {
  const label = document.createElement("label");
  label.className = "check";
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = checked;
  input.addEventListener("change", () => change(input.checked));
  label.append(input, " " + text);
  return label;
}

function hardpointMode() {
  return Boolean(project?.hardpoints?.enabled);
}

function syncProject(summary) {
  ({project, derived, lap} = summary);
  projectRevision++;
  $("project-name").textContent = project.name;
}

async function onProject(candidate) {
  syncProject(await request("project", {project: candidate}));
  renderPage();
  showStatus("Vehicle settings applied.");
}

const projectSettings = setupProjectSettings({request, getProject: () => project, onProject, showStatus});

function fieldInput(group, metadata) {
  const {key, label, unit, hint, minimum, maximum, openMinimum, openMaximum, integer} = metadata;
  const wrapper = document.createElement("div");
  wrapper.className = "design-field";
  const view = document.createElement("label");
  view.className = "field";
  const fieldName = document.createElement("span");
  fieldName.textContent = label + (unit ? " (" + unit + ")" : "");
  const input = document.createElement("input");
  const readOnly = hardpointMode() && (group === "front" || group === "rear") && key !== "static_camber_deg";
  input.type = readOnly ? "text" : "number";
  input.value = readOnly ? number(derived[group][key]) : project[group][key];
  input.readOnly = readOnly;
  input.required = !readOnly;
  const description = document.createElement("small");
  description.id = "design-hint-" + group + "-" + key;
  description.className = "design-field-hint";
  description.textContent = readOnly ? "Read-only value derived from hardpoints." : hint;
  input.setAttribute("aria-describedby", description.id);
  if (!readOnly) {
    input.step = integer ? "1" : "any";
    if (minimum != null && !openMinimum) input.min = minimum;
    if (maximum != null && !openMaximum) input.max = maximum;
    designInputs.push({group, key, input});
  }
  view.append(fieldName, input);
  wrapper.append(view, description);
  const search = ((page === "front" || page === "rear") && group === page) ?
    {rc_height: "roll-center", fvsa: "fvsa"}[key] :
    page === "sv" && key === "svsa" ? "svsa" :
    (page === "sv" || page === "we") && group === "front" && key === "caster_deg" ? "caster" : null;
  if (search && !readOnly) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "button button-quiet search-field-button";
    button.textContent = {svsa: "Optimize front and rear SVSA", caster: "Optimize front caster",
      fvsa: "Optimize FVSA", "roll-center": "Optimize roll center"}[search];
    button.addEventListener("click", () => openSearch(search));
    wrapper.append(button);
  }
  return wrapper;
}

function balanceNote() {
  const note = document.createElement("div");
  note.className = "design-hint";
  const update = () => {
    const inputs = Object.fromEntries(designInputs.filter(field => field.group === "vehicle")
      .map(field => [field.key, field.input.valueAsNumber]));
    const {wheelbase, cg_to_front, roll_stiffness_ratio} = inputs;
    const front = (1 - cg_to_front / wheelbase) * 100;
    note.textContent = front > 0 && front < 100 && roll_stiffness_ratio > 0 ?
      "Rear RC optimization targets a 50/50 lateral load transfer split at zero travel. Static axle weight " +
      front.toFixed(2) + "% front / " + (100 - front).toFixed(2) + "% rear. Roll stiffness ratio " +
      number(roll_stiffness_ratio) + ". Fixed front RC " + number(project.front.rc_height) + " m." :
      "Check the wheelbase, CG position and front / rear roll stiffness ratio.";
  };
  designInputs.filter(item => item.group === "vehicle").forEach(field => field.input.addEventListener("input", update));
  update();
  return note;
}

function renderInputs() {
  designInputs = [];
  $("design-error").hidden = true;
  $("geometry-mode").textContent = hardpointMode() ?
    "Hardpoint mode · Derived axle values are read-only." : "Parameter mode";
  const groups = views[page].fields.map(({group, label, keys}) => {
    const details = document.createElement("details");
    details.className = "design-group";
    details.open = group === "vehicle" || (group !== "analysis" && (page !== "static" || group === "front"));
    const header = document.createElement("summary");
    header.textContent = label;
    const body = document.createElement("div");
    body.className = "field-grid";
    const lookup = new Map(fields[group].map(field => [field.key, field]));
    body.append(...keys.map(key => fieldInput(group, lookup.get(key))));
    details.append(header, body);
    return details;
  });
  $("design-fields").replaceChildren(...(page === "rear" ? [balanceNote()] : []), ...groups);
  $("design-action").hidden = false;
}

function readCandidate() {
  const candidate = structuredClone(project);
  for (const {group, key, input} of designInputs) {
    if (!input.validity.valid || !Number.isFinite(input.valueAsNumber)) {
      input.closest("details").open = true;
      input.reportValidity();
      throw new Error("Check " + input.parentElement.querySelector("span").textContent + ".");
    }
    candidate[group][key] = input.valueAsNumber;
  }
  return candidate;
}

function camberAvailable() {
  return lap?.camber && (page === "front" || page === "rear" || (page === "we" && lap.steering));
}

function renderViewOptions() {
  const v = state();
  const view = views[page];
  const plots = [...view.plots, ...(camberAvailable() ? [camberPlot] : [])];
  if (!plots.some(item => item.key === v.plot)) v.plot = plots[0].key;
  const cornering = v.plot === camberPlot.key;
  $("plot-select").replaceChildren(...plots.map(item => option(item.key, item.label)));
  $("plot-select").value = v.plot;
  $("mode-field").hidden = page === "we" || page === "static";
  $("mode-select").replaceChildren(option("travel", page === "sv" ? "Front / rear travel" : "Left / right travel"),
    option("pose", page === "sv" ? "Heave / Pitch" : "Heave / Roll"));
  $("mode-select").value = v.mode;
  $("axle-field").hidden = page !== "we" || cornering;
  $("axle-select").value = v.axle;
  $("lap-controls").hidden = !view.lap;
  $("lap-overlay-field").hidden = !lap?.name || !view.lapOverlay || cornering || v.plot === "motion";
  $("lap-overlay").checked = showLap;
  $("lap-name").textContent = lap?.name ? lap.name + " · " + lap.total.toLocaleString("en") + " samples" : "";
  $("clear-lap").hidden = !lap?.name;
  $("camber-controls").hidden = !cornering;
  const controls = view.camberSeries.map(({key, label}) => checkbox(label, !v.hiddenSeries.includes(key), checked => {
    v.hiddenSeries = v.hiddenSeries.filter(item => item !== key);
    if (!checked) v.hiddenSeries.push(key);
    scheduleUpdate();
  }));
  controls.push(checkbox("Normal distribution curves", v.showNormalCurves, checked => {
    v.showNormalCurves = checked;
    scheduleUpdate();
  }));
  $("camber-controls").replaceChildren(...controls);
  const display = page === "static" ? [
    ...[["front", "Front FV plane"], ["rear", "Rear FV plane"], ["sv", "SV plane"]].map(([key, label]) =>
      checkbox(label, v.groups[key], checked => { v.groups[key] = checked; scheduleUpdate(); })),
    checkbox("Instant center and kingpin references", v.staticReferences, checked => {
      v.staticReferences = checked;
      scheduleUpdate();
    }),
  ] : [checkbox("Construction references", v.construction, checked => {
    v.construction = checked;
    scheduleUpdate();
  })];
  if (page !== "static" && !(page === "we" && cornering)) {
    const zero = document.createElement("button");
    zero.type = "button";
    zero.className = "button button-quiet";
    zero.textContent = "Zero pose";
    zero.addEventListener("click", () => { v.values = [0, 0]; scheduleUpdate(); });
    display.push(zero);
  }
  $("view-options").replaceChildren(...display);
  const entries = [["state", formats.state], ["points", formats.points], ["mapping", cornering ? "Lap camber CSV" : formats.mapping],
    ["svg", formats.svg], ["png", formats.png], ["report", formats.report], ["step", formats.step]]
    .filter(([key]) => page === "static" ? !["state", "mapping"].includes(key) : page !== "we" || key !== "step");
  const lastFormat = $("export-select").value;
  $("export-select").replaceChildren(...entries.map(([key, label]) => option(key, label)));
  if (entries.some(([key]) => key === lastFormat)) $("export-select").value = lastFormat;
}

function renderPage() {
  for (const button of $("views").querySelectorAll("[data-page]")) {
    const selected = button.dataset.page === page;
    button.classList.toggle("active", selected);
    if (selected) button.setAttribute("aria-current", "page");
    else button.removeAttribute("aria-current");
  }
  $("view-heading").textContent = titles[page];
  $("view-note").textContent = views[page].note + (hardpointMode() ? " " + hardpointNote : "");
  $("pose-controls").replaceChildren();
  $("plot-controls").hidden = false;
  renderInputs();
  renderViewOptions();
  showGeometryNotes(page, titles[page]);
  clearResults();
  if (renderedPage !== page) {
    $("design-pane").scrollTop = 0;
    $("results-pane").scrollTop = 0;
    renderedPage = page;
  }
  scheduleUpdate();
}

function payload() {
  return {page, ...structuredClone(state()), showLap: showLap && Boolean(lap?.name)};
}

function controlChanged(index, value, row) {
  if (!Number.isFinite(value)) return;
  state().values[index] = value;
  row.querySelector('input[type="number"]').value = number(value);
  row.querySelector('input[type="range"]').value = value;
  scheduleUpdate();
}

function renderControls(controls) {
  const container = $("pose-controls");
  if (container.childElementCount !== controls.length ||
      controls.some((control, i) => container.children[i]?.dataset.label !== control.label)) {
    container.replaceChildren(...controls.map((control, index) => {
      const row = document.createElement("div");
      row.className = "pose-row";
      row.dataset.label = control.label;
      const label = document.createElement("label");
      label.htmlFor = "pose-slider-" + index;
      label.textContent = control.label;
      const range = document.createElement("input");
      range.type = "range";
      range.id = label.htmlFor;
      range.step = "any";
      const value = document.createElement("input");
      value.type = "number";
      value.step = "any";
      value.setAttribute("aria-label", control.label + " value");
      range.addEventListener("input", () => controlChanged(index, range.valueAsNumber, row));
      value.addEventListener("change", () => controlChanged(index, value.valueAsNumber, row));
      row.append(label, range, value);
      return row;
    }));
  }
  for (const [i, control] of controls.entries()) {
    state().values[i] = control.value;
    for (const input of container.children[i].querySelectorAll("input")) {
      input.min = control.min;
      input.max = control.max;
      input.disabled = control.disabled;
      if (input.type !== "number" || document.activeElement !== input) input.value = control.value;
    }
  }
}

function metricName(key) {
  if (metricLabels[key] || specialMetrics[key]) return metricLabels[key] || specialMetrics[key];
  const unit = key.match(/_(deg_per_mm|deg|mm|pct|m)$/)?.[1];
  let label = (unit ? key.slice(0, -unit.length - 1) : key).replaceAll("_", " ")
    .replace(/\b(rc|pc|cg|fvsa|svsa|kpi)\b/gi, match => match.toUpperCase())
    .replace(/\bprime\b/g, "′");
  label = label[0].toUpperCase() + label.slice(1);
  return label + (unit ? " (" + ({pct: "%", deg_per_mm: "deg/mm"}[unit] ?? unit) + ")" : "");
}

function metricRow(label, value) {
  const row = document.createElement("tr");
  const name = document.createElement("th");
  name.scope = "row";
  name.textContent = label;
  const cell = document.createElement("td");
  cell.textContent = value;
  row.append(name, cell);
  return row;
}

function clearResults() {
  latest = null;
  $("coverage").textContent = "";
  $("result-count").textContent = "";
  $("metrics-body").replaceChildren(metricRow("Calculating…", ""));
  chart.set(null, "loading", "Calculating geometry…");
}

function renderResult(result, args, current) {
  const {coverage, camberStats, spec, error, controls} = result;
  if (current) {
    latest = result;
    renderControls(controls);
  }
  const key = [args.page, args.plot, args.mode, args.axle].join(".");
  chart.set(spec, key, error || (coverage ? "No valid points in this sweep." : "No data available for this chart."));
  const count = coverage ? coverage.valid.toLocaleString("en") + " / " + coverage.total.toLocaleString("en") + " valid samples" : "";
  $("coverage").textContent = count;
  $("metrics-heading").textContent = camberStats ? "Cornering camber statistics" : "Current results";
  $("result-value-header").textContent = camberStats ? "Samples · Range (deg)" : "Value";
  const metrics = Object.entries(result.state?.metrics ?? {});
  const rows = camberStats ? camberStats.map(item => metricRow(item.label,
    item.count.toLocaleString("en") + " · " + number(item.min) + " to " + number(item.max))) :
    metrics.map(([metric, value]) => metricRow(metricName(metric), number(value)));
  if (camberStats && coverage?.straight != null) rows.unshift(metricRow("Straight-line samples excluded", coverage.straight.toLocaleString("en")));
  $("result-count").textContent = camberStats ? count : metrics.length ? metrics.length + " metrics" : "";
  $("metrics-body").replaceChildren(...(rows.length ? rows : [metricRow("Pose", error || "N/A")]));
  if (current) showStatus(error || titles[args.page] + (count ? " · " + count : " · Ready"), Boolean(error));
}

function refreshExport() {
  $("export-button").disabled = exporting || !latest || completed !== requested;
}

function scheduleUpdate() {
  requested++;
  refreshExport();
  if (running || scheduled) return;
  scheduled = true;
  requestAnimationFrame(update);
}

async function update() {
  scheduled = false;
  running = true;
  const token = requested;
  const revision = projectRevision;
  const args = payload();
  showStatus(args.plot === "motion" || args.page === "static" ? "Solving current geometry…" : "Calculating chart and sweep…");
  try {
    const result = await request("plot", args);
    const current = token === requested;
    const sameChart = revision === projectRevision && page === args.page &&
      ["plot", "mode", "axle"].every(key => state()[key] === args[key]);
    if (current || sameChart) renderResult(result, args, current);
  } catch (error) {
    if (token === requested) {
      latest = null;
      chart.set(null, "error", error.message);
      $("coverage").textContent = "";
      $("metrics-body").replaceChildren(metricRow("Calculation", error.message));
      showStatus(error.message, true);
    }
  } finally {
    running = false;
    completed = token;
    refreshExport();
    if (token !== requested) {
      scheduled = true;
      requestAnimationFrame(update);
    }
  }
}

async function exportData(kind, stepOptions) {
  if (!latest || completed !== requested || exporting) return;
  if (["svg", "png", "report"].includes(kind) && !chart.svg) {
    showStatus("Choose a chart with valid data before exporting.", true);
    return;
  }
  const args = payload();
  const name = "suspension_" + page + "_" + (kind === "mapping" && args.plot === camberPlot.key ? "lap_camber" : kind);
  const snapshot = {svg: kind === "report" ? chart.svg : undefined, coverage: latest.coverage};
  exporting = true;
  refreshExport();
  showStatus("Preparing " + formats[kind] + "…");
  try {
    if (kind === "svg") downloadFile(chart.svg, name + ".svg", "image/svg+xml;charset=utf-8");
    else if (kind === "png") downloadFile(await chart.png(), name + ".png");
    else {
      const result = await request("export-data", {...args, ...snapshot, kind, stepOptions,
        lapName: lap?.name, lapVisibleSeries: views[page].camberSeries.filter(item =>
          !args.hiddenSeries.includes(item.key)).map(item => item.key), lapNormalCurves: args.showNormalCurves});
      downloadFile(result.content, name + "." + result.extension, result.mime);
    }
    showStatus("Downloaded " + formats[kind] + ".");
  } catch (error) {
    showStatus(error.message, true);
  } finally {
    exporting = false;
    refreshExport();
  }
}

const searches = {
  "roll-center": {title: "Optimize roll center", fields: [["lower_m", "Minimum design height (m)", -0.005, -0.5, 1]]},
  fvsa: {title: "Optimize FVSA", note: "Find the longest front-view swing arm that meets the absolute camber limit across the loaded LapSim lap.",
    fields: [["max_camber_deg", "Maximum absolute camber (deg)", 2, 0, 90]]},
  svsa: {title: "Optimize SVSA", note: "Search both axles for signed swing arm lengths that meet the caster-change targets at straight heave.",
    fields: [["heave_mm", "Test heave (mm)", 7], ["front_min_deg", "Minimum front caster change (deg)", 0.15, 0],
      ["rear_max_deg", "Maximum rear caster change (deg)", 0.1, 0]]},
  caster: {title: "Optimize front caster", note: "Find the front design caster that places mean outer-wheel cornering camber closest to zero using LapSim body pose and steering data.", fields: []},
};

function openSearch(kind) {
  try {
    if (!$("design-form").reportValidity()) return;
    const search = searches[kind];
    searchTask = {kind, page, candidate: readCandidate(), running: false};
    $("search-title").textContent = search.title;
    $("search-note").textContent = search.note || (page === "rear" ?
      "Solve the rear roll center height for equal front and rear lateral load transfer. Verify that it stays above ground throughout the configured travel range." :
      "Find the lowest design roll center height that stays above ground throughout the configured travel range.");
    $("search-result").textContent = (kind === "fvsa" || kind === "caster") && !lapFile ?
      "Import a LapSim CSV from this view before running the search." : "";
    $("run-search").disabled = false;
    $("cancel-search").textContent = "Cancel";
    $("search-fields").replaceChildren(...search.fields.map(([name, title, value, min, max]) => {
      const label = document.createElement("label");
      label.className = "field";
      label.textContent = title;
      const input = document.createElement("input");
      input.type = "number";
      input.name = name;
      input.value = value;
      input.step = "any";
      input.required = true;
      if (min != null) input.min = min;
      if (max != null) input.max = max;
      label.append(input);
      return label;
    }));
    $("search-dialog").showModal();
  } catch (error) {
    showStatus(error.message, true);
  }
}

function closeSearch() {
  searchTask?.worker?.close();
  searchTask = null;
  $("search-dialog").close();
}

async function runSearch() {
  const task = searchTask;
  if (!task || task.running) return;
  if ((task.kind === "fvsa" && !lap?.camber) || (task.kind === "caster" && (!lap?.camber || !lap.steering))) {
    $("search-result").textContent = "Import a LapSim CSV with " + (task.kind === "caster" ?
      "body pose and both front steering angles." : "body pose and turn direction.");
    return;
  }
  task.running = true;
  $("run-search").disabled = true;
  $("search-result").textContent = "Starting optimization solver…";
  const settings = Object.fromEntries(Array.from(new FormData($("search-form")), ([key, value]) => [key, Number(value)]));
  try {
    const validated = await request("project-apply", {project: task.candidate, base: project});
    if (task !== searchTask) return;
    task.worker = new Solver();
    await task.worker.request("init");
    if (task.kind === "fvsa" || task.kind === "caster") await task.worker.request("lap_import", lapFile);
    const result = await task.worker.request("search", {kind: task.kind, page: task.page, project: validated.project, settings}, progress => {
      if (task !== searchTask) return;
      $("search-result").textContent = (progress.axle ? progress.axle + " axle · " : "") +
        progress.tested.toLocaleString("en") + " / " + progress.total.toLocaleString("en") + " candidates" +
        (progress.reason ? " · " + progress.reason : "");
    });
    if (task !== searchTask) return;
    if (result.project) {
      if (result.view) {
        const {page: target, ...view} = result.view;
        page = target;
        Object.assign(states[page], view);
      }
      await onProject(result.project);
    }
    $("search-result").textContent = result.status;
    $("cancel-search").textContent = "Close";
    task.running = false;
    showStatus(result.status);
  } catch (error) {
    if (task !== searchTask) return;
    task.running = false;
    $("search-result").textContent = error.message;
    $("run-search").disabled = false;
  } finally {
    task.worker?.close();
    task.worker = null;
  }
}

async function importLap(file) {
  const candidate = {name: file.name, text: await file.text()};
  lap = await request("lap_import", candidate);
  lapFile = candidate;
  showLap = true;
  if (state().plot === camberPlot.key) clearResults();
  renderViewOptions();
  scheduleUpdate();
}

function bind() {
  $("views").addEventListener("click", event => {
    const target = event.target.closest("button[data-page]")?.dataset.page;
    if (target && target !== page) {
      page = target;
      renderPage();
    }
  });
  $("design-form").addEventListener("submit", async event => {
    event.preventDefault();
    $("design-error").hidden = true;
    const button = event.submitter;
    button.disabled = true;
    try {
      const applied = await request("project-apply", {project: readCandidate(), base: project});
      await onProject(applied.project);
    } catch (error) {
      $("design-error").textContent = error.message;
      $("design-error").hidden = false;
      showStatus(error.message, true);
    } finally {
      button.disabled = false;
    }
  });
  $("plot-select").addEventListener("change", event => {
    state().plot = event.target.value;
    renderViewOptions();
    clearResults();
    scheduleUpdate();
  });
  $("mode-select").addEventListener("change", event => {
    Object.assign(state(), {mode: event.target.value, values: [0, 0]});
    scheduleUpdate();
  });
  $("axle-select").addEventListener("change", event => {
    state().axle = event.target.value;
    scheduleUpdate();
  });
  $("reset-view").addEventListener("click", () => chart.reset());
  $("open-settings").addEventListener("click", () => projectSettings.openSettings());
  $("download-project").addEventListener("click", () => projectSettings.downloadProject());
  $("open-project").addEventListener("click", () => $("project-file").click());
  $("project-file").addEventListener("change", async event => {
    const [file] = event.target.files;
    event.target.value = "";
    if (file) {
      try { await projectSettings.importProject(file); }
      catch (error) { showStatus(error.message, true); }
    }
  });
  $("import-lap").addEventListener("click", () => $("lap-file").click());
  $("lap-file").addEventListener("change", async event => {
    const [file] = event.target.files;
    event.target.value = "";
    if (file) {
      showStatus("Reading LapSim CSV…");
      try { await importLap(file); }
      catch (error) { showStatus(error.message, true); }
    }
  });
  $("lap-overlay").addEventListener("change", event => {
    showLap = event.target.checked;
    scheduleUpdate();
  });
  $("clear-lap").addEventListener("click", async () => {
    try {
      lap = await request("lap_clear");
      lapFile = null;
      renderViewOptions();
      scheduleUpdate();
    } catch (error) { showStatus(error.message, true); }
  });
  $("export-button").addEventListener("click", () => {
    const kind = $("export-select").value;
    if (kind !== "step") { exportData(kind); return; }
    $("step-construction").checked = page === "static" ? state().staticReferences : state().construction;
    $("step-dialog").showModal();
  });
  $("cancel-step").addEventListener("click", () => $("step-dialog").close());
  $("step-solids").addEventListener("change", event => {
    $("diameter-field").hidden = !event.target.checked;
    $("step-construction").disabled = event.target.checked;
    $("step-form").elements.tube_diameter_mm.disabled = !event.target.checked;
  });
  $("step-tires").addEventListener("change", event => { $("step-camber").disabled = !event.target.checked; });
  $("step-form").addEventListener("submit", event => {
    event.preventDefault();
    const options = Object.fromEntries(["export_solids", "include_tires", "include_static_camber", "show_construction"]
      .map(name => [name, event.target.elements[name].checked]));
    if (options.export_solids) options.tube_diameter_mm = event.target.elements.tube_diameter_mm.valueAsNumber;
    $("step-dialog").close();
    exportData("step", options);
  });
  $("search-form").addEventListener("submit", event => { event.preventDefault(); runSearch(); });
  $("cancel-search").addEventListener("click", closeSearch);
  $("search-dialog").addEventListener("cancel", event => { event.preventDefault(); closeSearch(); });
}

async function start() {
  bind();
  showGeometryNotes(page, titles[page]);
  try {
    const summary = await request("init");
    ({views, metricLabels, camberPlot, hardpointNote} = summary);
    syncProject(summary);
    ({fields} = await request("project-prepare", {project}));
    for (const element of document.querySelectorAll("[data-ready]")) element.disabled = false;
    renderPage();
  } catch (error) {
    const message = "Could not start the solver. " + error.message;
    $("view-note").textContent = message;
    chart.set(null, "startup-error", message);
    showStatus(message, true);
  }
}

start();
