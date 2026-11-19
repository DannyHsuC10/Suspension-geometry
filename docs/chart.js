const palette = ["#38bdf8", "#fbbf24", "#c4b5fd", "#2dd4bf", "#fb7185", "#f8fafc"];
const xml = value => String(value ?? "").replace(/[&<>"']/g, char => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&apos;",
})[char]);
const finite = value => typeof value === "number" && Number.isFinite(value);
const valid = (point, size = 2) => point?.length >= size && point.slice(0, size).every(finite);
const coord = value => Number(value.toFixed(2));
const amount = value => Number(value.toPrecision(4)).toString();
const stroke = (color, width = 1, dash = []) =>
  ' fill="none" stroke="' + xml(color) + '" stroke-width="' + width +
  '" stroke-linejoin="round" stroke-linecap="round"' +
  (dash.length ? ' stroke-dasharray="' + dash.join(" ") + '"' : "");
const path = (data, color, width, dash) =>
  '<path d="' + data + '"' + stroke(color, width, dash) + "/>";
const pointAttr = ([x, y]) => ' cx="' + coord(x) + '" cy="' + coord(y) + '"';
const text = (x, y, label, options = {}) =>
  '<text x="' + coord(x) + '" y="' + coord(y) + '" fill="' + (options.color ?? "#c4d4e8") +
  '" font-size="' + (options.size ?? 13) + '" text-anchor="' + (options.anchor ?? "start") + '"' +
  (options.transform ? ' transform="' + options.transform + '"' : "") + ">" + xml(label) + "</text>";

function extent(points, dimension) {
  const ranges = Array.from({length: dimension}, () => [Infinity, -Infinity]);
  for (const point of points) {
    if (!valid(point, dimension)) continue;
    for (let i = 0; i < dimension; i++) {
      ranges[i][0] = Math.min(ranges[i][0], point[i]);
      ranges[i][1] = Math.max(ranges[i][1], point[i]);
    }
  }
  return ranges.map(([min, max]) => !finite(min) ? [-1, 1] : min === max ?
    [min - Math.max(Math.abs(min) * 0.05, 0.01), max + Math.max(Math.abs(max) * 0.05, 0.01)] : [min, max]);
}

function ticks([min, max]) {
  const rough = (max - min) / 6;
  if (!finite(rough) || rough <= 0) return [];
  const base = 10 ** Math.floor(Math.log10(rough));
  const step = ([1, 2, 2.5, 5, 10].find(value => value >= rough / base) ?? 10) * base;
  const start = Math.ceil(min / step - 1e-9);
  const end = Math.floor(max / step + 1e-9);
  return Array.from({length: Math.min(16, Math.max(0, end - start + 1))}, (_, i) => (start + i) * step);
}

const fit = (ranges, equal, bounds, padding = 0.08) => {
  const width = bounds.right - bounds.left;
  const height = bounds.bottom - bounds.top;
  let [[left, right], [bottom, top]] = ranges.map(([min, max]) => [
    min - (max - min) * padding, max + (max - min) * padding,
  ]);
  if (equal) {
    const scale = Math.max((right - left) / width, (top - bottom) / height);
    const cx = (left + right) / 2;
    const cy = (top + bottom) / 2;
    [left, right] = [cx - width * scale / 2, cx + width * scale / 2];
    [bottom, top] = [cy - height * scale / 2, cy + height * scale / 2];
  }
  return [[left, right], [bottom, top]];
};

function projection(kind, azimuth, elevation) {
  const ca = Math.cos(azimuth), sa = Math.sin(azimuth);
  const ce = Math.cos(elevation), se = Math.sin(elevation);
  switch (kind) {
    case "Front": return ([, y, z]) => [-y, z];
    case "Rear": return ([, y, z]) => [y, z];
    case "Side": return ([x, , z]) => [x, z];
    case "Top": return ([x, y]) => [-y, x];
    case "Iso": return ([x, y, z]) => [ca*x - sa*y, -se*(sa*x + ca*y) + ce*z, ce*(sa*x + ca*y) + se*z];
    default: return ([x, y]) => [x, y];
  }
}

function lineData(points, screen) {
  let connected = false;
  let data = "";
  for (const point of points) {
    if (!valid(point)) {
      connected = false;
      continue;
    }
    const [x, y] = screen(point);
    data += (connected ? "L" : "M") + coord(x) + " " + coord(y) + " ";
    connected = true;
  }
  return data.trim();
}

function axes(sx, sy, inverse, xlabel, ylabel, bounds, height) {
  const {left, right, top, bottom} = bounds;
  const result = [];
  for (const x of ticks([inverse(left, bottom)[0], inverse(right, bottom)[0]])) {
    const v = sx(x);
    result.push(path("M" + v + " " + top + "V" + bottom, "#273a56"),
      text(v, bottom + 20, amount(x), {anchor: "middle", size: 11}));
  }
  for (const y of ticks([inverse(left, bottom)[1], inverse(left, top)[1]])) {
    const v = sy(y);
    result.push(path("M" + left + " " + v + "H" + right, "#273a56"),
      text(left - 8, v + 4, amount(y), {anchor: "end", size: 11}));
  }
  result.push('<rect x="' + left + '" y="' + top + '" width="' + (right-left) + '" height="' + (bottom-top) +
    '" fill="none" stroke="#425674"/>');
  if (xlabel) result.push(text((left+right)/2, height - 13, xlabel, {
    anchor: "middle", size: Math.min(13, (right-left) / Math.max(xlabel.length * .55, 1))}));
  if (ylabel) result.push(text(16, (top+bottom)/2, ylabel, {
    anchor: "middle", size: 12, transform: "rotate(-90 16 " + (top+bottom)/2 + ")",
  }));
  return result.join("");
}

function markers(items, project, screen, dimension, bounds) {
  return items.map(item => {
    if (!valid(item.position, dimension)) return "";
    const location = screen(project(item.position));
    const [x, y] = location;
    const radius = item.radius ?? 0;
    const color = xml(item.color ?? "#fb7185");
    const [dx, dy] = item.offset ?? [10, -12];
    const label = item.label_lines?.length ? item.label_lines : [item.name];
    const description = label.join(" · ");
    const start = valid(item.start, dimension) ? screen(project(item.start)) : null;
    const leader = start ? path(lineData([start, location], point => point), color, 1.15, [4, 4]) : "";
    const endpoint = radius > 0 ? '<circle' + pointAttr(location) + ' r="' + radius + '" fill="' +
      (item.hollow ? "#111d31" : color) + '" stroke="' + color + '" stroke-width="1.5"/>' : "";
    const showLabel = radius >= 6 || label.length > 1 || start;
    const labelWidth = Math.max(...label.map(value => String(value).length)) * 6.6 + 16;
    const right = x + dx + labelWidth > bounds.right;
    const tx = right ? Math.max(bounds.left + 8, x - labelWidth - 10) : Math.max(bounds.left + 8, x + dx);
    const ty = Math.min(bounds.bottom - 10 - (label.length - 1) * 16, Math.max(bounds.top + 20, y + dy));
    const box = showLabel && label.length > 1 ? '<rect x="' + (tx - 6) + '" y="' + (ty - 13) + '" width="' +
      labelWidth + '" height="' + (label.length * 16 + 5) + '" rx="4" fill="#111d31" stroke="' + color + '" opacity=".92"/>' : "";
    return "<g><title>" + xml(description) + "</title>" + leader + endpoint + box +
      (showLabel ? label.map((value, index) => text(tx, ty + index * 16, value, {color, size: 12})).join("") : "") +
      "</g>";
  }).join("");
}

function lapPoints(points, project, screen, dimension, colors) {
  if (!points?.length) return "";
  return '<g fill="#fbbf24" fill-opacity=".65">' + points.map((point, index) =>
    valid(point, dimension) ? '<circle' + pointAttr(screen(project(point))) +
      ' r="' + (colors && index % 2 === 0 ? '3.2' : '2.3') + '"' +
      (colors ? ' fill="' + xml(colors[index]) + '"' : '') + '/>' : "").join("") + "</g>";
}

function meshLayer(mesh, project, screen, [min, max]) {
  const faces = [];
  for (let row = 1; row < mesh.length; row++) {
    for (let col = 1; col < mesh[row].length; col++) {
      const corners = [mesh[row-1][col-1], mesh[row-1][col], mesh[row][col], mesh[row][col-1]];
      if (!corners.every(point => valid(point, 3))) continue;
      const projected = corners.map(project);
      const depth = projected.reduce((total, point) => total + point[2], 0);
      const height = corners.reduce((total, point) => total + point[2], 0) / 4;
      const hue = 204 - 168 * Math.max(0, Math.min(1, (height - min) / (max - min)));
      const vertices = projected.map(point => screen(point).map(coord).join(",")).join(" ");
      faces.push({depth, html: '<polygon points="' + vertices + '" fill="hsl(' + hue.toFixed(1) +
        ' 72% 46%)" stroke="#0f1b30" stroke-opacity=".4" stroke-width=".7"/>'});
    }
  }
  return faces.sort((a, b) => a.depth - b.depth).map(face => face.html).join("");
}

function seriesLayer(spec, series, curves, screen) {
  const result = [];
  const baseline = screen([0, 0])[1];
  for (const [index, item] of series.entries()) {
    const color = item.color ?? palette[index % palette.length];
    const samples = item.x.map((x, i) => [x, item.y[i]]);
    if (spec.bin_edges) {
      result.push(...samples.map((point, i) => {
        if (!valid(point) || point[1] <= 0) return "";
        const [x, y] = screen([spec.bin_edges[i], point[1]]);
        const [x2] = screen([spec.bin_edges[i + 1], 0]);
        const fill = item.hollow ? 'fill="none" stroke="' + xml(color) + '" stroke-dasharray="4 2"' :
          'fill="' + xml(color) + '" fill-opacity=".32" stroke="' + xml(color) + '"';
        return '<rect x="' + coord(x) + '" y="' + coord(y) + '" width="' + coord(x2-x) +
          '" height="' + coord(baseline-y) + '" ' + fill + "><title>" +
          xml(item.name + " · " + point[1] + " samples") + "</title></rect>";
      }));
    } else {
      result.push(path(lineData(samples, screen), color, item.width ?? 1.8, item.dash ?? []));
    }
  }
  for (const item of curves) {
    result.push(path(lineData(item.x.map((x, i) => [x, item.y[i]]), screen), item.color, 2, item.dash ?? []));
  }
  return result.join("");
}

export class LabChart {
  constructor(host, legend, help) {
    this.host = host;
    this.legend = legend;
    this.help = help;
    this.reset(false);
    host.addEventListener("pointerdown", event => {
      if (!this.spec || event.button !== 0) return;
      this.pointer = [event.clientX, event.clientY];
      host.setPointerCapture(event.pointerId);
    });
    host.addEventListener("pointermove", event => {
      if (!this.pointer) {
        this.hover(event);
        return;
      }
      const [x, y] = this.pointer;
      this.pointer = [event.clientX, event.clientY];
      this.move(event.clientX - x, event.clientY - y, event.shiftKey);
    });
    for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
      host.addEventListener(type, () => { this.pointer = null; });
    }
    host.addEventListener("pointerleave", () => { this.help.textContent = this.instructions; });
    host.addEventListener("dblclick", () => this.reset());
    new ResizeObserver(() => {
      if (this.spec && this.host.clientWidth !== this.observedWidth) this.schedule();
      this.observedWidth = this.host.clientWidth;
    }).observe(host);
    host.addEventListener("wheel", event => {
      if (!this.spec || !(event.ctrlKey || event.metaKey)) return;
      event.preventDefault();
      this.zoomBy(Math.exp(-event.deltaY * 0.0012));
    }, {passive: false});
    host.addEventListener("keydown", event => {
      if (!this.spec) return;
      const delta = {ArrowLeft: [-20, 0], ArrowRight: [20, 0], ArrowUp: [0, -20], ArrowDown: [0, 20]}[event.key];
      if (delta) this.move(...delta, event.shiftKey);
      else if (event.key === "+" || event.key === "=") this.zoomBy(1.12);
      else if (event.key === "-") this.zoomBy(1 / 1.12);
      else if (event.key === "Home") this.reset();
      else return;
      event.preventDefault();
    });
  }

  reset(render = true) {
    this.azimuth = -0.65;
    this.elevation = 0.5;
    this.pan = [0, 0];
    this.zoom = 1;
    this.fittedRanges = null;
    if (render) this.render();
  }

  move(dx, dy, pan) {
    if (this.spec.projection === "Iso" && !pan) {
      this.azimuth += dx * 0.009;
      this.elevation = Math.max(-1.3, Math.min(1.3, this.elevation + dy * 0.009));
    } else {
      const ratio = this.width / this.host.getBoundingClientRect().width;
      this.pan[0] += dx * ratio;
      this.pan[1] += dy * ratio;
    }
    this.schedule();
  }

  zoomBy(factor) {
    this.zoom = Math.max(0.2, Math.min(22, this.zoom * factor));
    this.schedule();
  }

  schedule() {
    if (this.frame) return;
    this.frame = requestAnimationFrame(() => {
      this.frame = null;
      this.render();
    });
  }

  set(spec, key, message = "No data available for this chart.") {
    if (key !== this.key) this.reset(false);
    this.spec = spec;
    this.key = key;
    this.message = message;
    this.render();
  }

  hover(event) {
    if (!this.readCursor) return;
    const area = this.host.getBoundingClientRect();
    const label = this.readCursor(event.clientX - area.left, event.clientY - area.top);
    this.help.textContent = label || this.instructions;
  }

  render() {
    const spec = this.spec;
    const meshPoints = spec?.mesh?.flat().filter(point => valid(point, 3)) ?? [];
    const hasChart = spec?.mesh ? meshPoints.length : spec?.world_lines?.length || spec?.series?.length;
    this.readCursor = null;
    this.instructions = spec?.projection === "Iso" ?
      "Drag to rotate. Shift-drag to pan. Ctrl/⌘ + scroll to zoom." : "Drag to pan. Ctrl/⌘ + scroll to zoom.";
    this.help.textContent = this.instructions;
    if (!hasChart) {
      this.svg = "";
      const unavailable = spec?.mesh ? "The sweep contains no finite geometry for this chart." : this.message;
      this.host.innerHTML = '<p class="chart-empty">' + xml(spec?.current_status || unavailable || "Waiting for chart…") + "</p>";
      this.legend.hidden = true;
      return;
    }
    const W = this.width = Math.max(280, this.host.clientWidth || 1000);
    const H = this.height = Math.round(Math.min(640, Math.max(295, W * .57)));
    const bounds = {left: W < 420 ? 55 : 67, right: W - (W < 480 ? 25 : 46),
      top: W < 560 && spec.current_status ? 62 : 48, bottom: H - 55};
    const hidden = new Set(spec.hidden_series ?? []);
    const world = spec.world_lines ?? [];
    const series = (spec.series ?? []).filter(item => !hidden.has(item.name));
    const curves = spec.show_normal_curves === false ? [] :
      (spec.normal_curves ?? []).filter(item => !hidden.has(item.name));
    const is3d = Boolean(spec.mesh || spec.dimension === 3);
    const dimension = is3d ? 3 : 2;
    const baseProjection = projection(spec.projection, this.azimuth, this.elevation);
    let project = baseProjection;
    let xyzExtent;
    let fitPoints;
    if (spec.mesh) {
      xyzExtent = extent(meshPoints, 3);
      project = point => baseProjection(point.map((v, i) =>
        (v - xyzExtent[i][0]) / (xyzExtent[i][1] - xyzExtent[i][0]) * 2 - 1));
      const origin = xyzExtent.map(range => range[0]);
      fitPoints = meshPoints.map(project);
      fitPoints.push(project(origin), ...xyzExtent.map((range, axis) =>
        project(origin.map((value, i) => i === axis ? range[1] : value))));
    } else if (world.length) {
      const fullCar = world.filter(item => item.name === "physical");
      const physical = fullCar.length ? fullCar : world.filter(item => !item.construction && item.fit !== false);
      fitPoints = (physical.length ? physical : world).flatMap(item =>
        item.points.filter(point => valid(point, dimension)).map(project));
    } else {
      fitPoints = (series.length ? series : spec.series).flatMap(item => item.x.map((x, i) => [x, item.y[i]]));
      for (const curve of curves) fitPoints.push(...curve.x.map((x, i) => [x, curve.y[i]]));
      if (spec.bin_edges) fitPoints.push([spec.bin_edges[0], 0], [spec.bin_edges.at(-1), 0]);
      for (const point of spec.lap_points ?? []) fitPoints.push(point);
      for (const point of spec.fit_points ?? []) fitPoints.push(point);
    }
    const ranges = extent(fitPoints, 2);
    if (world.length && spec.projection !== "Iso") this.fittedRanges ??= ranges;
    const [[left, right], [bottom, top]] = fit(this.fittedRanges ?? ranges, Boolean(spec.equal || spec.mesh), bounds);
    const midX = (bounds.left + bounds.right) / 2;
    const midY = (bounds.bottom + bounds.top) / 2;
    const width = bounds.right - bounds.left;
    const height = bounds.bottom - bounds.top;
    const sx = x => midX + this.pan[0] + ((x - left) / (right - left) - 0.5) * width * this.zoom;
    const sy = y => midY + this.pan[1] - ((y - bottom) / (top - bottom) - 0.5) * height * this.zoom;
    const inverse = (x, y) => [
      left + ((x - midX - this.pan[0]) / (width * this.zoom) + 0.5) * (right - left),
      bottom + (0.5 - (y - midY - this.pan[1]) / (height * this.zoom)) * (top - bottom),
    ];
    const screen = ([x, y]) => [sx(x), sy(y)];
    const grid = spec.projection === "Iso" ? "" : axes(sx, sy, inverse, spec.xlabel, spec.ylabel, bounds, H);
    if (!spec.mesh) {
      this.readCursor = (x, y) => {
        if (x < bounds.left || x > bounds.right || y < bounds.top || y > bounds.bottom) return "";
        const [xx, yy] = inverse(x, y);
        if (!spec.bin_edges) return spec.projection === "Iso" ? "" : "x = " + amount(xx) + " · y = " + amount(yy);
        const index = spec.bin_edges.findIndex((value, i) => value <= xx && xx < spec.bin_edges[i+1]);
        if (index < 0 || !series.length) return "";
        const counts = series.map(item => item.name + " " + item.y[index]).join(" · ");
        const stats = curves.filter(curve => curve.count).map(curve =>
          curve.name + " μ " + amount(curve.mean_deg) + "°, σ " + amount(curve.std_deg) + "°");
        return [amount(spec.bin_edges[index]) + "° to " + amount(spec.bin_edges[index+1]) + "°",
          counts, ...stats].join(" · ");
      };
    }
    const labels = [...(spec.markers ?? []), ...(spec.annotations ?? [])];
    let data;
    let meshLabels = "";
    if (spec.mesh) {
      data = meshLayer(spec.mesh, project, screen, xyzExtent[2]);
      const start = xyzExtent.map(range => range[0]);
      meshLabels = [spec.xlabel, spec.ylabel, spec.zlabel].map((label, i) => {
        const end = [...start];
        end[i] = xyzExtent[i][1];
        const [x, y] = screen(project(end));
        const color = palette[i];
        const [mx, my] = screen(project(start));
        const tx = Math.max(12, Math.min(W - 12, x + (x < mx ? -8 : 8)));
        const ty = Math.max(bounds.top + 5, Math.min(H - 36, y + (y < my ? -8 : 18)));
        const ticks = [0, .5, 1].map(fraction => {
          const point = [...start];
          point[i] += (end[i] - start[i]) * fraction;
          const [px, py] = screen(project(point));
          return text(px + 4, py - 5, amount(point[i]), {color, size: 9});
        }).join("");
        return path(lineData([start, end].map(project), screen), color, 1.6) + ticks +
          text(tx, ty, label, {color, size: W < 560 ? 10 : 12, anchor: tx > W * .55 ? "end" : "start"});
      }).join("");
      meshLabels += text(bounds.right, H - 18, xyzExtent.map(([min, max]) => amount(min) + " to " + amount(max)).join("   /   "), {anchor: "end"});
    } else if (world.length) {
      data = world.map(item => '<g><title>' + xml(item.name) + '</title>' +
        path(lineData(item.points.map(point => valid(point, dimension) ? project(point) : null), screen),
          item.color ?? "#9bb2cd", item.width ?? 1.6, item.dash ?? []) + "</g>").join("");
      if (spec.projection === "Iso") {
        meshLabels = text(bounds.right, H - 17, "X forward · Y left · Z up · m", {anchor: "end"});
      }
    } else {
      data = seriesLayer(spec, series, curves, screen);
    }
    const title = spec.title ?? "Suspension geometry";
    const titleSize = Math.min(18, (W - 32) / Math.max(title.length * .54, 1));
    const guides = spec.mesh ? labels.filter(item => valid(item.position, 3)).map(({position}) => path(
      lineData([[position[0], position[1], xyzExtent[2][0]], position].map(project), screen), "#f8fafc", 1, [3, 4],
    )).join("") : "";
    this.svg = '<svg xmlns="http://www.w3.org/2000/svg" role="img" aria-label="' + xml(title) +
      '" viewBox="0 0 ' + W + " " + H + '" width="' + W + '" height="' + H +
      '" font-family="Segoe UI,system-ui,sans-serif"><title>' + xml(title) + '</title>' +
      '<rect width="100%" height="100%" fill="#111d31"/>' +
      '<defs><clipPath id="plot-clip"><rect x="' + bounds.left + '" y="' + bounds.top +
      '" width="' + width + '" height="' + height + '"/></clipPath></defs>' +
      text(15, 25, title, {color: "#f6f9ff", size: titleSize}) + grid +
      '<g clip-path="url(#plot-clip)">' + data + lapPoints(spec.lap_points, project, screen, dimension, spec.lap_colors) +
      guides + markers(labels, project, screen, dimension, bounds) + "</g>" + meshLabels +
      (spec.current_status ? text(W - 12, 42, spec.current_status, {color: "#fbbf24", anchor: "end", size: Math.min(12, (W - 24) / (spec.current_status.length * .55))}) : "") + "</svg>";
    this.host.innerHTML = this.svg;
    const worldNames = {front: "Front FV plane", rear: "Rear FV plane", sv: "SV plane", physical: "Physical geometry"};
    const keys = spec.series ? series.map((item, index) => [item.name, item.color ?? palette[index % palette.length]]) :
      world.map(item => [worldNames[item.name] ?? item.name, item.color ?? "#9bb2cd"]);
    if (spec.lap_points?.length) keys.push(["LapSim samples (" + (spec.lap_visible ?? spec.lap_points.length) + ")", "#fbbf24"]);
    const options = Array.from(new Map(keys)).filter(([name]) => name);
    this.legend.hidden = options.length === 0;
    this.legend.innerHTML = options.map(([name, color]) =>
      '<span><i aria-hidden="true" style="background:' + xml(color) + '"></i>' + xml(name) + "</span>").join("");
  }

  async png() {
    if (!this.svg) throw new Error("Wait for a chart before exporting.");
    const url = URL.createObjectURL(new Blob([this.svg], {type: "image/svg+xml;charset=utf-8"}));
    try {
      const picture = new Image();
      picture.src = url;
      await picture.decode();
      const canvas = document.createElement("canvas");
      canvas.width = picture.naturalWidth * 2;
      canvas.height = picture.naturalHeight * 2;
      canvas.getContext("2d").drawImage(picture, 0, 0, canvas.width, canvas.height);
      const blob = await new Promise(resolve => canvas.toBlob(resolve, "image/png"));
      if (!blob) throw new Error("Could not create the PNG.");
      return blob;
    } finally {
      URL.revokeObjectURL(url);
    }
  }
}
