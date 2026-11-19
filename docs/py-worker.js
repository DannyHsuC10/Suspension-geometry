/* global importScripts, loadPyodide */
const runtime = "https://cdn.jsdelivr.net/pyodide/v0.27.7/full/";
const files = [
  "lab_worker.py", "lab_formats.py", "tv_worker.py",
  ...[
    "__init__.py", "analysis.py", "anti.py", "caster_search.py", "default_project.json",
    "engine.py", "export.py", "fvsa_search.py", "hardpoints.py", "input_limits.py", "lap_data.py",
    "project.py", "project_import.py", "roll_balance.py", "roll_center_search.py", "step_solids.py",
    "svsa_search.py", "wheel_alignment.py", "wheel_end_geometry.py",
    "steering.py", "steering_chart.py", "tv_analysis.py", "tv_lap.py",
    "static/__init__.py", "static/analysis_settings.py", "static/car.py", "static/geometry.py",
    "static/Geometric_Analysis.py", "static/sv/__init__.py", "static/sv/side_view_geometry.py",
    ...[
      "__init__.py", "arm_mount_geometry.py", "arm_plane_geometry.py", "chassis_geometry.py",
      "circle_plane_intersections.py", "interference_geometry.py", "kingpin_geometry.py",
      "lower_arm_mount_geometry.py", "mount_position_geometry.py", "rear_arm_mount_geometry.py",
      "upper_arm_mount_geometry.py", "wheel_projection_geometry.py", "wireframe_geometry.py",
    ].map(name => "static/fv/" + name),
  ].map(name => "suspension_geometry_lab/" + name),
];

async function start() {
  const source = Promise.all(files.map(async name => {
    // Match the interface's release version for every solver source.
    const url = new URL(name, self.location.href);
    url.search = self.location.search;
    const response = await fetch(url);
    if (!response.ok) throw new Error("Could not load " + name + " (" + response.status + ").");
    return [name, new Uint8Array(await response.arrayBuffer())];
  }));
  const python = (async () => {
    importScripts(runtime + "pyodide.js");
    const instance = await loadPyodide({indexURL: runtime});
    await instance.loadPackage("numpy");
    return instance;
  })();
  const [sources, instance] = await Promise.all([source, python]);
  for (const [name, content] of sources) {
    const path = "/home/pyodide/" + name;
    instance.FS.mkdirTree(path.slice(0, path.lastIndexOf("/")));
    instance.FS.writeFile(path, content);
  }
  instance.globals.set("web_publish_progress", progress => self.postMessage({id: activeId, progress: JSON.parse(progress)}));
  instance.runPython(
    "import sys\nsys.path.insert(0, '/home/pyodide')\nimport lab_worker\n" +
    "lab_worker.publish_progress = web_publish_progress\nfrom lab_worker import dispatch",
  );
  return instance.globals.get("dispatch");
}

let ready;
let activeId;
self.addEventListener("message", async ({data: {id, action, payload}}) => {
  try {
    ready ??= start();
    const dispatch = await ready;
    activeId = id;
    self.postMessage({id, ...JSON.parse(dispatch(action, JSON.stringify(payload ?? {})))});
  } catch (error) {
    self.postMessage({id, ok: false, error: error.message});
  }
});
