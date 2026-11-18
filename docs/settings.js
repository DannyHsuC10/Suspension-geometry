const AXLES = ['front', 'rear'];
const CORNERS = ['FL', 'FR', 'RL', 'RR'];
const MIRROR = { FL: 'FR', FR: 'FL', RL: 'RR', RR: 'RL' };
const GROUPS = [
  ['vehicle', 'Vehicle and wheels'], ['front', 'Front axle'], ['rear', 'Rear axle'],
  ['hardpoints', 'Hardpoints'], ['analysis', 'Sweep ranges'],
];
const NUMBER = /^[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?$/i;

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function inputNumber(value, label) {
  const input = element('input');
  input.type = 'text';
  input.inputMode = 'decimal';
  input.autocomplete = 'off';
  input.spellcheck = false;
  input.value = value === null ? '' : String(value);
  input.setAttribute('aria-label', label);
  return input;
}

function getNumber(input, label, optional = false) {
  const text = input.value.trim();
  if (optional && !text) return null;
  if (!NUMBER.test(text) || !Number.isFinite(Number(text))) {
    input.setAttribute('aria-invalid', 'true');
    throw new Error(`Enter a finite number for ${label}.`);
  }
  return Number(text);
}

function settingField(label, input, hint) {
  const field = element('label', 'project-field');
  const description = element('small', 'project-hint', hint);
  field.append(element('span', 'project-field-label', label), input, description);
  return [field, description];
}

function button(label, className = '') {
  const control = element('button', className, label);
  control.type = 'button';
  return control;
}

function createDialog(id, title, summary) {
  const dialog = element('dialog', 'project-dialog');
  dialog.id = id;
  dialog.setAttribute('aria-labelledby', `${id}-title`);
  const header = element('header', 'project-dialog-header');
  const copy = element('div');
  const heading = element('h2', '', title);
  heading.id = `${id}-title`;
  copy.append(heading, element('p', '', summary));
  const close = button('×', 'project-close');
  close.setAttribute('aria-label', 'Close');
  close.addEventListener('click', () => dialog.close());
  header.append(copy, close);
  const body = element('div', 'project-dialog-body');
  const error = element('div', 'project-error');
  error.setAttribute('role', 'alert');
  error.hidden = true;
  const footer = element('footer', 'project-dialog-footer');
  const cancel = button('Cancel');
  cancel.addEventListener('click', () => dialog.close());
  footer.append(cancel);
  dialog.append(header, body, error, footer);
  document.body.append(dialog);
  return { dialog, body, footer, error };
}

function displayError(region, error) {
  region.hidden = !error;
  region.textContent = error ? (error.message || String(error)) : '';
}

function formatValue(value) {
  return value == null ? 'N/A' : Number(value.toPrecision(7)).toString();
}

function numberInRange(value, field) {
  if (field.minimum !== null && (value < field.minimum || (field.openMinimum && value === field.minimum))) return false;
  if (field.maximum !== null && (value > field.maximum || (field.openMaximum && value === field.maximum))) return false;
  if (field.integer && !Number.isInteger(value)) return false;
  return Math.abs(value) >= field.minimumMagnitude;
}

function numericFieldValue(input, field) {
  const value = getNumber(input, field.label);
  if (!numberInRange(value, field)) {
    input.setAttribute('aria-invalid', 'true');
    throw new Error(`${field.label}. ${field.hint}`);
  }
  input.removeAttribute('aria-invalid');
  return value;
}

function populateDerived(table, metadata, derived) {
  table.replaceChildren();
  const head = table.createTHead().insertRow();
  for (const text of ['Parameter', 'Front', 'Rear']) head.append(element('th', '', text));
  const body = table.createTBody();
  for (const [key, field] of Object.entries(metadata.derivedFields)) {
    const row = body.insertRow();
    const label = element('th', '', field.unit ? `${field.label} (${field.unit})` : field.label);
    label.scope = 'row';
    row.append(label);
    for (const axle of AXLES) row.insertCell().textContent = formatValue(derived?.[axle]?.[key]);
  }
}

function buildSettings(view, metadata, base, request, commit) {
  const { dialog, body, footer, error } = view;
  const sections = {};
  const sectionButtons = {};
  const inputs = [];
  const mountInputs = [];
  const pointInputs = {};
  let points = structuredClone(metadata.points);
  let pointsEdited = Boolean(base.hardpoints);
  let activeSection = 'vehicle';
  let previewRevision = 0;
  let previewTimer;

  const name = element('input');
  name.value = base.name;
  name.maxLength = 128;
  name.required = true;
  name.autofocus = true;
  const [nameField] = settingField('Project name', name, '1–128 characters.');
  const mode = element('select');
  for (const [value, title] of [['parameter', 'Design parameters'], ['hardpoint', 'Direct hardpoint coordinates']]) {
    const option = element('option', '', title);
    option.value = value;
    mode.append(option);
  }
  mode.value = base.hardpoints?.enabled ? 'hardpoint' : 'parameter';
  const [modeField] = settingField('Geometry mode', mode, 'Both modes share vehicle, static camber and sweep settings.');
  const general = element('div', 'project-general');
  general.append(nameField, modeField);

  const tabs = element('div', 'project-tabs');
  tabs.setAttribute('role', 'tablist');
  tabs.setAttribute('aria-label', 'Vehicle settings groups');
  const sectionsHost = element('div', 'project-sections');
  for (const [key, label] of GROUPS) {
    const tab = button(label);
    tab.id = `project-tab-${key}`;
    tab.setAttribute('role', 'tab');
    tab.setAttribute('aria-controls', `project-panel-${key}`);
    tab.addEventListener('click', () => selectSection(key));
    tab.addEventListener('keydown', event => {
      const index = GROUPS.findIndex(([group]) => group === key);
      const offset = event.key === 'ArrowLeft' ? -1 : 1;
      if (event.key !== 'ArrowLeft' && event.key !== 'ArrowRight') return;
      event.preventDefault();
      const next = GROUPS[(index + offset + GROUPS.length) % GROUPS.length][0];
      selectSection(next);
      sectionButtons[next].focus();
    });
    tabs.append(tab);
    sectionButtons[key] = tab;
    const section = element('section', 'project-section');
    section.id = `project-panel-${key}`;
    section.setAttribute('role', 'tabpanel');
    section.setAttribute('aria-labelledby', tab.id);
    sectionsHost.append(section);
    sections[key] = section;
  }
  body.append(general, tabs, sectionsHost);

  for (const [group, fields] of Object.entries(metadata.fields)) {
    const grid = element('div', 'project-field-grid');
    sections[group].append(grid);
    for (const field of fields) {
      const label = field.unit ? `${field.label} (${field.unit})` : field.label;
      const input = inputNumber(base[group][field.key], label);
      const [node, hint] = settingField(label, input, field.hint);
      const descriptor = { group, field, input, hint };
      inputs.push(descriptor);
      input.addEventListener('input', () => {
        try { numericFieldValue(input, field); } catch { /* Allow unfinished number input. */ }
        schedulePreview();
      });
      grid.append(node);
    }
  }

  for (const axle of AXLES) {
    const grid = element('div', 'project-field-grid');
    sections[axle].append(element('h3', '', 'Chassis arm mounts'), element('p', 'project-explanation',
      'X increases forward. Leave a mount blank to use its calculated rim-clearance limit. The rear X must be less than the front X on each arm.'), grid);
    for (const [key, label] of Object.entries(metadata.mountFields)) {
      const stored = metadata.mounts[axle][key];
      const input = inputNumber(stored === null ? null : stored * 1000, `${label} (mm)`);
      input.placeholder = 'Automatic';
      const [node, hint] = settingField(`${label} (mm)`, input, 'Leave blank for automatic placement.');
      mountInputs.push({ axle, key, label, input, hint, stored, original: input.value });
      input.addEventListener('input', () => {
        input.removeAttribute('aria-invalid');
        schedulePreview();
      });
      grid.append(node);
    }
  }

  const pointActions = element('div', 'project-point-actions');
  const regenerate = button('Regenerate from design parameters');
  pointActions.append(regenerate);
  const hardpointSection = sections.hardpoints;
  hardpointSection.append(element('p', 'project-explanation', 'Coordinates are in millimetres. X points forward, Y left and Z up. The front axle is at X = 0. Editing one side reflects the same point across Y = 0.'), pointActions);
  const pointGrid = element('div', 'project-point-grid');
  for (const corner of CORNERS) {
    pointInputs[corner] = {};
    const group = element('section', 'project-point-group');
    const table = element('table', 'project-point-table');
    const columns = table.createTHead().insertRow();
    for (const label of ['Hardpoint', 'X', 'Y', 'Z']) columns.append(element('th', '', label));
    const rows = table.createTBody();
    group.append(element('h3', '', { FL: 'Front left', FR: 'Front right', RL: 'Rear left', RR: 'Rear right' }[corner]), table);
    for (const [key, label] of Object.entries(metadata.pointLabels)) {
      const row = rows.insertRow();
      const heading = element('th', '', key);
      heading.title = label;
      heading.scope = 'row';
      row.append(heading);
      pointInputs[corner][key] = [];
      [...'XYZ'].forEach((axis, index) => {
        const input = inputNumber(points[corner][key][index] * 1000, `${corner} ${label} ${axis} (mm)`);
        pointInputs[corner][key].push(input);
        row.insertCell().append(input);
        input.addEventListener('input', () => {
          pointsEdited = true;
          input.removeAttribute('aria-invalid');
          const text = input.value.trim();
          const opposite = pointInputs[MIRROR[corner]][key][index];
          opposite.removeAttribute('aria-invalid');
          opposite.value = index === 1 && NUMBER.test(text) ? String(-Number(text)) : text;
          schedulePreview();
        });
      });
    }
    pointGrid.append(group);
  }
  const derived = element('table', 'project-derived-table');
  hardpointSection.append(pointGrid, element('h3', '', 'Derived axle geometry'), derived);
  const previewMessage = element('p', 'project-explanation');
  previewMessage.setAttribute('aria-live', 'polite');
  hardpointSection.append(previewMessage);
  populateDerived(derived, metadata, metadata.derived);
  setMountHints(metadata.mountChecks);

  const apply = button('Apply vehicle settings', 'project-primary');
  footer.append(apply);
  apply.addEventListener('click', async () => {
    displayError(error);
    try {
      const project = candidate(false);
      apply.disabled = true;
      const result = await request('project-apply', { project, base });
      if (!dialog.open) return;
      await commit(result.project);
      dialog.close();
    } catch (failure) {
      displayError(error, failure);
      const invalid = body.querySelector('[aria-invalid="true"]');
      if (invalid) {
        const section = invalid.closest('[role="tabpanel"]');
        if (section && section.hidden) selectSection(section.id.replace('project-panel-', ''));
        invalid.focus();
      }
    } finally {
      apply.disabled = false;
    }
  });

  mode.addEventListener('change', async () => {
    if (mode.value === 'hardpoint') {
      selectSection('hardpoints');
      if (!pointsEdited) await regeneratePoints(true);
    }
    changeMode();
    schedulePreview();
  });
  regenerate.addEventListener('click', () => regeneratePoints(false));
  dialog.addEventListener('close', () => {
    previewRevision++;
    clearTimeout(previewTimer);
  });
  selectSection('vehicle');
  changeMode();

  function selectSection(key) {
    activeSection = key;
    for (const [group] of GROUPS) {
      sections[group].hidden = group !== key;
      sectionButtons[group].setAttribute('aria-selected', String(group === key));
      sectionButtons[group].tabIndex = group === key ? 0 : -1;
    }
    schedulePreview();
  }

  function changeMode() {
    const direct = mode.value === 'hardpoint';
    for (const { group, field, input, hint } of inputs) {
      input.disabled = direct && AXLES.includes(group) && field.key !== 'static_camber_deg';
      if (input.disabled) input.removeAttribute('aria-invalid');
      hint.textContent = input.disabled ? 'Design value retained for parameter mode.' : field.hint;
    }
    for (const { input } of mountInputs) input.disabled = direct;
    for (const corner of Object.values(pointInputs)) {
      for (const coordinates of Object.values(corner)) {
        for (const input of coordinates) input.disabled = !direct || regenerate.disabled;
      }
    }
  }

  function readPoints() {
    const result = structuredClone(points);
    for (const [corner, entries] of Object.entries(pointInputs)) {
      for (const [key, coordinates] of Object.entries(entries)) {
        for (const [index, input] of coordinates.entries()) {
          const stored = points[corner][key][index];
          const label = `${corner} ${metadata.pointLabels[key]} ${'XYZ'[index]}`;
          result[corner][key][index] = input.value === String(stored * 1000) ? stored : getNumber(input, label) / 1000;
        }
      }
    }
    return result;
  }

  function candidate(includePoints, forceParameters = false) {
    const project = structuredClone(base);
    project.name = name.value.trim();
    const direct = mode.value === 'hardpoint' && !forceParameters;
    for (const { group, field, input } of inputs) {
      project[group][field.key] = numericFieldValue(input, field);
    }
    project.arm_mount_x = structuredClone(metadata.mounts);
    for (const { axle, key, label, input, stored, original } of mountInputs) {
      const value = getNumber(input, label, true);
      project.arm_mount_x[axle][key] = input.value === original ? stored : value === null ? null : value / 1000;
    }
    if (includePoints || direct || base.hardpoints || pointsEdited) {
      project.hardpoints = { enabled: direct, corners: forceParameters ? points : readPoints() };
    }
    return project;
  }

  function setPoints(next) {
    points = structuredClone(next);
    for (const [corner, entries] of Object.entries(pointInputs)) {
      for (const [key, coordinates] of Object.entries(entries)) {
        for (const [index, input] of coordinates.entries()) {
          input.value = String(points[corner][key][index] * 1000);
          input.removeAttribute('aria-invalid');
        }
      }
    }
  }

  function setMountHints(checks) {
    for (const { axle, key, hint, input } of mountInputs) {
      const check = checks?.[axle]?.[key];
      if (!check) continue;
      const label = check.limit_min_x === null ? '' :
        `${formatValue(check.limit_min_x * 1000)} to ${formatValue(check.limit_max_x * 1000)} mm. Actual X ${formatValue(check.actual_x * 1000)} mm.`;
      hint.textContent = [label, check.message].filter(Boolean).join(' ');
      if (check.fallback && check.requested_x !== null) input.setAttribute('aria-invalid', 'true');
      else input.removeAttribute('aria-invalid');
    }
  }

  function schedulePreview() {
    const revision = ++previewRevision;
    clearTimeout(previewTimer);
    if (!['front', 'rear', 'hardpoints'].includes(activeSection)) return;
    previewTimer = setTimeout(async () => {
      if (!dialog.open || regenerate.disabled) return;
      try {
        const project = candidate(true);
        const activeMode = mode.value;
        const result = await request('project-preview', { project, mode: activeMode, base });
        if (!dialog.open || revision !== previewRevision) return;
        setMountHints(result.mountChecks);
        if (activeMode === 'parameter' && !pointsEdited) setPoints(result.points);
        populateDerived(derived, metadata, result.derived);
        previewMessage.textContent = 'Geometry preview calculated.';
      } catch (failure) {
        if (!dialog.open || revision !== previewRevision) return;
        previewMessage.textContent = failure.message || String(failure);
      }
    }, 350);
  }

  async function regeneratePoints(switching) {
    previewRevision++;
    clearTimeout(previewTimer);
    regenerate.disabled = true;
    mode.disabled = true;
    apply.disabled = true;
    changeMode();
    try {
      const result = await request('project-preview', { project: candidate(true, true), mode: 'parameter', base });
      if (!dialog.open) return;
      setPoints(result.points);
      pointsEdited = true;
      displayError(error);
      populateDerived(derived, metadata, result.derived);
      setMountHints(result.mountChecks);
      previewMessage.textContent = 'Hardpoints regenerated from the current parameter inputs.';
    } catch (failure) {
      if (switching) mode.value = 'parameter';
      if (dialog.open) displayError(error, failure);
    } finally {
      regenerate.disabled = false;
      mode.disabled = false;
      apply.disabled = false;
      changeMode();
    }
  }
}

function showRepair(result, request, commit) {
  const view = createDialog('project-repair', 'Complete project import', 'Fill in the missing fields or use the displayed defaults.');
  const grid = element('div', 'project-field-grid');
  view.body.append(grid);
  const inputs = result.missing.map(field => {
    let input;
    if (typeof field.default === 'boolean') {
      input = element('select');
      for (const [value, title] of [['', `Default (${field.default ? 'Yes' : 'No'})`], ['true', 'Yes'], ['false', 'No']]) {
        const option = element('option', '', title);
        option.value = value;
        input.append(option);
      }
    } else {
      input = typeof field.default === 'string' ? element('input') : inputNumber(null, field.label);
      input.placeholder = field.default === null ? 'Automatic' : String(field.displayDefault);
    }
    const label = field.unit ? `${field.label} (${field.unit})` : field.label;
    grid.append(settingField(label, input, field.location)[0]);
    return input;
  });
  const apply = button('Complete import', 'project-primary');
  view.footer.append(apply);
  apply.addEventListener('click', async () => {
    apply.disabled = true;
    displayError(view.error);
    try {
      const values = inputs.map(input => input.value);
      const repaired = await request('project-repair', { ...result, values });
      if (!view.dialog.open) return;
      await commit(repaired.project);
      view.dialog.close();
    } catch (error) {
      displayError(view.error, error);
    } finally {
      apply.disabled = false;
    }
  });
  view.dialog.addEventListener('close', () => view.dialog.remove(), { once: true });
  view.dialog.showModal();
}

export function downloadFile(content, filename, mime) {
  const blob = content instanceof Blob ? content : new Blob([content], { type: mime });
  const url = URL.createObjectURL(blob);
  const link = element('a');
  link.href = url;
  link.download = filename;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 60_000);
}

export function setupProjectSettings({ request, getProject, onProject, showStatus }) {
  let openDialog;
  let opening = false;

  async function openSettings() {
    const base = getProject();
    if (!base || opening || openDialog?.open) return;
    opening = true;
    try {
      const metadata = await request('project-prepare', { project: base });
      const view = createDialog('vehicle-settings', 'Vehicle settings', 'Shared vehicle dimensions and suspension geometry for all five views.');
      openDialog = view.dialog;
      view.dialog.addEventListener('close', () => {
        view.dialog.remove();
        openDialog = null;
      }, { once: true });
      buildSettings(view, metadata, metadata.project, request, async project => {
        await onProject(project);
        showStatus('Vehicle settings applied.');
      });
      view.dialog.showModal();
    } catch (error) {
      showStatus(error.message || String(error), true);
    } finally {
      opening = false;
    }
  }

  async function importProject(file) {
    if (!file) return;
    try {
      const result = await request('project-import', { text: await file.text() });
      const commit = async project => {
        await onProject(project);
        showStatus(`Imported ${file.name}.`);
      };
      if (result.missing.length) showRepair(result, request, commit);
      else await commit(result.project);
    } catch (error) {
      showStatus(`Import failed. ${error.message || String(error)}`, true);
    }
  }

  function downloadProject() {
    const project = getProject();
    if (!project) return;
    const name = project.name.trim().replace(/[^\p{L}\p{N}._-]+/gu, '_').replace(/^\.+|\.+$/g, '') || 'suspension_geometry';
    downloadFile(`${JSON.stringify(project, null, 2)}\n`, `${name}.json`, 'application/json;charset=utf-8');
    showStatus('Project JSON downloaded.');
  }

  return { openSettings, importProject, downloadProject };
}
