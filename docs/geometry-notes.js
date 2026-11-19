const notes = {
  front: () => import("./geometry-fv.js").then(module => module.frontFV),
  rear: () => import("./geometry-fv.js").then(module => module.rearFV),
  sv: () => import("./geometry-sv-we.js").then(module => module.sideView),
  we: () => import("./geometry-sv-we.js").then(module => module.wheelEnd),
  static: () => import("./geometry-static.js").then(module => module.staticVehicle),
};
const katexBase = "https://cdn.jsdelivr.net/npm/katex@0.19.1/dist/";
let mathReady;

function loadAsset(file, integrity) {
  const isStylesheet = file.endsWith(".css");
  const asset = document.createElement(isStylesheet ? "link" : "script");
  asset.crossOrigin = "anonymous";
  asset.integrity = integrity;
  if (isStylesheet) {
    asset.rel = "stylesheet";
    asset.href = katexBase + file;
  } else {
    asset.src = katexBase + file;
  }
  return new Promise((resolve, reject) => {
    asset.addEventListener("load", resolve, {once: true});
    asset.addEventListener("error", () => reject(new Error("Could not load " + file)), {once: true});
    document.head.append(asset);
  });
}

function loadMath() {
  mathReady ??= Promise.all([
    loadAsset("katex-swap.min.css", "sha384-/YlNbK67U4WeyjZpwGJd2C4H3Lm8gq0m47PFlrmuKj9TDQzW6HbfSbWI6gpHx2ol"),
    loadAsset("katex.min.js", "sha384-a1WQdjRj162i159OYq041X6I4K8NenkrI09LlPmiHg0YOb7CR+wJOraXwK9abpnE")
      .then(() => loadAsset("contrib/auto-render.min.js", "sha384-bjyGPfbij8/NDKJhSGZNP/khQVgtHUE5exjm4Ydllo42FwIgYsdLO2lXGmRBf5Mz")),
  ]).then(() => window.renderMathInElement);
  return mathReady;
}

export function setupGeometryNotes(details, summary, content) {
  const articles = new Map();
  const openViews = new Set();
  let page;
  let reveal;

  async function renderMath(article) {
    try {
      const render = await loadMath();
      render(article, {throwOnError: false, trust: false});
    } catch {
      const status = document.createElement("p");
      status.className = "status";
      status.textContent = "The math renderer could not load. Equations are shown as LaTeX source.";
      article.prepend(status);
    }
  }

  async function populate(article, key) {
    try {
      article.innerHTML = await notes[key]();
      renderMath(article);
    } catch {
      article.textContent = "Could not load the geometry notes. Reload the page to try again.";
    }
  }

  function showArticle() {
    if (!page || !details.open) return;
    const key = page;
    let note = articles.get(key);
    if (!note) {
      const article = document.createElement("article");
      article.textContent = "Loading geometry notes…";
      note = {article, loaded: populate(article, key)};
      articles.set(key, note);
    }
    if (content.firstChild !== note.article) content.replaceChildren(note.article);
    if (reveal === key) {
      reveal = null;
      note.loaded.then(() => {
        if (details.open && page === key) details.scrollIntoView({block: "start"});
      });
    }
  }

  details.addEventListener("toggle", showArticle);
  summary.addEventListener("click", () => { reveal = details.open ? null : page; });
  return (next, title) => {
    if (next === page) return;
    if (details.open && page) openViews.add(page);
    else openViews.delete(page);
    page = next;
    reveal = null;
    summary.textContent = "Geometry notes · " + title;
    content.replaceChildren();
    details.open = openViews.has(page);
    showArticle();
  };
}
