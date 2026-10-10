const MODES = Object.freeze({
  human: { label: "Human vs Human", ai: [] },
  "player-v-ai": { label: "Blue vs Jev", ai: ["red"] },
  "ai-v-ai": { label: "Jev vs Jev", ai: ["blue", "red"] },
});

function trimEndpoint(value) {
  return String(value || "").trim().replace(/\/+$/, "");
}

export class JevPreviewController {
  constructor({ fetchImpl = globalThis.fetch?.bind(globalThis), endpoint = "", timeoutMs = 25000,
                documentRef = globalThis.document } = {}) {
    this.fetchImpl = fetchImpl;
    this.endpoint = trimEndpoint(endpoint);
    this.timeoutMs = timeoutMs;
    this.document = documentRef;
    this.mode = "human";
    this.paused = false;
    this.fallback = false;
    this.generation = 1;
    this.modules = new Map();
    this.inFlight = new Map();
    this.lastError = "";
  }

  registerModule(player, module) {
    this.modules.set(player, module);
    if (!Array.isArray(module.tbgwafAiInbox)) module.tbgwafAiInbox = [];
    this.#configureModule(player, module);
  }

  setEndpoint(endpoint) {
    this.endpoint = trimEndpoint(endpoint);
    this.cancelAll();
    this.generation += 1;
    this.#syncModules();
    this.retry();
  }

  setMode(mode) {
    if (!MODES[mode]) throw new Error(`Unknown preview mode: ${mode}`);
    this.mode = mode;
    this.fallback = false;
    this.paused = mode === "ai-v-ai";
    this.restart();
  }

  restart() {
    this.cancelAll();
    this.generation += 1;
    this.lastError = "";
    for (const [player, module] of this.modules) {
      this.#configureModule(player, module);
      module.tbgwafRestart = true;
      module.tbgwafRetryAI = true;
      module.tbgwafAiInbox.length = 0;
    }
    this.#render();
  }

  start() {
    if (this.mode !== "ai-v-ai") return;
    this.paused = false;
    this.generation += 1;
    this.#syncModules();
    this.retry();
  }

  pause() {
    if (this.mode !== "ai-v-ai") return;
    this.paused = true;
    this.cancelAll();
    this.generation += 1;
    this.#syncModules();
    this.#render();
  }

  retry() {
    this.cancelAll();
    this.generation += 1;
    this.lastError = "";
    for (const [player, module] of this.modules) {
      module.tbgwafAiInbox.length = 0;
      this.#configureModule(player, module);
      module.tbgwafRetryAI = true;
    }
    this.#render();
  }

  useFallback() {
    this.cancelAll();
    this.fallback = true;
    this.paused = false;
    this.lastError = "";
    this.generation += 1;
    this.#syncModules();
    this.retry();
  }

  cancelAll() {
    for (const pending of this.inFlight.values()) pending.abort.abort();
    this.inFlight.clear();
  }

  async requestDecision(player, rawPayload) {
    if (!this.#isAiPlayer(player) || this.paused || this.fallback) return;
    const module = this.modules.get(player);
    if (!module || this.inFlight.has(player)) return;

    let payload;
    try {
      payload = JSON.parse(rawPayload);
    } catch {
      this.#fail(player, "invalid local request", "unknown");
      return;
    }
    const requestId = payload.requestId;
    const generation = this.generation;
    if (!this.endpoint) {
      this.#fail(player, "Jev proxy URL is not configured", requestId);
      return;
    }
    if (!this.fetchImpl) {
      this.#fail(player, "Fetch is unavailable", requestId);
      return;
    }

    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(new Error("client timeout")), this.timeoutMs);
    this.inFlight.set(player, { abort, requestId, generation });
    module.tbgwafAiState = "thinking";
    this.#render();
    try {
      const response = await this.fetchImpl(`${this.endpoint}/v1/choice`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: rawPayload,
        signal: abort.signal,
        credentials: "omit",
      });
      const body = await response.json().catch(() => ({}));
      if (!response.ok) throw new Error(body.error || `proxy returned HTTP ${response.status}`);
      if (generation !== this.generation || !this.#isAiPlayer(player) || this.paused) return;
      const choices = body.choices;
      const figures = new Set(payload.candidates?.map((candidate) => candidate.figure));
      const legal = choices && typeof choices === "object" &&
        Object.keys(choices).length === figures.size &&
        [...figures].every((figure) => payload.candidates.some((candidate) =>
          candidate.figure === figure && candidate.id === choices[figure]));
      if (!legal) throw new Error("proxy returned choices outside the legal candidate set");
      module.tbgwafAiInbox.push(`D ${requestId} ${Object.values(choices).join(",")}`);
      module.tbgwafAiState = "idle";
    } catch (error) {
      if (generation !== this.generation || (abort.signal.aborted && this.paused)) return;
      const message = abort.signal.aborted ? "Jev request timed out" : String(error?.message || error);
      this.#fail(player, message, requestId);
    } finally {
      clearTimeout(timer);
      const current = this.inFlight.get(player);
      if (current?.requestId === requestId) this.inFlight.delete(player);
      this.#render();
    }
  }

  reportMatch(player, phase, round, winner) {
    if (player !== "blue") return;
    const outcome = this.document?.getElementById("outcome");
    if (!outcome) return;
    if (phase !== "game-over") {
      outcome.textContent = `Round ${round}`;
    } else if (winner === 0) {
      outcome.textContent = "Match over: Blue wins.";
    } else if (winner === 1) {
      outcome.textContent = "Match over: Red wins.";
    } else {
      outcome.textContent = "Match over: draw.";
    }
  }

  reportLocalError(player, message) {
    const module = this.modules.get(player);
    if (module) module.tbgwafAiState = "error";
    this.lastError = `${player === "blue" ? "Blue" : "Red"} AI: ${message}`;
    this.#render();
  }

  #fail(player, message, requestId) {
    const module = this.modules.get(player);
    if (module && requestId) module.tbgwafAiInbox.push(`E ${requestId}`);
    if (module) module.tbgwafAiState = "error";
    this.lastError = `${player === "blue" ? "Blue" : "Red"} Jev: ${message}`;
    this.#render();
  }

  #isAiPlayer(player) { return MODES[this.mode].ai.includes(player); }

  #configureModule(player, module) {
    module.tbgwafAI = this.#isAiPlayer(player);
    module.tbgwafAIPaused = this.paused;
    module.tbgwafAIFallback = this.fallback;
    module.tbgwafAutoCommit = this.mode !== "human";
    module.tbgwafAIGeneration = this.generation;
  }

  #syncModules() {
    for (const [player, module] of this.modules) this.#configureModule(player, module);
    this.#render();
  }

  #render() {
    if (!this.document) return;
    const spec = MODES[this.mode];
    const modeLabel = this.document.getElementById("mode-label");
    if (modeLabel) modeLabel.textContent = `Mode: ${spec.label}`;
    for (const button of this.document.querySelectorAll("[data-mode]")) {
      button.setAttribute("aria-pressed", String(button.dataset.mode === this.mode));
    }
    const start = this.document.getElementById("start-ai");
    const pause = this.document.getElementById("pause-ai");
    if (start) start.disabled = this.mode !== "ai-v-ai" || !this.paused;
    if (pause) pause.disabled = this.mode !== "ai-v-ai" || this.paused;
    const retry = this.document.getElementById("retry-ai");
    const fallback = this.document.getElementById("fallback-ai");
    if (retry) retry.hidden = !this.lastError;
    if (fallback) fallback.hidden = !this.lastError || this.fallback;
    const status = this.document.getElementById("ai-status");
    if (!status) return;
    if (this.mode === "human") {
      status.textContent = "Human-controlled match.";
      status.dataset.kind = "normal";
    } else if (this.fallback) {
      status.textContent = "Deterministic fallback active — this is not live Jev.";
      status.dataset.kind = "fallback";
    } else if (this.lastError) {
      status.textContent = this.lastError;
      status.dataset.kind = "error";
    } else if (this.paused) {
      status.textContent = "AI paused.";
      status.dataset.kind = "normal";
    } else if (this.inFlight.size) {
      status.textContent = `Jev thinking (${[...this.inFlight.keys()].join(", ")})…`;
      status.dataset.kind = "normal";
    } else {
      status.textContent = "Live Jev ready.";
      status.dataset.kind = "normal";
    }
  }
}

const DEFAULT_JEV_ENDPOINT = "https://tbgwaf-jev-adapter.fly.dev";

function initialConfiguration() {
  const params = new URLSearchParams(window.location.search);
  const storedEndpoint = window.localStorage.getItem("tbgwafJevProxy") || "";
  return {
    mode: MODES[params.get("mode")] ? params.get("mode") : "human",
    endpoint: params.get("jev") || window.TBGWAF_JEV_PROXY_URL || storedEndpoint || DEFAULT_JEV_ENDPOINT,
  };
}

function bootstrap() {
  const config = initialConfiguration();
  const controller = new JevPreviewController({ endpoint: config.endpoint });
  window.tbgwafJevPreview = controller;
  const endpointInput = document.getElementById("jev-endpoint");
  endpointInput.value = controller.endpoint;

  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => controller.setMode(button.dataset.mode));
  });
  document.getElementById("start-ai").addEventListener("click", () => controller.start());
  document.getElementById("pause-ai").addEventListener("click", () => controller.pause());
  document.getElementById("restart-match").addEventListener("click", () => controller.restart());
  document.getElementById("retry-ai").addEventListener("click", () => controller.retry());
  document.getElementById("fallback-ai").addEventListener("click", () => controller.useFallback());
  document.getElementById("save-endpoint").addEventListener("click", () => {
    const value = trimEndpoint(endpointInput.value);
    window.localStorage.setItem("tbgwafJevProxy", value);
    controller.setEndpoint(value);
  });

  const status = document.getElementById("status");
  window.onerror = () => { status.textContent = "Failed to load — check the browser console."; };
  const bus = { inboxes: [] };
  let pending = 2;
  for (const player of ["blue", "red"]) {
    const pane = document.getElementById(`pane-${player}`);
    const moduleConfig = {
      canvas: pane.querySelector("canvas"),
      tbgwafPlayer: player,
      tbgwafBus: bus,
      tbgwafRequestDecision: (payload) => controller.requestDecision(player, payload),
      tbgwafReportMatch: (phase, round, winner) =>
        controller.reportMatch(player, phase, round, winner),
      tbgwafReportAiError: (message) => controller.reportLocalError(player, message),
      onRuntimeInitialized() {
        controller.registerModule(player, moduleConfig);
        if (--pending === 0) status.textContent = "";
      },
    };
    createTbgwafModule(moduleConfig);
  }
  controller.setMode(config.mode);
}

if (typeof window !== "undefined" && typeof document !== "undefined") bootstrap();

export { MODES };
