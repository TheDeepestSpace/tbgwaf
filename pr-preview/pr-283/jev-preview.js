import { matchOptions } from "./maps.js";
const MODES = Object.freeze({
  human: { label: "Human vs Human", ai: [] },
  "player-v-ai": { label: "Blue vs Jev", ai: ["red"] },
  "ai-v-ai": { label: "Jev vs Jev", ai: ["blue", "red"] },
});

function trimEndpoint(value) {
  return String(value || "").trim().replace(/\/+$/, "");
}

export class JevPreviewController {
  // serverMode: the page talks to the authoritative game server, which hosts
  // Jev itself in the Blue vs Jev / Jev vs Jev modes (it builds the requests,
  // calls the Jev adapter and plans for the Jev teams). This controller then
  // only mirrors the server's status and relays Start/Pause/Retry/Fallback.
  constructor({ fetchImpl = globalThis.fetch?.bind(globalThis), endpoint = "", timeoutMs = 25000,
                documentRef = globalThis.document,
                serverMode = Boolean(globalThis.TBGWAF_SERVER) } = {}) {
    this.fetchImpl = fetchImpl;
    this.endpoint = trimEndpoint(endpoint);
    this.timeoutMs = timeoutMs;
    this.document = documentRef;
    this.mode = "human";
    this.paused = false;
    this.fallback = false;
    this.generation = 1;
    this.ctf = null;  // null: per-mode default; true/false: chosen on the splash.
    this.modules = new Map();
    this.inFlight = new Map();
    this.lastError = "";
    this.serverMode = serverMode;
    this.serverThinking = [];
    this.sockets = new Map();
  }

  get serverBots() { return this.serverMode && this.mode !== "human"; }

  // Server Jev modes: watch the module's WebSocket for the server's `bots`
  // status. The socket opens after the WASM runtime starts, so poll for it.
  watchServer(module) {
    const attach = () => {
      const socket = module.tbgwafWs;
      if (!socket || this.sockets.get(module) === socket) return Boolean(socket);
      this.sockets.set(module, socket);
      socket.addEventListener("message", (event) => {
        if (typeof event.data !== "string" || !event.data.includes('"bots"')) return;
        try {
          const message = JSON.parse(event.data);
          if (message.t === "bots") this.applyBotStatus(message);
        } catch { /* not ours */ }
      });
      return true;
    };
    if (attach()) return;
    const timer = setInterval(() => { if (attach()) clearInterval(timer); }, 100);
  }

  applyBotStatus(message) {
    this.paused = message.mode === "ai-v-ai" && message.running === false;
    this.serverThinking = Array.isArray(message.thinking) ? message.thinking : [];
    this.lastError = typeof message.error === "string" ? message.error : "";
    this.fallback = Boolean(message.fallback);
    this.#render();
  }

  #sendControl(control) {
    for (const module of this.sockets.keys()) {
      const socket = module.tbgwafWs;
      if (socket && socket.readyState === 1) {
        socket.send(JSON.stringify({ t: "bot_control", ...control }));
        return;
      }
    }
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
    this.serverThinking = [];
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
    if (this.serverBots) { this.#sendControl({ run: true }); return; }
    this.paused = false;
    this.generation += 1;
    this.#syncModules();
    this.retry();
  }

  pause() {
    if (this.mode !== "ai-v-ai") return;
    if (this.serverBots) { this.#sendControl({ run: false }); return; }
    this.paused = true;
    this.cancelAll();
    this.generation += 1;
    this.#syncModules();
    this.#render();
  }

  retry() {
    if (this.serverBots) { this.#sendControl({ retry: true }); return; }
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
    if (this.serverBots) { this.#sendControl({ fallback: true }); return; }
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

  // In-browser planning only; with server-hosted Jev the server plans.
  #isAiPlayer(player) { return !this.serverBots && MODES[this.mode].ai.includes(player); }

  #configureModule(player, module) {
    module.tbgwafAI = this.#isAiPlayer(player);
    module.tbgwafAIPaused = this.paused;
    module.tbgwafAIFallback = this.fallback;
    module.tbgwafDefaultCtf = this.ctf ?? (this.mode === "ai-v-ai" && !this.serverBots);
    // Jev vs Jev: the Blue instance renders every figure and both FOVs in one
    // pane; the Red instance keeps simulating off-screen. On a server both
    // Jev clients are fog-limited, so a separate spectator client takes that pane.
    module.tbgwafSpectator =
      this.mode === "ai-v-ai" && player === "blue" && !globalThis.TBGWAF_SERVER;
    // Only with no human player; Blue vs Jev needs a manual Commit round.
    module.tbgwafAutoCommit = this.mode === "ai-v-ai" && !this.serverBots;
    module.tbgwafAIGeneration = this.generation;
  }

  #syncModules() {
    for (const [player, module] of this.modules) this.#configureModule(player, module);
    this.#render();
  }

  #render() {
    if (!this.document) return;
    const spec = MODES[this.mode];
    this.document.getElementById("panes")?.setAttribute("data-spectator", String(this.mode === "ai-v-ai"));
    const spectatorPane = this.mode === "ai-v-ai" && Boolean(globalThis.TBGWAF_SERVER);
    this.document.getElementById("panes")?.setAttribute("data-spectator-pane", String(spectatorPane));
    const spectatorEl = this.document.getElementById("pane-spectator");
    if (spectatorEl) spectatorEl.hidden = !spectatorPane;
    // With server-hosted Jev the server owns the Jev connection: no proxy to configure.
    const config = this.document.getElementById("config");
    if (config) config.hidden = this.serverBots;
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
    } else if (this.inFlight.size || this.serverThinking.length) {
      const thinking = [...this.inFlight.keys(), ...this.serverThinking];
      status.textContent = `Jev thinking (${thinking.join(", ")})…`;
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
    // ?ctf=1 / ?ctf=0 from the splash; absent keeps the per-mode default.
    ctf: params.has("ctf") ? params.get("ctf") === "1" : null,
    endpoint: params.get("jev") || window.TBGWAF_JEV_PROXY_URL || storedEndpoint || DEFAULT_JEV_ENDPOINT,
  };
}

function bootstrap() {
  const config = initialConfiguration();
  const controller = new JevPreviewController({ endpoint: config.endpoint });
  window.tbgwafJevPreview = controller;
  const endpointInput = document.getElementById("jev-endpoint");
  endpointInput.value = controller.endpoint;

  const reloadInto = (mode) => {
    const url = new URL(window.location.href);
    url.searchParams.set("mode", mode);
    url.searchParams.delete("room");
    window.location.href = url.toString();
  };
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      if (window.TBGWAF_SERVER) reloadInto(button.dataset.mode);
      else controller.setMode(button.dataset.mode);
    });
  });
  document.getElementById("start-ai").addEventListener("click", () => controller.start());
  document.getElementById("pause-ai").addEventListener("click", () => controller.pause());
  document.getElementById("restart-match").addEventListener("click", () => {
    if (window.TBGWAF_SERVER) reloadInto(controller.mode);
    else controller.restart();
  });
  document.getElementById("retry-ai").addEventListener("click", () => controller.retry());
  document.getElementById("fallback-ai").addEventListener("click", () => controller.useFallback());
  document.getElementById("save-endpoint").addEventListener("click", () => {
    const value = trimEndpoint(endpointInput.value);
    window.localStorage.setItem("tbgwafJevProxy", value);
    controller.setEndpoint(value);
  });

  const status = document.getElementById("status");
  window.onerror = () => { status.textContent = "Failed to load — check the browser console."; };
  // Server mode (window.TBGWAF_SERVER = { url, room }): each pane is its own
  // client of the authoritative server, paired through a private room, so
  // there is no in-page bus. Changing mode or restarting reloads the page
  // into a fresh room (the server owns the match, so it cannot be reset
  // from here).
  const server = window.TBGWAF_SERVER;
  const bus = { inboxes: [] };
  // Server-hosted Jev: the server seats Jev itself (Blue vs Jev: the human is
  // Blue and Jev is Red; Jev vs Jev: two Jevs, watched by one read-only
  // spectator), so only the human / spectator pane connects. The mode stays
  // selectable in the toolbar: it just reloads into a fresh room.
  const serverJev = Boolean(server) && config.mode !== "human";
  controller.ctf = config.ctf;
  controller.mode = config.mode;  // serverBots needs it before setMode() below.
  const spectate = Boolean(server) && config.mode === "ai-v-ai";
  let pending = serverJev ? 1 : spectate ? 3 : 2;
  const match = matchOptions(new URLSearchParams(window.location.search));
  const startPlayer = (player) => {
    const pane = document.getElementById(`pane-${player}`);
    const moduleConfig = {
      canvas: pane.querySelector("canvas"),
      tbgwafPlayer: player,
      tbgwafMap: match.map,
      tbgwafMapSeed: match.seed,
      ...(server
        ? { tbgwafServer: server.url, tbgwafRoom: server.room, tbgwafAutoEnter: true }
        : { tbgwafBus: bus }),
      tbgwafRequestDecision: (payload) => controller.requestDecision(player, payload),
      tbgwafReportMatch: (phase, round, winner) =>
        controller.reportMatch(player, phase, round, winner),
      tbgwafReportAiError: (message) => controller.reportLocalError(player, message),
      onRuntimeInitialized() {
        controller.registerModule(player, moduleConfig);
        if (--pending === 0) status.textContent = "";
      },
    };
    return moduleConfig;
  };
  if (serverJev && spectate) {
    // Jev vs Jev: one read-only client opens the room in the server's Jev mode.
    document.getElementById("panes")?.setAttribute("data-spectator-pane", "true");
    document.getElementById("pane-spectator").hidden = false;
    const spectatorConfig = {
      canvas: document.querySelector("#pane-spectator canvas"),
      tbgwafServer: server.url,
      tbgwafRoom: server.room,
      tbgwafBotMode: "ai-v-ai",
      tbgwafMap: match.map,
      tbgwafMapSeed: match.seed,
      tbgwafServerSpectate: true,
      tbgwafSpectator: true,
      tbgwafAutoEnter: true,
      onRuntimeInitialized() {
        controller.watchServer(spectatorConfig);
        if (--pending === 0) status.textContent = "";
      },
    };
    createTbgwafModule(spectatorConfig);
  } else if (serverJev) {
    // Blue vs Jev: the human's pane only; Red is Jev on the server.
    document.getElementById("pane-red").hidden = true;
    const blueConfig = startPlayer("blue");
    blueConfig.tbgwafBotMode = "player-v-ai";
    const registered = blueConfig.onRuntimeInitialized;
    blueConfig.onRuntimeInitialized = () => {
      registered();
      controller.watchServer(blueConfig);
    };
    createTbgwafModule(blueConfig);
  } else {
    const blueConfig = startPlayer("blue");
    if (server) {
      // The first client to join a room plays Blue, so Red connects only once
      // Blue's join has gone out.
      blueConfig.tbgwafOnFirstSend = () => createTbgwafModule(startPlayer("red"));
      createTbgwafModule(blueConfig);
    } else {
      createTbgwafModule(blueConfig);
      createTbgwafModule(startPlayer("red"));
    }
  }
  controller.setMode(config.mode);
}

if (typeof window !== "undefined" && typeof document !== "undefined") bootstrap();

export { MODES };
