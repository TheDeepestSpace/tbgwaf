import http from "node:http";
import { fileURLToPath } from "node:url";
import { chooseWithJev, JevUpstreamError, validateCandidates } from "./jev-client.mjs";

const MAX_BODY_BYTES = 16 * 1024;

function integerEnv(env, name, fallback, minimum, maximum) {
  const parsed = Number.parseInt(env[name] || "", 10);
  return Number.isFinite(parsed) ? Math.min(maximum, Math.max(minimum, parsed)) : fallback;
}

function readBody(request) {
  return new Promise((resolve, reject) => {
    let size = 0;
    let tooLarge = false;
    const chunks = [];
    request.on("data", (chunk) => {
      if (tooLarge) return;
      size += chunk.length;
      if (size > MAX_BODY_BYTES) {
        tooLarge = true;
        reject(Object.assign(new Error("request body is too large"), { status: 413 }));
        return;
      }
      chunks.push(chunk);
    });
    request.on("end", () => {
      if (tooLarge) return;
      try {
        resolve(JSON.parse(Buffer.concat(chunks).toString("utf8")));
      } catch {
        reject(Object.assign(new Error("request body must be valid JSON"), { status: 400 }));
      }
    });
    request.on("error", reject);
  });
}

function isAllowedOrigin(origin, configuredOrigins) {
  if (!origin) return true; // CLI/health clients do not participate in CORS.
  if (/^https?:\/\/(localhost|127\.0\.0\.1)(:\d+)?$/.test(origin)) return true;
  return configuredOrigins.has(origin);
}

function sendJson(response, status, body, origin = "") {
  const json = JSON.stringify(body);
  response.writeHead(status, {
    "Content-Type": "application/json; charset=utf-8",
    "Content-Length": Buffer.byteLength(json),
    "Cache-Control": "no-store",
    "Vary": "Origin",
    ...(origin ? { "Access-Control-Allow-Origin": origin } : {}),
  });
  response.end(json);
}

class FixedWindowLimiter {
  constructor({ perMinute, daily }) {
    this.perMinute = perMinute;
    this.daily = daily;
    this.clients = new Map();
    this.day = new Date().toISOString().slice(0, 10);
    this.dailyCount = 0;
  }

  take(key, now = Date.now()) {
    const day = new Date(now).toISOString().slice(0, 10);
    if (day !== this.day) {
      this.day = day;
      this.dailyCount = 0;
      this.clients.clear();
    }
    if (this.dailyCount >= this.daily) return { ok: false, reason: "daily request budget exhausted" };
    const minute = Math.floor(now / 60000);
    const current = this.clients.get(key);
    const entry = current?.minute === minute ? current : { minute, count: 0 };
    if (entry.count >= this.perMinute) return { ok: false, reason: "per-client rate limit exceeded" };
    entry.count += 1;
    this.clients.set(key, entry);
    this.dailyCount += 1;
    return { ok: true, dailyRemaining: this.daily - this.dailyCount };
  }
}

function clientAddress(request, trustProxy) {
  if (trustProxy) {
    const forwarded = request.headers["x-forwarded-for"];
    if (typeof forwarded === "string") return forwarded.split(",")[0].trim();
  }
  return request.socket.remoteAddress || "unknown";
}

function authorized(request, bearerToken) {
  if (!bearerToken) return true;
  return request.headers.authorization === `Bearer ${bearerToken}`;
}

export function createJevAdapter({ env = process.env, choose = chooseWithJev, logger = console } = {}) {
  const origins = new Set((env.TBGWAF_JEV_ALLOWED_ORIGINS || "")
    .split(",").map((value) => value.trim()).filter(Boolean));
  const limiter = new FixedWindowLimiter({
    perMinute: integerEnv(env, "TBGWAF_JEV_RATE_PER_MINUTE", 12, 1, 120),
    daily: integerEnv(env, "TBGWAF_JEV_DAILY_LIMIT", 500, 1, 100000),
  });
  const concurrencyLimit = integerEnv(env, "TBGWAF_JEV_MAX_CONCURRENCY", 2, 1, 16);
  const timeoutMs = integerEnv(env, "TBGWAF_JEV_TIMEOUT_MS", 8000, 500, 30000);
  const retries = integerEnv(env, "TBGWAF_JEV_RETRIES", 1, 0, 2);
  const trustProxy = env.TBGWAF_JEV_TRUST_PROXY === "1";
  let active = 0;

  return http.createServer(async (request, response) => {
    const origin = typeof request.headers.origin === "string" ? request.headers.origin : "";
    if (!isAllowedOrigin(origin, origins)) {
      sendJson(response, 403, { error: "origin is not allowed" });
      return;
    }
    if (request.method === "OPTIONS") {
      response.writeHead(204, {
        "Access-Control-Allow-Origin": origin,
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type, Authorization",
        "Access-Control-Max-Age": "600",
        "Vary": "Origin",
      });
      response.end();
      return;
    }
    if (request.method === "GET" && request.url === "/health") {
      sendJson(response, 200, { ok: true }, origin);
      return;
    }
    if (request.method !== "POST" || request.url !== "/v1/choice") {
      sendJson(response, 404, { error: "not found" }, origin);
      return;
    }
    if (!authorized(request, env.TBGWAF_JEV_PROXY_BEARER_TOKEN || "")) {
      sendJson(response, 401, { error: "unauthorized" }, origin);
      return;
    }
    if (!env.TBGWAF_JEV_API_KEY) {
      sendJson(response, 503, { error: "Jev is not configured" }, origin);
      return;
    }
    const allowance = limiter.take(clientAddress(request, trustProxy));
    if (!allowance.ok) {
      sendJson(response, 429, { error: allowance.reason }, origin);
      return;
    }
    if (active >= concurrencyLimit) {
      sendJson(response, 429, { error: "proxy is busy" }, origin);
      return;
    }

    let requestId = "unknown";
    let acquiredSlot = false;
    try {
      const body = await readBody(request);
      requestId = typeof body.requestId === "string" ? body.requestId.slice(0, 96) : "unknown";
      if (!/^[A-Za-z0-9_-]{1,96}$/.test(requestId)) {
        throw Object.assign(new Error("requestId is invalid"), { status: 400 });
      }
      if (!body.state || typeof body.state !== "object" || Array.isArray(body.state)) {
        throw Object.assign(new Error("state must be an object"), { status: 400 });
      }
      try {
        validateCandidates(body.candidates);
      } catch (error) {
        throw Object.assign(error, { status: 400 });
      }

      active += 1;
      acquiredSlot = true;
      let upstreamRequests = 0;
      const result = await choose({
        apiKey: env.TBGWAF_JEV_API_KEY,
        state: body.state,
        candidates: body.candidates,
        model: env.TBGWAF_JEV_MODEL || "jev-latest",
        timeoutMs,
        retries,
        onAttempt: () => { upstreamRequests += 1; },
      });
      sendJson(response, 200, {
        requestId,
        choice: result.choice,
        confidence: result.confidence,
        model: result.model,
        upstreamRequests,
      }, origin);
      logger.info?.("Jev choice completed", { requestId, upstreamRequests });
    } catch (error) {
      const status = Number.isInteger(error.status) ? error.status
        : error instanceof JevUpstreamError ? error.status : 500;
      const publicMessage = status < 500 ? error.message : "Jev request failed";
      sendJson(response, status, { requestId, error: publicMessage }, origin);
      logger.warn?.("Jev choice failed", { requestId, status });
    } finally {
      if (acquiredSlot) active -= 1;
    }
  });
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const port = integerEnv(process.env, "PORT", 8787, 1, 65535);
  const server = createJevAdapter();
  server.listen(port, "0.0.0.0", () => {
    // Configuration only; never include request headers or the API key.
    console.info(`tbgwaf Jev adapter listening on port ${port}`);
  });
}

export { FixedWindowLimiter, isAllowedOrigin };
