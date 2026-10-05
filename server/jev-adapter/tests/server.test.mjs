import assert from "node:assert/strict";
import test from "node:test";

import { createJevAdapter } from "../server.mjs";

const silentLogger = { info() {}, warn() {} };

async function withServer(options, callback) {
  const server = createJevAdapter({ logger: silentLogger, ...options });
  await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
  const address = server.address();
  try {
    await callback(`http://127.0.0.1:${address.port}`);
  } finally {
    await new Promise((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
  }
}

const requestBody = {
  requestId: "r1-t0-u0-n1",
  state: { round: 1, team: "blue", visible_enemies: [] },
  candidates: [
    { id: "move_0", description: "Advance." },
    { id: "wait", description: "Wait." },
  ],
};

test("adapts a browser request without returning the secret", async () => {
  await withServer({
    env: { TBGWAF_JEV_API_KEY: "never-return-this", TBGWAF_JEV_ALLOWED_ORIGINS: "https://preview.example" },
    choose: async ({ apiKey, candidates, onAttempt }) => {
      assert.equal(apiKey, "never-return-this");
      onAttempt();
      return { choice: candidates[0].id, confidence: 0.8, model: "jev-test" };
    },
  }, async (baseUrl) => {
    const response = await fetch(`${baseUrl}/v1/choice`, {
      method: "POST",
      headers: { "content-type": "application/json", origin: "https://preview.example" },
      body: JSON.stringify(requestBody),
    });
    assert.equal(response.status, 200);
    assert.equal(response.headers.get("access-control-allow-origin"), "https://preview.example");
    const text = await response.text();
    assert.equal(text.includes("never-return-this"), false);
    const body = JSON.parse(text);
    assert.equal(body.choice, "move_0");
    assert.equal(body.upstreamRequests, 1);
  });
});

test("blocks unlisted browser origins before spending an API call", async () => {
  let calls = 0;
  await withServer({
    env: { TBGWAF_JEV_API_KEY: "test", TBGWAF_JEV_ALLOWED_ORIGINS: "https://good.example" },
    choose: async () => { calls += 1; },
  }, async (baseUrl) => {
    const response = await fetch(`${baseUrl}/v1/choice`, {
      method: "POST",
      headers: { "content-type": "application/json", origin: "https://bad.example" },
      body: JSON.stringify(requestBody),
    });
    assert.equal(response.status, 403);
    assert.equal(calls, 0);
  });
});

test("rate limits before exceeding the configured paid-request budget", async () => {
  let calls = 0;
  await withServer({
    env: { TBGWAF_JEV_API_KEY: "test", TBGWAF_JEV_RATE_PER_MINUTE: "1" },
    choose: async ({ onAttempt }) => {
      calls += 1;
      onAttempt();
      return { choice: "wait", confidence: 1, model: "stub" };
    },
  }, async (baseUrl) => {
    const first = await fetch(`${baseUrl}/v1/choice`, {
      method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(requestBody),
    });
    const second = await fetch(`${baseUrl}/v1/choice`, {
      method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify(requestBody),
    });
    assert.equal(first.status, 200);
    assert.equal(second.status, 429);
    assert.equal(calls, 1);
  });
});
