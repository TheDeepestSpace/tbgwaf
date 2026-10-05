import assert from "node:assert/strict";
import test from "node:test";

import { JevPreviewController, MODES } from "../jev-preview.js";

const payload = JSON.stringify({
  requestId: "r1-t1-u3-n1",
  state: { round: 1, team: "red", visible_enemies: [] },
  candidates: [
    { id: "move_0", description: "Advance." },
    { id: "wait", description: "Wait." },
  ],
});

function moduleStub() { return { tbgwafAiInbox: [] }; }
function okChoice(choice = "move_0") {
  return new Response(JSON.stringify({ choice }), {
    status: 200,
    headers: { "content-type": "application/json" },
  });
}

test("exposes all three modes and configures their controlled teams", () => {
  assert.deepEqual(Object.keys(MODES), ["human", "player-v-ai", "ai-v-ai"]);
  const controller = new JevPreviewController({ documentRef: null, endpoint: "http://adapter" });
  const blue = moduleStub();
  const red = moduleStub();
  controller.registerModule("blue", blue);
  controller.registerModule("red", red);

  controller.setMode("human");
  assert.equal(blue.tbgwafAI, false);
  assert.equal(red.tbgwafAI, false);
  assert.equal(blue.tbgwafAutoCommit, false);

  controller.setMode("player-v-ai");
  assert.equal(blue.tbgwafAI, false);
  assert.equal(red.tbgwafAI, true);
  assert.equal(blue.tbgwafAutoCommit, true);

  controller.setMode("ai-v-ai");
  assert.equal(blue.tbgwafAI, true);
  assert.equal(red.tbgwafAI, true);
  assert.equal(blue.tbgwafAIPaused, true);
  controller.start();
  assert.equal(blue.tbgwafAIPaused, false);
});

test("accepts one legal response and rejects duplicate concurrent requests", async () => {
  let calls = 0;
  let resolveFetch;
  const controller = new JevPreviewController({
    documentRef: null,
    endpoint: "http://adapter",
    fetchImpl: async () => {
      calls += 1;
      return await new Promise((resolve) => { resolveFetch = resolve; });
    },
  });
  const red = moduleStub();
  controller.registerModule("red", red);
  controller.setMode("player-v-ai");
  const first = controller.requestDecision("red", payload);
  const duplicate = controller.requestDecision("red", payload);
  resolveFetch(okChoice());
  await Promise.all([first, duplicate]);
  assert.equal(calls, 1);
  assert.deepEqual(red.tbgwafAiInbox, ["D r1-t1-u3-n1 move_0"]);
});

test("pause and restart abort stale requests without delivering decisions", async () => {
  const controller = new JevPreviewController({
    documentRef: null,
    endpoint: "http://adapter",
    fetchImpl: async (_url, { signal }) => await new Promise((_resolve, reject) => {
      signal.addEventListener("abort", () => reject(new DOMException("aborted", "AbortError")));
    }),
  });
  const red = moduleStub();
  controller.registerModule("red", red);
  controller.setMode("ai-v-ai");
  controller.start();
  const pending = controller.requestDecision("red", payload);
  controller.pause();
  await pending;
  assert.deepEqual(red.tbgwafAiInbox, []);
  controller.restart();
  assert.equal(red.tbgwafRestart, true);
});

test("surfaces invalid responses and timeout failures for retry or fallback", async () => {
  const red = moduleStub();
  const invalid = new JevPreviewController({
    documentRef: null,
    endpoint: "http://adapter",
    fetchImpl: async () => okChoice("teleport"),
  });
  invalid.registerModule("red", red);
  invalid.setMode("player-v-ai");
  await invalid.requestDecision("red", payload);
  assert.deepEqual(red.tbgwafAiInbox, ["E r1-t1-u3-n1"]);
  invalid.useFallback();
  assert.equal(red.tbgwafAIFallback, true);
  assert.equal(red.tbgwafRetryAI, true);

  const timedOutRed = moduleStub();
  const timedOut = new JevPreviewController({
    documentRef: null,
    endpoint: "http://adapter",
    timeoutMs: 5,
    fetchImpl: async (_url, { signal }) => await new Promise((_resolve, reject) => {
      signal.addEventListener("abort", () => reject(new DOMException("aborted", "AbortError")));
    }),
  });
  timedOut.registerModule("red", timedOutRed);
  timedOut.setMode("player-v-ai");
  await timedOut.requestDecision("red", payload);
  assert.deepEqual(timedOutRed.tbgwafAiInbox, ["E r1-t1-u3-n1"]);
});
