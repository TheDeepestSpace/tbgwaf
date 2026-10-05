import { chooseWithJev } from "../server/jev-adapter/jev-client.mjs";

const apiKey = process.env.TBGWAF_JEV_API_KEY;
if (!apiKey) {
  console.error("TBGWAF_JEV_API_KEY is not available");
  process.exit(2);
}

let apiRequests = 0;
const result = await chooseWithJev({
  apiKey,
  state: {
    test: "tbgwaf bounded live smoke",
    actor: { team: "blue", position: [-4, 0, 0] },
    visible_enemies: [{ id: 3, position: [4, 0, 0] }],
  },
  candidates: [
    { id: "shoot_3", description: "Shoot the visible opposing figure." },
    { id: "wait", description: "Wait in place despite a visible target." },
  ],
  retries: 0,
  timeoutMs: 8000,
  onAttempt: () => { apiRequests += 1; },
});

if (!["shoot_3", "wait"].includes(result.choice) || apiRequests !== 1) {
  throw new Error("live smoke returned an invalid or unbounded result");
}
console.log(`Jev live smoke passed; api_requests=${apiRequests}; model=${result.model}; choice=${result.choice}`);
