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
    { id: "f0_shoot_3", figure: 0, description: "Shoot the visible opposing figure." },
    { id: "f0_wait", figure: 0, description: "Wait in place despite a visible target." },
  ],
  retries: 0,
  timeoutMs: 8000,
  onAttempt: () => { apiRequests += 1; },
});

if (!["f0_shoot_3", "f0_wait"].includes(result.choices[0]) || apiRequests !== 1) {
  throw new Error("live smoke returned an invalid or unbounded result");
}
console.log(`Jev live smoke passed; api_requests=${apiRequests}; model=${result.model}; choice=${result.choices[0]}`);
