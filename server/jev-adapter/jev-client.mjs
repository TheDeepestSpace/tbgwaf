export class JevUpstreamError extends Error {
  constructor(message, { status = 502, retryable = false } = {}) {
    super(message);
    this.name = "JevUpstreamError";
    this.status = status;
    this.retryable = retryable;
  }
}

const MAX_CANDIDATES_PER_FIGURE = 32;
const MAX_CANDIDATES = 200;

// Candidates are grouped by `figure`; each figure gets its own Choice question
// and all of them are answered in one upstream request. Returns
// Map<figure, Set<candidate id>>.
function validateCandidates(candidates) {
  if (!Array.isArray(candidates) || candidates.length < 1 || candidates.length > MAX_CANDIDATES) {
    throw new TypeError(`candidates must contain 1 to ${MAX_CANDIDATES} legal actions`);
  }
  const ids = new Set();
  const figures = new Map();
  for (const candidate of candidates) {
    if (!candidate || typeof candidate.id !== "string" ||
        !/^[A-Za-z0-9_-]{1,64}$/.test(candidate.id) || ids.has(candidate.id)) {
      throw new TypeError("candidate ids must be unique URL-safe strings");
    }
    if (!Number.isInteger(candidate.figure) || candidate.figure < 0 || candidate.figure > 999) {
      throw new TypeError("candidates must name the acting figure");
    }
    if (typeof candidate.description !== "string" || !candidate.description.trim() ||
        candidate.description.length > 512) {
      throw new TypeError("candidate descriptions must contain 1 to 512 characters");
    }
    ids.add(candidate.id);
    if (!figures.has(candidate.figure)) figures.set(candidate.figure, new Set());
    figures.get(candidate.figure).add(candidate.id);
  }
  for (const group of figures.values()) {
    if (group.size > MAX_CANDIDATES_PER_FIGURE) {
      throw new TypeError(`each figure may have at most ${MAX_CANDIDATES_PER_FIGURE} actions`);
    }
  }
  return figures;
}

function boundedRetryDelay(response) {
  const seconds = Number(response.headers?.get?.("retry-after"));
  return Number.isFinite(seconds) ? Math.min(500, Math.max(0, seconds * 1000)) : 100;
}

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

export async function chooseWithJev({
  apiKey,
  state,
  candidates,
  fetchImpl = globalThis.fetch,
  endpoint = "https://api.typesafe.ai/v1/systemone",
  model = "jev-latest",
  timeoutMs = 15000,
  retries = 1,
  onAttempt = () => {},
}) {
  if (!apiKey) throw new TypeError("TBGWAF_JEV_API_KEY is required");
  if (!state || typeof state !== "object" || Array.isArray(state)) {
    throw new TypeError("state must be an object");
  }
  const figures = validateCandidates(candidates);

  // A figure with exactly one legal action needs no model judgment; if every
  // figure is forced, skip the paid request entirely.
  const choices = {};
  const questions = {};
  for (const [figure, group] of figures) {
    if (group.size === 1) {
      choices[figure] = [...group][0];
      continue;
    }
    const criteria = Object.fromEntries(candidates
      .filter((candidate) => candidate.figure === figure)
      .map(({ id, description }) => [id, description]));
    questions[`figure_${figure}`] = {
      type: "choice",
      instructions: `Select the action for figure ${figure}. You are planning the whole squad at once: the other figures' questions are answered in the same request, so coordinate. Prefer useful shots, cover for each other, focus fire, stagger exposure, and avoid lines of fire through allies; otherwise make progress toward eliminating the opposing squad. Shot options fire a scattered burst; weigh the stated per-shot hit chance (it falls with distance and off-axis angle) and the weapon details in state.allies. Use state.map (grid, obstacles), state.ghosts and state.playbook to hide from enemy line of sight, take cover options when exposed, and hunt options to chase ghosts; move options state whether the destination is hidden or exposed. Return exactly one supplied option.`,
      criteria,
    };
  }
  if (Object.keys(questions).length === 0) {
    return { choices, confidence: 1, model: "forced-legal-choice", attempts: 0 };
  }
  const payload = { state, model, questions };

  let lastError;
  for (let attempt = 0; attempt <= retries; attempt += 1) {
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(), timeoutMs);
    onAttempt(attempt + 1);
    try {
      const response = await fetchImpl(endpoint, {
        method: "POST",
        headers: {
          "Authorization": `Bearer ${apiKey}`,
          "Content-Type": "application/json",
        },
        body: JSON.stringify(payload),
        signal: abort.signal,
      });
      if (!response.ok) {
        const retryable = response.status === 429 || response.status >= 500;
        const error = new JevUpstreamError(`TypeSafe returned HTTP ${response.status}`, {
          status: 502,
          retryable,
        });
        if (!retryable || attempt === retries) throw error;
        lastError = error;
        await delay(boundedRetryDelay(response));
        continue;
      }
      const body = await response.json();
      const answered = { ...choices };
      const confidences = [];
      for (const figure of Object.keys(questions).map((name) => Number(name.slice(7)))) {
        const answer = body?.answers?.[`figure_${figure}`];
        if (answer?.type !== "choice" || !figures.get(figure).has(answer.choice)) {
          throw new JevUpstreamError("TypeSafe returned an invalid Choice answer");
        }
        answered[figure] = answer.choice;
        confidences.push(answer.confidence);
      }
      const confidence = confidences.every(Number.isFinite) ? Math.min(...confidences) : null;
      return {
        choices: answered,
        confidence,
        model: typeof body.model === "string" ? body.model : model,
        attempts: attempt + 1,
      };
    } catch (error) {
      const timedOut = abort.signal.aborted;
      const normalized = timedOut
        ? new JevUpstreamError("TypeSafe request timed out", { retryable: true })
        : error instanceof JevUpstreamError
          ? error
          : new JevUpstreamError("TypeSafe request failed", { retryable: true });
      if (!normalized.retryable || attempt === retries) throw normalized;
      lastError = normalized;
      await delay(100);
    } finally {
      clearTimeout(timer);
    }
  }
  throw lastError || new JevUpstreamError("TypeSafe request failed");
}

export { validateCandidates };
