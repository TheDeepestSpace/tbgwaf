export class JevUpstreamError extends Error {
  constructor(message, { status = 502, retryable = false } = {}) {
    super(message);
    this.name = "JevUpstreamError";
    this.status = status;
    this.retryable = retryable;
  }
}

function validateCandidates(candidates) {
  if (!Array.isArray(candidates) || candidates.length < 1 || candidates.length > 24) {
    throw new TypeError("candidates must contain 1 to 24 legal actions");
  }
  const ids = new Set();
  for (const candidate of candidates) {
    if (!candidate || typeof candidate.id !== "string" ||
        !/^[A-Za-z0-9_-]{1,64}$/.test(candidate.id) || ids.has(candidate.id)) {
      throw new TypeError("candidate ids must be unique URL-safe strings");
    }
    if (typeof candidate.description !== "string" || !candidate.description.trim() ||
        candidate.description.length > 512) {
      throw new TypeError("candidate descriptions must contain 1 to 512 characters");
    }
    ids.add(candidate.id);
  }
  return ids;
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
  timeoutMs = 8000,
  retries = 1,
  onAttempt = () => {},
}) {
  if (!apiKey) throw new TypeError("TBGWAF_JEV_API_KEY is required");
  if (!state || typeof state !== "object" || Array.isArray(state)) {
    throw new TypeError("state must be an object");
  }
  const candidateIds = validateCandidates(candidates);

  // No model judgment is needed when game code found exactly one legal
  // action. This also avoids spending a paid request on a forced move.
  if (candidates.length === 1) {
    return { choice: candidates[0].id, confidence: 1, model: "forced-legal-choice", attempts: 0 };
  }

  const criteria = Object.fromEntries(candidates.map(({ id, description }) => [id, description]));
  const payload = {
    state,
    model,
    questions: {
      action: {
        type: "choice",
        instructions: "Select the strongest legal action for the acting figure. Prefer a useful visible shot; otherwise make progress toward eliminating the opposing squad. Return exactly one supplied option.",
        criteria,
      },
    },
  };

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
      const answer = body?.answers?.action;
      if (answer?.type !== "choice" || !candidateIds.has(answer.choice)) {
        throw new JevUpstreamError("TypeSafe returned an invalid Choice answer");
      }
      return {
        choice: answer.choice,
        confidence: Number.isFinite(answer.confidence) ? answer.confidence : null,
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
