#!/bin/sh
# Starts the Jev adapter as a loopback sidecar (when a Jev API key is
# configured) and then the game server, which asks it for Jev's moves in the
# Blue vs Jev / Jev vs Jev modes. The API key stays in this container.
if [ -n "$TBGWAF_JEV_API_KEY" ] && [ -z "$TBGWAF_JEV_ADAPTER_URL" ]; then
  PORT=8787 \
  TBGWAF_JEV_BIND=127.0.0.1 \
  TBGWAF_JEV_RATE_PER_MINUTE="${TBGWAF_JEV_RATE_PER_MINUTE:-120}" \
  TBGWAF_JEV_MAX_CONCURRENCY="${TBGWAF_JEV_MAX_CONCURRENCY:-4}" \
    node /opt/jev-adapter/server.mjs &
  export TBGWAF_JEV_ADAPTER_URL=http://127.0.0.1:8787
fi
exec tbgwaf_server
