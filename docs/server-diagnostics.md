# Diagnosing the game server on Fly.io

`tbgwaf_server` logs timestamped lines to stdout; Fly keeps them for `fly logs`.

```
fly logs -a tbgwaf-server            # live tail (PR apps: tbgwaf-server-pr-<N>)
fly logs -a tbgwaf-server --no-tail  # recent buffer, then exit
fly status -a tbgwaf-server          # machine state, restarts, last event
fly machine status <id> -a tbgwaf-server   # event history: start/stop/exit code/OOM
```

## What is logged

| Line | Meaning |
| --- | --- |
| `conn N accepted` / `conn N closed: reason=…` | Every socket, with close reason (`peer-closed`, `idle-timeout`, `rate-limit`, `output-backlog`, `protocol-error`, `handler-exception`, …), age, bytes and message counts. |
| `stats: conns=… rooms=… fds=… rssKb=… maxPassMs=…` | Every 30 s. Watch `conns` creeping toward 128, `fds`, `rssKb` toward the 256 MB VM limit, and `maxPassMs`. |
| `room X: match started` / `round N simulated in M ms` / `… left` | Room lifecycle; a slow round blocks every other room (single thread). |
| `slow loop pass: N ms` | One pass of the event loop took ≥ 500 ms. |
| `watchdog: loop pass stuck for N ms in phase '…'` | The loop has not finished a pass for ≥ 3 s; the phase says where (`read-and-dispatch`, `lobby-tick`, `flush`, …). Logged from a separate thread, so it appears even when the loop is hung. |
| `SIGTERM received` | Fly asked the process to stop (autostop, deploy, restart). |
| `FATAL: crashed with signal …` + backtrace, `FATAL: uncaught exception` | The process died; Fly then restarts it and all matches are lost. |

A machine that dies with **no** FATAL/SIGTERM line was most likely SIGKILLed — typically the OOM killer
(256 MB). Confirm with `fly machine status`, and compare the last `rssKb` in `stats`.

Other causes visible without logs: `auto_stop_machines = "stop"` stops a machine with no
connections (state is in memory and is lost), and the first request after that waits for a cold start.
