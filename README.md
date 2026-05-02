# mavlink-tools

Fast CLI tools for working with MAVLink telemetry logs.

## Tools

### mavlink-tools dump

Dump MAVLink tlog files as human-readable text or JSON Lines.

```
mavlink-tools dump [OPTIONS] <FILE>
```

| Flag | Description |
|---|---|
| `-f, --filter <NAMES>` | Exact message type names, comma-separated (e.g. `HEARTBEAT,ATTITUDE`) |
| `-m, --match <PATTERN>` | Substring match on message name, case-insensitive (e.g. `FLOAT` matches `NAMED_VALUE_FLOAT`) |
| `-j, --json` | Output as JSON Lines (one JSON object per message) |
| `-s, --summary` | Print only a summary table (message counts and rates) |

Filters `-f` and `-m` can be combined (both must match). Multiple `-f` values can be passed as `-f A,B` or `-f A -f B`.

### mavlink-tools ftp

MAVLink FTP client (compatible with `pymavlink`'s `mavftp`). Downloads files from an autopilot's filesystem over a live MAVLink connection.

```
mavlink-tools ftp --master <CONN> [OPTIONS] <get|ls> <args...>
```

| Flag | Description |
|---|---|
| `--master <CONN>` | Connection string. Format: `udpin:<addr>:<port>`, `udpout:<addr>:<port>`, `tcpin:<addr>:<port>`, `tcpout:<addr>:<port>`, `serial:<port>:<baud>` |
| `--target-system <N>` | Autopilot system ID (default `1`) |
| `--target-component <N>` | Autopilot component ID (default `1`) |
| `--source-system <N>` | This client's system ID (default `255`, GCS-style) |
| `--source-component <N>` | This client's component ID (default `0`) |
| `--timeout-ms <MS>` | Per-request timeout (default `500`) |
| `--retries <N>` | Retries per request (default `3`) |

`<LOCAL>` may be `-` to write to stdout.

Examples:
```sh
# Read ArduPilot's UART status virtual file to stdout
mavlink-tools ftp --master udpin:0.0.0.0:14550 get @SYS/uarts.txt -

# Pull a flight log to disk
mavlink-tools ftp --master serial:/dev/cu.usbmodem0:115200 get /APM/LOGS/00001.BIN ./00001.BIN

# List a directory
mavlink-tools ftp --master udpin:0.0.0.0:14550 ls /APM/LOGS
```

Notes:
- `get` uses `BurstReadFile` — server pushes 239-byte chunks; on stall the client re-issues from the current offset.
- `ls` paginates through `ListDirectory` (offset = entry index). Output: `D` entries print as `name/`, `F` entries print as `name<TAB>size`.
- A 1 Hz `HEARTBEAT` is emitted from `(source_system, source_component)` so FCs that filter unknown peers route FTP responses back.
- A `ResetSessions` is sent on connect to clear leftover sessions on the FC after aborted runs.
- Other ops (`put`, `rm`, `mkdir`, `rmdir`, `rename`, `crc`) are not yet implemented.

### mavlink-gps

GPS track extraction from tlog files. (planned)

## Install

```sh
cargo install --path .
```

## Examples

Dump all messages:
```sh
mavlink-tools dump flight.tlog
```

```
2026-04-15 14:39:15.072 [  2:1  ] ATTITUDE {time_boot_ms: 840960, roll: 0.136, pitch: 0.069, ...}
2026-04-15 14:39:15.079 [  2:1  ] GLOBAL_POSITION_INT {time_boot_ms: 840960, lat: 503997842, ...}
```

Filter by exact name:
```sh
mavlink-tools dump -f HEARTBEAT,ATTITUDE flight.tlog
```

Filter by substring:
```sh
mavlink-tools dump -m status flight.tlog
# matches: SYS_STATUS, POWER_STATUS, EKF_STATUS_REPORT, STATUSTEXT
```

Summary table:
```sh
mavlink-tools dump -s flight.tlog
```

```
Message                     Count      Hz
NAMED_VALUE_INT             11123     9.2
ATTITUDE                     4836     4.0
GPS_RAW_INT                  4836     4.0
HEARTBEAT                    1610     1.3
---
Total: 113015 messages, duration: 1208.8s
```

JSON Lines output (pipe to `jq`, load in Python, etc.):
```sh
mavlink-tools dump -j -m HEART flight.tlog | jq '.data'
```

```json
{"ts":"2026-04-15 14:39:15.671","ts_us":1776263955671529,"sys":2,"comp":1,"msg":"HEARTBEAT","data":{"autopilot":{"type":"MAV_AUTOPILOT_ARDUPILOTMEGA"},"base_mode":"MAV_MODE_FLAG_CUSTOM_MODE_ENABLED","custom_mode":0,"mavtype":{"type":"MAV_TYPE_FIXED_WING"},...}}
```

## tlog format

A `.tlog` file is a repeating sequence of:

```
[8-byte big-endian u64 timestamp (microseconds since Unix epoch)][MAVLink v1/v2 frame]
```

This is the standard format written by QGroundControl, Mission Planner, and MAVProxy.

## Output columns

```
TIMESTAMP              [SYS:COMP] MSG_TYPE {field: value, ...}
```

- **TIMESTAMP** -- UTC, millisecond precision
- **SYS:COMP** -- MAVLink system ID and component ID
- **MSG_TYPE** -- message name from ardupilotmega dialect
- **fields** -- message fields in human-readable form (byte arrays shown as ASCII strings when printable)

## Notes

- Pipe-safe: exits cleanly on broken pipe (`| head`)
- Parse errors reported to stderr; error count printed at end
- Desync detection: if a timestamp jumps >1 day from the previous good one, prints `[desync]` instead of a bogus time
- Uses the `ardupilotmega` MAVLink dialect (superset of `common`)
