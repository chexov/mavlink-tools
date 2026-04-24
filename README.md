# mavlink-dump

Fast CLI tool to dump MAVLink tlog files as human-readable text or JSON Lines.

## Install

```sh
cargo install --path .
```

## Usage

```
mavlink-dump [OPTIONS] <FILE>
```

### Options

| Flag | Description |
|---|---|
| `-f, --filter <NAMES>` | Exact message type names, comma-separated (e.g. `HEARTBEAT,ATTITUDE`) |
| `-m, --match <PATTERN>` | Substring match on message name, case-insensitive (e.g. `FLOAT` matches `NAMED_VALUE_FLOAT`) |
| `-j, --json` | Output as JSON Lines (one JSON object per message) |
| `-s, --summary` | Print only a summary table (message counts and rates) |

Filters `-f` and `-m` can be combined (both must match). Multiple `-f` values can be passed as `-f A,B` or `-f A -f B`.

### Examples

Dump all messages:
```sh
mavlink-dump flight.tlog
```

```
2026-04-15 14:39:15.072 [  2:1  ] ATTITUDE {time_boot_ms: 840960, roll: 0.136, pitch: 0.069, ...}
2026-04-15 14:39:15.079 [  2:1  ] GLOBAL_POSITION_INT {time_boot_ms: 840960, lat: 503997842, ...}
```

Filter by exact name:
```sh
mavlink-dump -f HEARTBEAT,ATTITUDE flight.tlog
```

Filter by substring:
```sh
mavlink-dump -m status flight.tlog
# matches: SYS_STATUS, POWER_STATUS, EKF_STATUS_REPORT, STATUSTEXT
```

Summary table:
```sh
mavlink-dump -s flight.tlog
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
mavlink-dump -j -m HEART flight.tlog | jq '.data'
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
