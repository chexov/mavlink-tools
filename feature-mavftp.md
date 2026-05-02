# mavftp

## What was done

Added `mavlink-tools ftp` — a MAVLink FTP client compatible in spirit with `pymavlink`'s `mavftp.py`. Supports `get` (BurstReadFile) and `ls` (ListDirectory) over `udpin:` / `udpout:` / `tcpin:` / `tcpout:` / `serial:` connection strings. Source: `src/ftp.rs`.

Reorganized the crate from a single `mavlink-dump` binary into one `mavlink-tools` binary with subcommands (`dump`, `ftp`); old logic moved to `src/dump.rs`.

## How it works

`FtpClient` wraps `Arc<dyn MavConnection<MavMessage> + Send + Sync>` (the connection is internally synchronized via Mutex inside the `mavlink` crate, so send and recv from different threads is safe).

Three threads:
1. **Recv worker** — loops on `conn.recv()`, filters `FILE_TRANSFER_PROTOCOL` messages targeting our `(source_system, source_component)`, decodes the 251-byte payload into `Payload` struct, forwards via `mpsc::Sender`.
2. **Heartbeat emitter** — sends `HEARTBEAT(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_STATE_ACTIVE)` at 1 Hz so FCs that filter unknown peers route FTP responses back.
3. **Main** — drives request/response via `request()` (seq-tracked, retried on timeout) for one-shot ops, or `burst_get()` for `get`.

Payload wire format (from <https://mavlink.io/en/services/ftp.html>): seq(2,LE), session, opcode, size, req_opcode, burst_complete, padding, offset(4,LE), data[239].

Opcodes implemented: TerminateSession(0), ResetSessions(1), ListDirectory(3), OpenFileRO(4), BurstReadFile(15), ACK(128), NAK(129).

`get` flow: OpenFileRO → BurstReadFile loop (server pushes ACKs with incrementing offset; on `burst_complete=1` re-issue from current offset; on `NAK_EOF` done; on stall (no progress within `--timeout-ms`) re-issue) → TerminateSession.

`ls` flow: ListDirectory paginated by entry index; entries are `\0`-separated; first byte `F`/`D`/`S` = file/dir/skip; files have `name\tSIZE`. Loop until `NAK_EOF` or empty data.

## Quirks

- `ResetSessions` is sent on connect (matches pymavlink). Without it, after a few aborted runs the FC returns `NAK_NO_SESSIONS_AVAILABLE`.
- Burst stall handling: if no data arrives within `--timeout-ms` but `offset > burst_start`, we re-issue BurstReadFile from the new offset rather than failing. If `offset == burst_start` (true zero progress), it's a hard error.
- The `mavlink` crate's `connect()` returns `Connection<M>` which is internally `Sync` because each variant (`UdpConnection`, `TcpConnection`, `SerialConnection`) wraps reader/writer in separate `Mutex`es.
- `set_protocol_version(MavlinkVersion::V2)` is called on the connection — needed because `FILE_TRANSFER_PROTOCOL` payload (251 bytes) only fits in v2 frames.
- Pymavlink-style `udp:<addr>:<port>` (which means UDP-listen) is **not** translated. Use `udpin:<addr>:<port>` with this tool. Decision was explicit — no compat shim.
- HEARTBEAT field name is `mavtype`, not `type` (Rust keyword), in the generated `HEARTBEAT_DATA` struct.
- `BurstReadFile` not `ReadFile`. ReadFile was implemented and removed; burst is faster (one round-trip pushes many chunks) and matches pymavlink's default. If a future FC chokes on burst, `request()` is reusable for a ReadFile fallback.

## Not yet

`put`, `rm`, `mkdir`, `rmdir`, `rename`, `crc`, `cmp`. Protocol scaffolding (request/response with seq + NAK handling) is in place; each is ~20-40 lines plus CLI wiring.
