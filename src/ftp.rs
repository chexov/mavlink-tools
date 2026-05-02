// MAVLink FTP client — see https://mavlink.io/en/services/ftp.html
//
// Wire format of the 251-byte FILE_TRANSFER_PROTOCOL.payload:
//   [0..2]  seq_number     (u16 LE)
//   [2]     session
//   [3]     opcode
//   [4]     size           (data length on response, ignored on most requests)
//   [5]     req_opcode     (response: original opcode)
//   [6]     burst_complete (response: 1 = last burst chunk)
//   [7]     padding
//   [8..12] offset         (u32 LE)
//   [12..251] data         (239 bytes, `size` of which are valid)

use std::fs::File;
use std::io::{self, Write};
use std::sync::Arc;
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::thread;
use std::time::{Duration, Instant};

use clap::{Args, Subcommand};
use mavlink::ardupilotmega::{
    FILE_TRANSFER_PROTOCOL_DATA, HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState,
    MavType,
};
use mavlink::{MavConnection, MavHeader, MavlinkVersion};

const PAYLOAD_HEADER_LEN: usize = 12;
const PAYLOAD_DATA_LEN: usize = 239;
const PAYLOAD_LEN: usize = 251;

// Opcodes
const OP_TERMINATE_SESSION: u8 = 0;
const OP_RESET_SESSIONS: u8 = 1;
const OP_LIST_DIRECTORY: u8 = 3;
const OP_OPEN_FILE_RO: u8 = 4;
const OP_BURST_READ_FILE: u8 = 15;
const OP_ACK: u8 = 128;
const OP_NAK: u8 = 129;

// NAK error codes
const NAK_NONE: u8 = 0;
const NAK_FAIL: u8 = 1;
const NAK_FAIL_ERRNO: u8 = 2;
const NAK_INVALID_DATA_SIZE: u8 = 3;
const NAK_INVALID_SESSION: u8 = 4;
const NAK_NO_SESSIONS_AVAILABLE: u8 = 5;
const NAK_EOF: u8 = 6;
const NAK_UNKNOWN_COMMAND: u8 = 7;
const NAK_FILE_EXISTS: u8 = 8;
const NAK_FILE_PROTECTED: u8 = 9;
const NAK_FILE_NOT_FOUND: u8 = 10;

#[derive(Args)]
#[command(about = "MAVLink FTP client (compatible with pymavlink mavftp)")]
pub struct FtpArgs {
    /// MAVLink connection string (e.g. udpin:0.0.0.0:14550, tcpout:127.0.0.1:5760, serial:/dev/cu.usbmodem0:115200)
    #[arg(long)]
    master: String,
    /// Target system ID (autopilot)
    #[arg(long, default_value_t = 1)]
    target_system: u8,
    /// Target component ID
    #[arg(long, default_value_t = 1)]
    target_component: u8,
    /// Source system ID (this client)
    #[arg(long, default_value_t = 255)]
    source_system: u8,
    /// Source component ID
    #[arg(long, default_value_t = 0)]
    source_component: u8,
    /// Per-request timeout, milliseconds
    #[arg(long, default_value_t = 500)]
    timeout_ms: u64,
    /// Retries per request before giving up
    #[arg(long, default_value_t = 3)]
    retries: u32,

    #[command(subcommand)]
    op: FtpOp,
}

#[derive(Subcommand)]
enum FtpOp {
    /// Download a file. Use '-' for local to write to stdout.
    Get {
        /// Remote path (e.g. @SYS/uarts.txt, /APM/LOGS/00001.BIN)
        remote: String,
        /// Local path or '-' for stdout
        local: String,
    },
    /// List a directory.
    Ls {
        /// Remote directory (e.g. /, @SYS, /APM/LOGS)
        path: String,
    },
}

#[derive(Debug, Clone)]
struct Payload {
    seq: u16,
    session: u8,
    opcode: u8,
    size: u8,
    req_opcode: u8,
    burst_complete: u8,
    offset: u32,
    data: Vec<u8>,
}

impl Payload {
    fn encode(&self) -> [u8; PAYLOAD_LEN] {
        let mut buf = [0u8; PAYLOAD_LEN];
        buf[0..2].copy_from_slice(&self.seq.to_le_bytes());
        buf[2] = self.session;
        buf[3] = self.opcode;
        buf[4] = self.size;
        buf[5] = self.req_opcode;
        buf[6] = self.burst_complete;
        buf[8..12].copy_from_slice(&self.offset.to_le_bytes());
        let n = self.data.len().min(PAYLOAD_DATA_LEN);
        buf[PAYLOAD_HEADER_LEN..PAYLOAD_HEADER_LEN + n].copy_from_slice(&self.data[..n]);
        buf
    }

    fn decode(p: &[u8; PAYLOAD_LEN]) -> Self {
        let size = p[4] as usize;
        let n = size.min(PAYLOAD_DATA_LEN);
        Self {
            seq: u16::from_le_bytes([p[0], p[1]]),
            session: p[2],
            opcode: p[3],
            size: p[4],
            req_opcode: p[5],
            burst_complete: p[6],
            offset: u32::from_le_bytes([p[8], p[9], p[10], p[11]]),
            data: p[PAYLOAD_HEADER_LEN..PAYLOAD_HEADER_LEN + n].to_vec(),
        }
    }
}

fn nak_message(p: &Payload) -> String {
    let code = p.data.first().copied().unwrap_or(NAK_NONE);
    let base = match code {
        NAK_NONE => "none",
        NAK_FAIL => "fail",
        NAK_FAIL_ERRNO => return format!("fail (errno {})", p.data.get(1).copied().unwrap_or(0)),
        NAK_INVALID_DATA_SIZE => "invalid data size",
        NAK_INVALID_SESSION => "invalid session",
        NAK_NO_SESSIONS_AVAILABLE => "no sessions available",
        NAK_EOF => "eof",
        NAK_UNKNOWN_COMMAND => "unknown command",
        NAK_FILE_EXISTS => "file exists",
        NAK_FILE_PROTECTED => "file protected",
        NAK_FILE_NOT_FOUND => "file not found",
        _ => "unknown",
    };
    format!("{base} ({code})")
}

type Conn = Arc<dyn MavConnection<MavMessage> + Send + Sync>;

struct FtpClient {
    conn: Conn,
    target_system: u8,
    target_component: u8,
    source_system: u8,
    source_component: u8,
    seq: u16,
    timeout: Duration,
    retries: u32,
    rx: Receiver<Payload>,
}

impl FtpClient {
    fn new(args: &FtpArgs) -> io::Result<Self> {
        let mut conn_concrete = mavlink::connect::<MavMessage>(&args.master)?;
        conn_concrete.set_protocol_version(MavlinkVersion::V2);
        let conn: Conn = Arc::from(conn_concrete);

        let (tx, rx): (Sender<Payload>, Receiver<Payload>) = channel();
        let conn_recv = conn.clone();
        let our_sys = args.source_system;
        let our_comp = args.source_component;

        // Heartbeat emitter — many FC configurations only route messages to peers they've
        // heard from. 1 Hz, GCS-typed.
        let conn_hb = conn.clone();
        thread::spawn(move || {
            let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_GCS,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            });
            let hdr = MavHeader {
                system_id: our_sys,
                component_id: our_comp,
                sequence: 0,
            };
            loop {
                if conn_hb.send(&hdr, &hb).is_err() {
                    break;
                }
                thread::sleep(Duration::from_secs(1));
            }
        });

        thread::spawn(move || {
            loop {
                match conn_recv.recv() {
                    Ok((_hdr, MavMessage::FILE_TRANSFER_PROTOCOL(d))) => {
                        // ArduPilot replies with target_system/component set to the requester.
                        let to_us_sys = d.target_system == 0 || d.target_system == our_sys;
                        let to_us_comp = d.target_component == 0 || d.target_component == our_comp;
                        if to_us_sys && to_us_comp {
                            if tx.send(Payload::decode(&d.payload)).is_err() {
                                break;
                            }
                        }
                    }
                    Ok(_) => {}
                    // Transient errors (parse, etc.) — keep going. Real socket failures
                    // recur and the next request's timeout surfaces them.
                    Err(_) => {}
                }
            }
        });

        let mut client = Self {
            conn,
            target_system: args.target_system,
            target_component: args.target_component,
            source_system: args.source_system,
            source_component: args.source_component,
            seq: 0,
            timeout: Duration::from_millis(args.timeout_ms),
            retries: args.retries,
            rx,
        };
        // Best-effort: clear any leftover sessions on the FC. Matches pymavlink behavior;
        // prevents NAK_NO_SESSIONS_AVAILABLE after aborted runs.
        client.reset_sessions();
        Ok(client)
    }

    fn reset_sessions(&mut self) {
        let req = Payload {
            seq: 0,
            session: 0,
            opcode: OP_RESET_SESSIONS,
            size: 0,
            req_opcode: 0,
            burst_complete: 0,
            offset: 0,
            data: Vec::new(),
        };
        let _ = self.request(req);
    }

    fn next_seq(&mut self) -> u16 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    fn header(&self) -> MavHeader {
        MavHeader {
            system_id: self.source_system,
            component_id: self.source_component,
            sequence: 0, // mavlink layer manages frame sequence
        }
    }

    fn send(&self, payload: &Payload) -> io::Result<()> {
        let msg = MavMessage::FILE_TRANSFER_PROTOCOL(FILE_TRANSFER_PROTOCOL_DATA {
            target_network: 0,
            target_system: self.target_system,
            target_component: self.target_component,
            payload: payload.encode(),
        });
        self.conn
            .send(&self.header(), &msg)
            .map_err(io::Error::other)?;
        Ok(())
    }

    /// Drain stale messages, send request, wait for response with matching seq.
    fn request(&mut self, mut payload: Payload) -> io::Result<Payload> {
        // Drop any stale incoming frames so we don't match against an old session.
        while self.rx.try_recv().is_ok() {}

        let mut last_err: io::Error = io::Error::other("no response");
        for _ in 0..=self.retries {
            payload.seq = self.next_seq();
            self.send(&payload)?;
            let deadline = Instant::now() + self.timeout;
            loop {
                let remaining = deadline.saturating_duration_since(Instant::now());
                match self.rx.recv_timeout(remaining) {
                    Ok(resp) if resp.seq == payload.seq => return Ok(resp),
                    Ok(_) => continue, // stale seq, keep waiting within this attempt
                    Err(RecvTimeoutError::Timeout) => {
                        last_err = io::Error::new(
                            io::ErrorKind::TimedOut,
                            format!("timed out waiting for opcode {}", payload.opcode),
                        );
                        break;
                    }
                    Err(RecvTimeoutError::Disconnected) => {
                        return Err(io::Error::other("ftp recv worker died"));
                    }
                }
            }
        }
        Err(last_err)
    }

    fn open_ro(&mut self, path: &str) -> io::Result<(u8, u32)> {
        let req = Payload {
            seq: 0,
            session: 0,
            opcode: OP_OPEN_FILE_RO,
            size: 0,
            req_opcode: 0,
            burst_complete: 0,
            offset: 0,
            data: path.as_bytes().to_vec(),
        };
        let resp = self.request(req)?;
        if resp.opcode == OP_NAK {
            return Err(io::Error::other(format!(
                "open '{path}' failed: {}",
                nak_message(&resp)
            )));
        }
        if resp.opcode != OP_ACK {
            return Err(io::Error::other(format!(
                "open '{path}': unexpected opcode {}",
                resp.opcode
            )));
        }
        if resp.data.len() < 4 {
            return Err(io::Error::other("open ack: short payload"));
        }
        let size = u32::from_le_bytes([resp.data[0], resp.data[1], resp.data[2], resp.data[3]]);
        Ok((resp.session, size))
    }

    fn terminate(&mut self, session: u8) {
        let req = Payload {
            seq: 0,
            session,
            opcode: OP_TERMINATE_SESSION,
            size: 0,
            req_opcode: 0,
            burst_complete: 0,
            offset: 0,
            data: Vec::new(),
        };
        // Best effort — ignore errors on cleanup.
        let _ = self.request(req);
    }

    fn get(&mut self, remote: &str, mut sink: Box<dyn Write>) -> io::Result<u64> {
        let (session, expected_size) = self.open_ro(remote)?;
        let result = self.burst_get(session, expected_size, &mut sink);
        self.terminate(session);
        let written = result?;
        sink.flush()?;
        Ok(written)
    }

    /// Drive a BurstReadFile session: one request, multiple ACK responses pushed by the
    /// server until burst_complete=1 or NAK_EOF. On stall (no progress within timeout),
    /// re-issue from the current offset.
    fn burst_get(
        &mut self,
        session: u8,
        expected_size: u32,
        sink: &mut Box<dyn Write>,
    ) -> io::Result<u64> {
        let mut offset: u32 = 0;
        let mut written: u64 = 0;

        'outer: loop {
            if expected_size > 0 && offset as u64 >= expected_size as u64 {
                return Ok(written);
            }

            // Drain stale frames before issuing.
            while self.rx.try_recv().is_ok() {}

            let mut req = Payload {
                seq: 0,
                session,
                opcode: OP_BURST_READ_FILE,
                size: PAYLOAD_DATA_LEN as u8,
                req_opcode: 0,
                burst_complete: 0,
                offset,
                data: Vec::new(),
            };
            req.seq = self.next_seq();
            self.send(&req)?;

            let burst_start = offset;
            loop {
                match self.rx.recv_timeout(self.timeout) {
                    Ok(p) if p.session != session => continue,
                    Ok(p) => match p.opcode {
                        OP_NAK => {
                            let code = p.data.first().copied().unwrap_or(NAK_NONE);
                            if code == NAK_EOF {
                                return Ok(written);
                            }
                            return Err(io::Error::other(format!(
                                "burst NAK at offset {}: {}",
                                p.offset,
                                nak_message(&p)
                            )));
                        }
                        OP_ACK => {
                            // ArduPilot sends in order. Accept only the chunk we expect,
                            // drop anything else (covers duplicates after a re-issue).
                            if p.offset == offset && !p.data.is_empty() {
                                sink.write_all(&p.data)?;
                                written += p.data.len() as u64;
                                offset = offset.saturating_add(p.data.len() as u32);
                            }
                            if expected_size > 0 && offset as u64 >= expected_size as u64 {
                                return Ok(written);
                            }
                            if p.burst_complete == 1 {
                                continue 'outer;
                            }
                        }
                        _ => continue,
                    },
                    Err(RecvTimeoutError::Timeout) => {
                        if offset == burst_start {
                            return Err(io::Error::new(
                                io::ErrorKind::TimedOut,
                                "burst stalled (no data received)",
                            ));
                        }
                        // Got partial data; re-issue from updated offset.
                        continue 'outer;
                    }
                    Err(RecvTimeoutError::Disconnected) => {
                        return Err(io::Error::other("ftp recv worker died"));
                    }
                }
            }
        }
    }

    fn ls(&mut self, path: &str) -> io::Result<()> {
        let mut entry_index: u32 = 0;
        loop {
            let req = Payload {
                seq: 0,
                session: 0,
                opcode: OP_LIST_DIRECTORY,
                size: 0,
                req_opcode: 0,
                burst_complete: 0,
                offset: entry_index,
                data: path.as_bytes().to_vec(),
            };
            let resp = self.request(req)?;
            if resp.opcode == OP_NAK {
                let code = resp.data.first().copied().unwrap_or(NAK_NONE);
                if code == NAK_EOF {
                    return Ok(());
                }
                return Err(io::Error::other(format!(
                    "ls '{path}': {}",
                    nak_message(&resp)
                )));
            }
            if resp.opcode != OP_ACK || resp.data.is_empty() {
                return Ok(());
            }
            let count = print_dir_entries(&resp.data);
            if count == 0 {
                return Ok(());
            }
            entry_index = entry_index.saturating_add(count);
        }
    }
}

// ArduPilot directory entries: \0-separated. Each entry starts with a type byte:
//   F<name>\t<size>   — regular file
//   D<name>           — directory
//   S                 — skip (still consumes a pagination slot)
fn print_dir_entries(data: &[u8]) -> u32 {
    let mut stdout = io::stdout().lock();
    let mut count = 0u32;
    for entry in data.split(|&b| b == 0) {
        if entry.is_empty() {
            continue;
        }
        match entry[0] {
            b'F' => {
                let s = String::from_utf8_lossy(&entry[1..]);
                if let Some((name, size)) = s.split_once('\t') {
                    let _ = writeln!(stdout, "{name:<32} {size}");
                } else {
                    let _ = writeln!(stdout, "{s}");
                }
                count += 1;
            }
            b'D' => {
                let s = String::from_utf8_lossy(&entry[1..]);
                let _ = writeln!(stdout, "{s}/");
                count += 1;
            }
            b'S' => {
                count += 1;
            }
            _ => {}
        }
    }
    count
}

pub fn run(args: FtpArgs) {
    let mut client = match FtpClient::new(&args) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: connect to '{}': {e}", args.master);
            std::process::exit(1);
        }
    };

    match &args.op {
        FtpOp::Get { remote, local } => {
            let sink: Box<dyn Write> = if local == "-" {
                Box::new(io::stdout().lock())
            } else {
                match File::create(local) {
                    Ok(f) => Box::new(f),
                    Err(e) => {
                        eprintln!("error: create '{local}': {e}");
                        std::process::exit(1);
                    }
                }
            };
            match client.get(remote, sink) {
                Ok(n) => {
                    eprintln!("got {remote} ({n} bytes)");
                }
                Err(e) => {
                    eprintln!("error: get '{remote}': {e}");
                    std::process::exit(1);
                }
            }
        }
        FtpOp::Ls { path } => {
            if let Err(e) = client.ls(path) {
                eprintln!("error: ls '{path}': {e}");
                std::process::exit(1);
            }
        }
    }
}
