use std::collections::HashMap;
use std::fs::File;
use std::io::{self, BufReader, Write as _};

use chrono::{DateTime, Utc};
use clap::Parser;
use mavlink::ardupilotmega::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::peek_reader::PeekReader;
use mavlink::Message as _;
use serde_json::json;

#[derive(Parser)]
#[command(about = "Dump MAVLink tlog files as human-readable text")]
struct Cli {
    /// Path to .tlog file
    file: String,
    /// Comma-separated exact message type names (e.g. HEARTBEAT,ATTITUDE)
    #[arg(short, long, value_delimiter = ',')]
    filter: Vec<String>,
    /// Substring match on message name (case-insensitive, e.g. FLOAT matches NAMED_VALUE_FLOAT)
    #[arg(short = 'm', long = "match")]
    name_match: Option<String>,
    /// Output as JSON Lines (one JSON object per message)
    #[arg(short = 'j', long)]
    json: bool,
    /// Print only a summary table
    #[arg(short, long)]
    summary: bool,
}

fn main() {
    let cli = Cli::parse();
    let filter: Vec<String> = cli.filter.iter().map(|s| s.to_uppercase()).collect();
    let name_match = cli.name_match.as_ref().map(|s| s.to_uppercase());

    let file = File::open(&cli.file).unwrap_or_else(|e| {
        eprintln!("error: cannot open '{}': {e}", cli.file);
        std::process::exit(1);
    });
    let mut reader = PeekReader::new(BufReader::new(file));
    let mut stdout = io::stdout().lock();

    let mut counts: HashMap<&'static str, u64> = HashMap::new();
    let mut errors = 0u64;
    let mut first_ts = 0u64;
    let mut last_ts = 0u64;
    let mut last_accepted_ts = 0u64;
    let mut desync = false;

    loop {
        // read 8-byte BE timestamp; EOF = done, IO error = fatal
        let ts_us = match reader.read_exact(8) {
            Ok(b) => u64::from_be_bytes(b.try_into().unwrap()),
            Err(MessageReadError::Io(e))
                if e.kind() == io::ErrorKind::UnexpectedEof =>
            {
                break;
            }
            Err(e) => {
                eprintln!("error: reading timestamp: {e}");
                std::process::exit(1);
            }
        };

        match mavlink::read_any_msg::<MavMessage, _>(&mut reader) {
            Ok((header, msg)) => {
                let name = msg.message_name();
                if first_ts == 0 {
                    first_ts = ts_us;
                }
                last_ts = ts_us;

                let name_upper = name.to_uppercase();
                if !filter.is_empty() && !filter.contains(&name_upper) {
                    continue;
                }
                if let Some(ref pat) = name_match {
                    if !name_upper.contains(pat.as_str()) {
                        continue;
                    }
                }

                *counts.entry(name).or_insert(0) += 1;

                let ts_display = if desync
                    || (last_accepted_ts > 0
                        && ts_us.abs_diff(last_accepted_ts) > 86_400_000_000)
                {
                    desync = false;
                    "[desync]                   ".to_string()
                } else {
                    last_accepted_ts = ts_us;
                    format_timestamp(ts_us)
                };

                if !cli.summary {
                    let ok = if cli.json {
                        let obj = json!({
                            "ts": ts_display.trim(),
                            "ts_us": ts_us,
                            "sys": header.system_id,
                            "comp": header.component_id,
                            "msg": name,
                            "data": &msg,
                        });
                        writeln!(stdout, "{}", obj)
                    } else {
                        writeln!(
                            stdout,
                            "{} [{:>3}:{:<3}] {} {{{}}}",
                            ts_display,
                            header.system_id,
                            header.component_id,
                            name,
                            humanize_fields(&msg)
                        )
                    };
                    if ok.is_err() {
                        break; // broken pipe
                    }
                }
            }
            Err(e) => {
                errors += 1;
                desync = true;
                eprintln!("warning: frame parse error: {e}");
            }
        }
    }

    if cli.summary {
        print_summary(&counts, first_ts, last_ts, &filter, &name_match);
    }
    if errors > 0 {
        eprintln!("{errors} parse errors");
    }
}

fn format_timestamp(us: u64) -> String {
    let secs = (us / 1_000_000) as i64;
    let micros = (us % 1_000_000) as u32;
    let dt = DateTime::<Utc>::from_timestamp(secs, micros * 1000)
        .unwrap_or_default();
    dt.format("%Y-%m-%d %H:%M:%S%.3f").to_string()
}

fn humanize_fields(msg: &MavMessage) -> String {
    let dbg = format!("{:?}", msg);
    // Debug format: MSG_NAME { field: val, ... }
    if let Some(start) = dbg.find('{') {
        if let Some(end) = dbg.rfind('}') {
            return humanize_debug_fields(&dbg[start + 2..end]);
        }
    }
    dbg
}

// convert Debug byte arrays like [77, 97, 118, 0, 0] → "Mav" when printable ASCII
fn humanize_debug_fields(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    let bytes = s.as_bytes();
    while i < bytes.len() {
        if bytes[i] == b'[' {
            if let Some(end) = s[i..].find(']') {
                let inner = &s[i + 1..i + end];
                if let Some(text) = try_parse_char_array(inner) {
                    out.push('"');
                    out.push_str(&text);
                    out.push('"');
                    i += end + 1;
                    continue;
                }
            }
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

fn try_parse_char_array(inner: &str) -> Option<String> {
    if inner.is_empty() {
        return None;
    }
    let nums: Result<Vec<u8>, _> =
        inner.split(',').map(|s| s.trim().parse::<u8>()).collect();
    let nums = nums.ok()?;
    if nums.len() < 2 {
        return None;
    }
    let text: String = nums
        .iter()
        .take_while(|&&b| b != 0)
        .map(|&b| b as char)
        .collect();
    if text.is_empty() {
        return None;
    }
    if !text.chars().all(|c| c.is_ascii_graphic() || c == ' ') {
        return None;
    }
    Some(text)
}

fn print_summary(
    counts: &HashMap<&str, u64>,
    first_ts: u64,
    last_ts: u64,
    filter: &[String],
    name_match: &Option<String>,
) {
    let duration_s = last_ts.saturating_sub(first_ts) as f64 / 1_000_000.0;
    let mut entries: Vec<_> = counts.iter().collect();
    entries.sort_by(|a, b| b.1.cmp(a.1));

    if !filter.is_empty() {
        println!("Filter: {}", filter.join(", "));
    }
    if let Some(pat) = name_match {
        println!("Match: *{}*", pat);
    }
    println!("{:<24} {:>8}  {:>6}", "Message", "Count", "Hz");
    let total: u64 = entries.iter().map(|(_, c)| **c).sum();
    for (name, count) in &entries {
        let hz = if duration_s > 0.0 {
            **count as f64 / duration_s
        } else {
            0.0
        };
        println!("{:<24} {:>8}  {:>6.1}", name, count, hz);
    }
    println!("---");
    println!("Total: {total} messages, duration: {duration_s:.1}s");
}
