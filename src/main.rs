use clap::{Parser, Subcommand};

mod dump;
mod ftp;

#[derive(Parser)]
#[command(name = "mavlink-tools", version, about = "MAVLink CLI tools (dump, ftp)")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Dump tlog files as text or JSON Lines
    Dump(dump::DumpArgs),
    /// MAVLink FTP client
    Ftp(ftp::FtpArgs),
}

fn main() {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Dump(args) => dump::run(args),
        Cmd::Ftp(args) => ftp::run(args),
    }
}
