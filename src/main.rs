mod serial;
mod terminal;

use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("serial-zmodem: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut port = None;
    let mut baud = 9600u32;
    let mut send = None;
    let mut receive_dir = PathBuf::from(".");
    let mut list_ports = false;
    let mut experimental_baud = false;
    let mut max_baud = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--port" => port = Some(args.next().ok_or("--port needs a device path")?),
            "--baud" => baud = args.next().ok_or("--baud needs a rate")?.parse()?,
            "--send" => send = Some(PathBuf::from(args.next().ok_or("--send needs a file")?)),
            "--receive-dir" => {
                receive_dir = PathBuf::from(args.next().ok_or("--receive-dir needs a path")?)
            }
            "--list-ports" => list_ports = true,
            "--experimental-baud" => experimental_baud = true,
            "--max-baud" => {
                max_baud = Some(
                    args.next()
                        .ok_or("--max-baud needs a rate")?
                        .parse::<u32>()?,
                )
            }
            "-h" | "--help" => {
                println!(
                    "Usage: serial-zmodem [--port DEVICE] [--baud RATE] [--send FILE] [--receive-dir DIR] [--experimental-baud] [--max-baud RATE] [--list-ports]\n\n--experimental-baud: train ZRECVX/ZRECVY before upload; start and restore the console at 9600.\n--max-baud: cap training at 9600, 19200, 38400, 57600, or 115200 (default).\nKeys: Ctrl-] then ? for commands. No bytes are sent on startup."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    if list_ports {
        for path in serial::list_ports()? {
            println!("{path}");
        }
        return Ok(());
    }
    if !matches!(baud, 1200 | 2400 | 9600 | 19200 | 38400 | 57600 | 115200) {
        return Err(format!("unsupported baud rate: {baud}").into());
    }
    if experimental_baud && baud != 9600 {
        return Err("--experimental-baud requires --baud 9600".into());
    }
    if let Some(maximum) = max_baud {
        if !experimental_baud || !matches!(maximum, 9600 | 19200 | 38400 | 57600 | 115200) {
            return Err(
                "--max-baud requires experimental training and a supported rate >=9600".into(),
            );
        }
    }
    let (port_name, device) = serial::open(port.as_deref(), baud)?;
    eprintln!("Connected to {port_name} at {baud} baud (8N1, RTS/CTS). Ctrl-] ? for help.");
    terminal::run(
        device,
        send,
        receive_dir,
        experimental_baud.then_some(max_baud.unwrap_or(115200)),
    )?;
    Ok(())
}
