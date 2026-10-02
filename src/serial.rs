use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::{io, time::Duration};

pub fn list_ports() -> io::Result<Vec<String>> {
    let mut ports: Vec<_> = serialport::available_ports()
        .map_err(|e| io::Error::other(format!("enumerating serial ports: {e}")))?
        .into_iter()
        .map(|port| port.port_name)
        .filter(|name| name.starts_with("/dev/cu.") || name.starts_with("/dev/tty."))
        .collect();
    ports.sort_by_key(|name| (!name.starts_with("/dev/cu."), !name.contains("usbserial")));
    ports.dedup();
    Ok(ports)
}

pub fn open(explicit: Option<&str>, baud: u32) -> io::Result<(String, Box<dyn SerialPort>)> {
    let candidates = if let Some(path) = explicit {
        vec![path.to_owned()]
    } else {
        let ports = list_ports()?;
        if ports.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "no /dev/cu.* or /dev/tty.* serial devices found",
            ));
        }
        ports
    };
    let mut errors = Vec::new();
    for name in candidates {
        match serialport::new(&name, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::Hardware)
            .timeout(Duration::from_millis(20))
            .open()
        {
            Ok(port) => {
                port.clear(ClearBuffer::Input).map_err(io::Error::other)?;
                return Ok((name, port));
            }
            Err(error) => errors.push(format!("{name}: {error}")),
        }
    }
    Err(io::Error::other(format!(
        "could not open serial port: {}",
        errors.join("; ")
    )))
}
