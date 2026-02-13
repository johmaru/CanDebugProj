use std::time::Duration;

fn main() {
    let ports = serialport::available_ports().unwrap();
    for p in &ports {
        println!("Found port: {} ({:?})", p.port_name, p.port_type);
    }

    let mut port = serialport::new("COM3", 115200)
        .timeout(std::time::Duration::from_secs(2))
        .open()
        .expect("Failed to open port");

    std::thread::sleep(Duration::from_secs(2));

    let mut trash = [0u8; 256];
    let _ = port.read(&mut trash);

    port.write_all(b"uptime\n").unwrap();
    port.flush().unwrap();

    std::thread::sleep(Duration::from_secs(1));

    let mut buf = [0u8; 128];
    let n = port.read(&mut buf).unwrap();

    println!("Raw bytes: {:?}", &buf[..n]);

    match std::str::from_utf8(&buf[..n]) {
        Ok(s) => println!("Received: {}", s),
        Err(e) => println!("Failed to parse UTF-8: {}", e),
    }
}
