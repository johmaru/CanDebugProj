use std::{
    io::{Read, Write},
    sync::{LazyLock, Mutex},
    time::Duration,
};

use eframe::{CreationContext, egui};
use serialport::SerialPortInfo;

static ENABLE_SERIAL_PORT: LazyLock<Mutex<Option<Vec<SerialPortInfo>>>> =
    LazyLock::new(|| Mutex::new(None));

#[derive(Default)]
struct ToolsGUI {
    ports: Vec<SerialPortInfo>,
    selected_port: Option<usize>,
    output: String,
    is_loading: bool,
}

impl ToolsGUI {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut app = Self::default();
        app
    }

    fn refresh_ports(&mut self) {
        let ports = serialport::available_ports().unwrap_or_default();
        if let Some(idx) = self.selected_port {
            if idx >= ports.len() {
                self.selected_port = None;
            }
        }
        self.ports = ports;
    }

    fn selected_port_name(&self) -> Option<&str> {
        self.selected_port
            .and_then(|idx| self.ports.get(idx))
            .map(|p| p.port_name.as_str())
    }

    fn read_from_serial_port(&mut self, port_name: &str) -> Result<String, String> {
        self.is_loading = true;

        let mut port = serialport::new(port_name, 115200)
            .timeout(Duration::from_secs(1))
            .open()
            .map_err(|err| err.to_string())?;

        std::thread::sleep(Duration::from_secs(2));

        let mut trash = [0u8; 256];
        let _ = port.read(&mut trash);

        port.write_all(b"uptime\n").map_err(|err| err.to_string())?;
        port.flush().map_err(|err| err.to_string())?;

        std::thread::sleep(Duration::from_secs(1));

        let mut buf = [0u8; 128];
        let n = port.read(&mut buf).map_err(|err| err.to_string())?;

        self.is_loading = false;

        println!("Raw bytes: {:?}", &buf[..n]);

        String::from_utf8(buf[..n].to_vec()).map_err(|err| err.to_string())
    }
}

impl eframe::App for ToolsGUI {
    fn update(&mut self, _ctx: &eframe::egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(_ctx, |ui| {
            ui.heading("Tools");
            if ui.button("Refresh").clicked() {
                self.refresh_ports();
            }

            let selected_text = self
                .selected_port_name()
                .map(ToOwned::to_owned)
                .unwrap_or_else(|| "Select a port".to_string());

            egui::ComboBox::from_label("Serial Port")
                .selected_text(selected_text)
                .show_ui(ui, |ui| {
                    for (idx, port) in self.ports.iter().enumerate() {
                        ui.selectable_value(&mut self.selected_port, Some(idx), &port.port_name);
                    }
                });

            if ui.button("Read uptime").clicked() {
                if let Some(name) = self.selected_port_name().map(str::to_string) {
                    self.output = match self.read_from_serial_port(&name) {
                        Ok(s) => s,
                        Err(e) => format!("Error: {}", e),
                    };
                }
            }

            ui.separator();

            let busy = self.is_loading;

            if busy {
                ui.spinner();
                ui.label("Loading...");
            } else {
                ui.label(&self.output);
            }
        });
    }
}

fn main() {
    let options = eframe::NativeOptions::default();
    let _ = eframe::run_native(
        "Tools",
        options,
        Box::new(|cc| Ok(Box::new(ToolsGUI::new(cc)))),
    );
}
