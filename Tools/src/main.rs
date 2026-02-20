use std::{
    io::{Read, Write},
    result,
    sync::{LazyLock, Mutex},
    time::Duration,
};

use eframe::{CreationContext, egui};
use serialport::SerialPortInfo;

static ENABLE_SERIAL_PORT: LazyLock<Mutex<Option<Vec<SerialPortInfo>>>> =
    LazyLock::new(|| Mutex::new(None));
#[derive(Default)]
struct Output {
    content: String,
    is_error: bool,
    command: CommandType,
}

#[derive(Default)]
struct ToolsGUI {
    ports: Vec<SerialPortInfo>,
    selected_port: Option<usize>,
    output: Output,
    is_loading: bool,
    result_rx: Option<std::sync::mpsc::Receiver<Result<String, String>>>,
    show_help_window: bool,
}

#[derive(Clone, Copy)]
enum CommandType {
    Uptime,
    Help,
}

impl Default for CommandType {
    fn default() -> Self {
        CommandType::Help
    }
}

impl ToolsGUI {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut app = Self::default();
        app.show_help_window = false;
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

    fn start_read_from_serial_port(
        &mut self,
        port_name: String,
        ctx: egui::Context,
        command_type: CommandType,
    ) {
        self.is_loading = true;
        self.output = Output {
            content: String::new(),
            is_error: false,
            command: command_type,
        };

        let (tx, rx) = std::sync::mpsc::channel();
        self.result_rx = Some(rx);

        std::thread::spawn(move || {
            let result = (|| -> Result<String, String> {
                let mut port = serialport::new(port_name, 115200)
                    .timeout(Duration::from_secs(1))
                    .open()
                    .map_err(|err| err.to_string())?;

                std::thread::sleep(Duration::from_secs(2));

                let mut trash = [0u8; 256];
                let _ = port.read(&mut trash);

                match command_type {
                    CommandType::Uptime => {
                        port.write_all(b"uptime\n").map_err(|err| err.to_string())?;
                    }
                    CommandType::Help => {
                        port.write_all(b"help\n").map_err(|err| err.to_string())?;
                    }
                }
                port.flush().map_err(|err| err.to_string())?;

                std::thread::sleep(Duration::from_secs(1));

                let mut buf = [0u8; 128];
                let n = port.read(&mut buf).map_err(|err| err.to_string())?;

                println!("Raw bytes: {:?}", &buf[..n]);

                String::from_utf8(buf[..n].to_vec()).map_err(|err| err.to_string())
            })();

            let _ = tx.send(result);
            ctx.request_repaint();
        });
    }
}

impl eframe::App for ToolsGUI {
    fn update(&mut self, _ctx: &eframe::egui::Context, _frame: &mut eframe::Frame) {
        if let Some(rx) = &self.result_rx {
            if let Ok(result) = rx.try_recv() {
                self.is_loading = false;
                let command = self.output.command;
                self.output = match result {
                    Ok(s) => Output {
                        content: s,
                        is_error: false,
                        command,
                    },
                    Err(e) => Output {
                        content: format!("Error: {}", e),
                        is_error: true,
                        command,
                    },
                };

                if matches!(command, CommandType::Help) {
                    self.show_help_window = true;
                }
            }
        }

        if self.show_help_window {
            egui::Window::new("Help")
                .open(&mut self.show_help_window)
                .resizable(true)
                .show(_ctx, |ui| {
                    ui.label(&self.output.content);
                });
        }

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
                    self.start_read_from_serial_port(name, _ctx.clone(), CommandType::Uptime);
                }
            }

            if ui.button("Help").clicked() {
                if let Some(name) = self.selected_port_name().map(str::to_string) {
                    self.start_read_from_serial_port(name, _ctx.clone(), CommandType::Help);
                }
            }

            ui.separator();

            let busy = self.is_loading;

            if !busy {
                match self.output.command {
                    CommandType::Uptime => {
                        ui.label(format!("Uptime: {}", self.output.content));
                    }
                    CommandType::Help => {}
                }
            } else {
                ui.spinner();
                ui.label("Loading...");
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
