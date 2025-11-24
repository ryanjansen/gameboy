use gameboy::cpu::CPU;

use std::fs;
use std::io::{self, Read};
use std::path::Path;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::window::{Window, WindowId};
use winit::dpi::LogicalSize;



const ROM_SIZE: usize = 0xFFFF;
const WIDTH: usize = 160;
const HEIGHT: usize = 144;

struct Yarboy {
    window: Option<Window>,
    gameboy: CPU
}

fn read_rom<P: AsRef<Path>>(path: P) -> io::Result<[u8; ROM_SIZE]> {
    let mut file = fs::File::open(path)?;
    let mut buffer = [0u8; ROM_SIZE];
    let _bytes_read = file.read(&mut buffer)?;
    Ok(buffer)
}

fn main() -> io::Result<()> {

    // Create pixels, yarboy, run event loop
    // 
    let path = std::env::args().nth(1).expect("No ROM path given");

    let mut rom = read_rom(path)?;

    rom[0xFF80..=0xFFFE].fill(0xFF);

    let window = {
        let size = LogicalSize::new(WIDTH as f64, HEIGHT as f64);
    }

    let mut my_cpu: CPU = CPU::new(rom);

    my_cpu.run_and_log_state();
    Ok(())
}

impl Yarboy {
    // new (create cpu)
    //
    // run
    //
    // WHEN TO RENDER??
}

impl ApplicationHandler for Yarboy {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.window = Some(event_loop.create_window(Window::default_attributes()).unwrap());
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit()
            }
            WindowEvent::RedrawRequested => {
                // Pixels draw                
            }
            _ => ()
        }
    }
}
