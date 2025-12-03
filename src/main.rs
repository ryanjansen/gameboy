use gameboy::yb::Yarboy;

use std::io::{self};
use std::path::Path;
use winit::event_loop::{ControlFlow, EventLoop};

type Error = Box<dyn std::error::Error>;

fn read_rom<P: AsRef<Path>>(path: P) -> io::Result<Vec<u8>> {
    let buf = std::fs::read(path)?;
    Ok(buf)
}

fn main() -> std::result::Result<(), Error> {
    let path = std::env::args().nth(1).expect("No ROM path given");

    let rom = read_rom(path)?;

    let command = std::env::args().nth(2);

    let is_debug = match command {
        Some(string) => match string.as_str() {
            "debug" => true,
            _ => false,
        },
        None => false,
    };

    let mut yarboy: Yarboy = Yarboy::new(rom, is_debug);

    let event_loop = EventLoop::new()?;
    event_loop.set_control_flow(ControlFlow::Poll);
    event_loop.run_app(&mut yarboy)?;

    Ok(())
}
