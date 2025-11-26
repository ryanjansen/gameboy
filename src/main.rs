use gameboy::yb::Yarboy;

use std::fs;
use std::io::{self, Read};
use std::path::Path;
use winit::event_loop::{ControlFlow, EventLoop};

type Error = Box<dyn std::error::Error>;

const ROM_SIZE: usize = 0xFFFF;

fn read_rom<P: AsRef<Path>>(path: P) -> io::Result<[u8; ROM_SIZE]> {
    let mut file = fs::File::open(path)?;
    let mut buffer = [0u8; ROM_SIZE];
    let _bytes_read = file.read(&mut buffer)?;
    Ok(buffer)
}

fn main() -> std::result::Result<(), Error> {
    let path = std::env::args().nth(1).expect("No ROM path given");
    let mut rom = read_rom(path)?;
    rom[0xFF80..=0xFFFE].fill(0xFF);

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
