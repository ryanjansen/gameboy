use gameboy::cpu::CPU;

use std::fs;
use std::io::{self, Read};
use std::path::Path;

const ROM_SIZE: usize = 0xFFFF;

fn read_rom<P: AsRef<Path>>(path: P) -> io::Result<[u8; ROM_SIZE]> {
    let mut file = fs::File::open(path)?;
    let mut buffer = [0u8; ROM_SIZE];
    let _bytes_read = file.read(&mut buffer)?;
    Ok(buffer)
}

fn main() -> io::Result<()> {
    let path = std::env::args().nth(1).expect("No ROM path given");
    let mut rom = read_rom(path)?;
    rom[0xFF80..=0xFFFE].fill(0xFF);
    let mut my_cpu: CPU = CPU::new(rom);
    my_cpu.run_and_log_state();
    Ok(())
}
