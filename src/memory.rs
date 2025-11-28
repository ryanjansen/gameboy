use std::fmt;

use crate::mbc::MBC1;

// TODO: Split memory into distinct regions
pub struct Memory {
    cartridge: Box<dyn MemoryBus>,
    memory: [u8; 0xFFFF],
}

impl fmt::Debug for Memory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Memory")
            .field("length", &self.memory.len())
            .finish()
    }
}

impl Memory {
    pub fn new(rom: Vec<u8>) -> Memory {
        let cartridge = Memory::load_cartridge(rom);

        Memory {
            cartridge: cartridge,
            memory: [0u8; 0xFFFF],
        }
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x0000..=0x7FFF | 0xA000..=0xBFFF => self.cartridge.read_byte(address),
            _ => self.memory[address as usize],
        }
    }

    pub fn write_byte(&mut self, address: u16, val: u8) {
        match address {
            0x0000..=0x7FFF | 0xA000..=0xBFFF => self.cartridge.write_byte(address, val),
            _ => self.memory[address as usize] = val,
        }
    }

    pub fn get_dma_oam(&self, start_addr: u16) -> [u8; 0xA0] {
        let mut oam = [0u8; 0xA0];

        for i in 0..oam.len() {
            oam[i] = self.read_byte(start_addr + (i as u16));
        }

        oam
    }

    fn load_cartridge(buffer: Vec<u8>) -> Box<dyn MemoryBus> {
        match buffer[0x147] {
            0x00 => Box::new(ROM::new(buffer)),
            0x01 => Box::new(MBC1::new(buffer, 0)),
            0x02 => {
                let ram_size = Memory::get_ram_size(buffer[0x149]);
                Box::new(MBC1::new(buffer, ram_size))
            }
            _ => panic!("INVALID CARTRIDGE TYPE"),
        }
    }

    fn get_ram_size(size: u8) -> usize {
        let kb = 1024;
        match size {
            0 => 0,
            1 => 2 * kb,
            3 => 8 * kb,
            4 => 32 * kb,
            5 => 64 * kb,
            _ => 0,
        }
    }
}

struct ROM(Vec<u8>);

impl ROM {
    pub fn new(rom: Vec<u8>) -> ROM {
        ROM(rom)
    }
}

impl MemoryBus for ROM {
    fn read_byte(&self, address: u16) -> u8 {
        self.0[address as usize]
    }

    fn write_byte(&mut self, _: u16, _: u8) {}
}

pub trait MemoryBus {
    fn read_byte(&self, address: u16) -> u8;
    fn write_byte(&mut self, address: u16, val: u8);
}
