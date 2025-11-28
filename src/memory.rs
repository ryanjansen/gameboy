use std::fmt;

pub struct Memory {
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
    pub fn new(rom: [u8; 0xFFFF]) -> Memory {
        Memory { memory: rom }
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        if address == 0xFF44 {
            return 0x90;
        } else {
            self.memory[address as usize]
        }
    }

    pub fn write_byte(&mut self, address: u16, val: u8) {
        if address < 0xFFFF {
            self.memory[address as usize] = val
        } else {
            panic!("Invalid memory address {:04X}", address)
        }
    }

    pub fn get_dma_oam(&self, start_addr: u16) -> [u8; 0xA0] {
        let start = start_addr as usize;
        let end = start + 0x9F;

        let mut oam = [0u8; 0xA0];

        oam.copy_from_slice(&self.memory[start..=end]);

        oam
    }
}
