use crate::memory::MemoryBus;

#[derive(Default)]
pub struct MBC1 {
    rom: Vec<u8>,
    rom_bank: u8,
    ram_enable: bool,
    ram: Vec<u8>,
    ram_bank: u8,
    mode: bool,
}

impl MBC1 {
    pub fn new(rom: Vec<u8>, ram_size: usize) -> Self {
        let ram = vec![0; ram_size];

        Self {
            ram,
            rom,
            rom_bank: 1,
            ..Default::default()
        }
    }
}

impl MemoryBus for MBC1 {
    fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x0000..=0x3FFF => self.rom[address as usize],
            0x4000..=0x7FFF => {
                let offset = 0x4000 * self.rom_bank as usize;
                self.rom[offset + (address as usize - 0x4000)]
            }
            0xA000..=0xBFFF => {
                if self.ram_enable {
                    let offset = self.ram_bank as usize * 8192;
                    self.ram[offset + (address as usize - 0xA000)]
                } else {
                    0
                }
            }
            _ => 0,
        }
    }

    fn write_byte(&mut self, address: u16, val: u8) {
        match address {
            0x0000..=0x1FFF => self.ram_enable = val & 0x0F == 0x0A,
            0x2000..=0x3FFF => {
                let bank_num = if val == 0 { 1 } else { val };
                self.rom_bank = (self.rom_bank & 0b0110_0000) | (bank_num & 0b0001_1111)
            }
            0x4000..=0x5FFF => {
                if self.mode {
                    self.ram_bank = val & 0b11;
                } else {
                    self.rom_bank = (self.rom_bank & 0b0001_1111) | (val & 3) << 5;
                }
            }
            0x6000..=0x7FFF => self.mode = val == 1,
            _ => {}
        }
    }
}
