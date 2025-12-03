use crate::memory::MemoryBus;

use std::{path::PathBuf, time::SystemTime};

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

struct RealTimeClock {
    seconds: u8,
    mintues: u8,
    hours: u8,
    dl: u8,
    dh: u8,
    pub zero: u64,
}

impl RealTimeClock {
    fn new(rtc_path: Option<PathBuf>) -> Option<RealTimeClock> {
        match rtc_path {
            Some(path) => {
                let zero = match std::fs::read(path) {
                    Ok(f) => {
                        let mut b = [0_u8; 8];
                        b.copy_from_slice(&f);
                        u64::from_be_bytes(b)
                    }
                    Err(_) => SystemTime::now()
                        .duration_since(SystemTime::UNIX_EPOCH)
                        .unwrap()
                        .as_secs(),
                };
                Some(Self {
                    seconds: 0,
                    mintues: 0,
                    hours: 0,
                    dl: 0,
                    dh: 0,
                    zero,
                })
            }
            None => None,
        }
    }

    fn step(&mut self) {
        let duration = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_secs()
            - self.zero;

        self.seconds = (duration % 60) as u8;
        self.mintues = (duration / 60 % 60) as u8;
        self.hours = (duration / 3600 % 24) as u8;

        let days = (duration / 3600 / 24) as u16;
        self.dl = (days % 256) as u8;
        match days {
            0x0..=0xFF => {}
            0x100..=0x1FF => {
                self.dh |= 1;
            }
            _ => {
                self.dh |= 1;
                self.dh |= 0x80;
            }
        }
    }
}

/*
The Clock Counter Registers
08h  RTC S   Seconds   0-59 (0-3Bh)
09h  RTC M   Minutes   0-59 (0-3Bh)
0Ah  RTC H   Hours     0-23 (0-17h)
0Bh  RTC DL  Lower 8 bits of Day Counter (0-FFh)
0Ch  RTC DH  Upper 1 bit of Day Counter, Carry Bit, Halt Flag
      Bit 0  Most significant bit of Day Counter (Bit 8)
      Bit 6  Halt (0=Active, 1=Stop Timer)
      Bit 7  Day Counter Carry Bit (1=Counter Overflow)
*/

impl MemoryBus for RealTimeClock {
    fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x08 => self.seconds,
            0x09 => self.mintues,
            0x0A => self.hours,
            0x0B => self.dl,
            0x0C => self.dh,
            _ => panic!("invalid address rtc (read): {:#2X}", address),
        }
    }

    fn write_byte(&mut self, address: u16, b: u8) {
        match address {
            0x08 => self.seconds = b,
            0x09 => self.mintues = b,
            0x0A => self.hours = b,
            0x0B => self.dl = b,
            0x0C => self.dh = b,
            _ => panic!("invalid address rtc (write): {:#2X}", address),
        }
    }
}

pub struct MBC3 {
    rom: Vec<u8>,
    rom_bank: usize,

    ram: Vec<u8>,
    ram_bank: usize,
    ram_enable: bool,

    rtc: Option<RealTimeClock>,
    save_path: Option<PathBuf>,
}

impl MBC3 {
    pub fn new(
        rom: Vec<u8>,
        ram_size: usize,
        save_path: Option<PathBuf>,
        rtc_path: Option<PathBuf>,
    ) -> Self {
        let ram = match save_path {
            Some(ref path) => vec![0; ram_size], // TODO: implement saves
            None => vec![0; ram_size],
        };

        Self {
            ram,
            ram_bank: 1,
            rom,
            rom_bank: 0,
            ram_enable: false,
            save_path,
            rtc: RealTimeClock::new(rtc_path),
        }
    }
}

impl MemoryBus for MBC3 {
    fn read_byte(&self, address: u16) -> u8 {
        match address {
            // 0000-3FFF - ROM Bank 00 (Read Only)
            0x0000..=0x3FFF => self.rom[address as usize],
            // 4000-7FFF - ROM Bank 01-7F (Read Only)
            0x4000..=0x7FFF => {
                let offset = 0x4000 * self.rom_bank;
                self.rom[offset + (address as usize - 0x4000)]
            }
            // A000-BFFF - RAM Bank 00-03, if any (Read/Write)
            0xA000..=0xBFFF => {
                if self.ram_enable {
                    if self.ram_bank <= 3 {
                        let offset = self.ram_bank * 0x2000;
                        self.ram[offset + (address as usize - 0xA000)]
                    } else {
                        match &self.rtc {
                            Some(rtc) => rtc.read_byte(self.ram_bank as u16),
                            None => 0,
                        }
                    }
                } else {
                    0
                }
            }
            _ => 0,
        }
    }

    fn write_byte(&mut self, address: u16, b: u8) {
        match address {
            // 0000-1FFF - RAM and Timer Enable (Write Only)
            0x0000..=0x1FFF => self.ram_enable = b & 0xF == 0xA,
            // 2000-3FFF - ROM Bank Number (Write Only)
            // Whole 7 bits of the RAM Bank Number are written directly to this address.
            0x2000..=0x3FFF => {
                let n = b & 0b0111_1111;
                let n = if n == 0 { 1 } else { n };
                self.rom_bank = n as usize;
            }
            // 4000-5FFF - RAM Bank Number - or - RTC Register Select (Write Only)
            0x4000..=0x5FFF => {
                let n = b & 0x0F;
                self.ram_bank = n as usize;
            }
            // 6000-7FFF - Latch Clock Data (Write Only)
            0x6000..=0x7FFF => {
                if b & 1 != 0 && self.rtc.is_some() {
                    self.rtc.as_mut().unwrap().step();
                }
            }
            0xA000..=0xBFFF => {
                if self.ram_enable {
                    if self.ram_bank <= 3 {
                        let offset = 0x2000 * self.ram_bank;
                        self.ram[offset + (address as usize - 0xA000)] = b;
                    } else {
                        match &mut self.rtc {
                            Some(rtc) => rtc.write_byte(address, b),
                            None => {}
                        }
                    }
                }
            }
            _ => {}
        }
    }
}
