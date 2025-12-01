use crate::interrupts::{Interrupt, Interrupts};

#[derive(Debug)]
pub struct Timer {
    pub sysclock: u16, // FF04, Div (upper 8 bits)
    tima: u8,          // FF05, Timer Counter
    tma: u8,           // FF06, Timer Modulo
    tac: u8,           // FF07, Timer Control
    is_tima_reloading: bool,
    is_interrupt_scheduled: bool,
    last_bit: u8,
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            sysclock: 0xAB << 6 | 0b1111110000,
            tima: 0,
            tma: 0,
            tac: 0xF8,
            is_tima_reloading: false,
            is_interrupt_scheduled: false,
            last_bit: 0,
        }
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            0xFF04 => ((self.sysclock >> 6) & 0xFF) as u8,
            0xFF05 => self.tima,
            0xFF06 => self.tma,
            0xFF07 => self.tac,
            _ => unreachable!("Invalid address for Timer"),
        }
    }

    pub fn write_byte(&mut self, address: u16, val: u8) {
        match address {
            0xFF04 => self.set_sysclock(0),
            0xFF05 => {
                if !self.is_tima_reloading {
                    self.tima = val;
                }

                if self.is_interrupt_scheduled {
                    self.is_interrupt_scheduled = false;
                }
            }
            0xFF06 => {
                if self.is_tima_reloading {
                    self.tima = val
                }
                self.tma = val;
            }
            0xFF07 => {
                let last_bit = self.last_bit;
                self.last_bit &= (val & 0b100) >> 2;
                self.inc_tima(last_bit, self.last_bit);
                self.tac = val;
            }
            _ => unreachable!("Invalid address for Timer"),
        }
    }

    fn inc_tima(&mut self, before: u8, after: u8) {
        if (before == 1) && (after == 0) {
            self.tima = self.tima.wrapping_add(1);

            if self.tima == 0 {
                self.is_interrupt_scheduled = true;
            }
        }
    }

    fn set_sysclock(&mut self, val: u16) {
        self.sysclock = val;

        let watched_bit = match self.tac & 0b11 {
            0b00 => (self.sysclock >> 7) & 1,
            0b01 => (self.sysclock >> 1) & 1,
            0b10 => (self.sysclock >> 3) & 1,
            0b11 => (self.sysclock >> 5) & 1,
            _ => unreachable!(),
        } as u8;

        let new_bit = watched_bit & ((self.tac & 0b100) >> 2);

        self.inc_tima(self.last_bit, new_bit);
        self.last_bit = new_bit;
    }

    pub fn tick(&mut self, interrupts: &mut Interrupts) {
        self.is_tima_reloading = false;

        if self.is_interrupt_scheduled {
            interrupts.request_interrupt(Interrupt::Timer);
            self.tima = self.tma;
            self.is_interrupt_scheduled = false;
            self.is_tima_reloading = true;
        }

        self.set_sysclock(self.sysclock.wrapping_add(1));
    }
}
