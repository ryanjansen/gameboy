use crate::utils::is_bit_set;

pub struct Interrupts {
    pub interrupt_enable: u8,
    pub interrupt_flag: u8,
}

pub enum Interrupt {
    VBlank,
    LCD,
    Timer,
    Serial,
    Joypad,
}

impl Interrupts {
    pub fn new() -> Interrupts {
        Interrupts {
            interrupt_enable: 0,
            interrupt_flag: 0xE1,
        }
    }

    pub fn get_enabled_interrupt(&mut self) -> Option<Interrupt> {
        if is_bit_set(self.interrupt_enable, 0) && is_bit_set(self.interrupt_flag, 0) {
            self.interrupt_flag &= 0b11111110;
            Some(Interrupt::VBlank)
        } else if is_bit_set(self.interrupt_enable, 1) && is_bit_set(self.interrupt_flag, 1) {
            self.interrupt_flag &= 0b11111101;
            Some(Interrupt::LCD)
        } else if is_bit_set(self.interrupt_enable, 2) && is_bit_set(self.interrupt_flag, 2) {
            self.interrupt_flag &= 0b11111011;
            Some(Interrupt::Timer)
        } else if is_bit_set(self.interrupt_enable, 3) && is_bit_set(self.interrupt_flag, 3) {
            self.interrupt_flag &= 0b11110111;
            Some(Interrupt::Serial)
        } else if is_bit_set(self.interrupt_enable, 4) && is_bit_set(self.interrupt_flag, 4) {
            self.interrupt_flag &= 0b11101111;
            Some(Interrupt::Joypad)
        } else {
            None
        }
    }

    pub fn request_interrupt(&mut self, interrupt: Interrupt) {
        match interrupt {
            Interrupt::VBlank => self.interrupt_flag |= 1,
            Interrupt::LCD => self.interrupt_flag |= 1 << 1,
            Interrupt::Timer => self.interrupt_flag |= 1 << 2,
            Interrupt::Serial => self.interrupt_flag |= 1 << 3,
            Interrupt::Joypad => self.interrupt_flag |= 1 << 4,
        }
    }

    pub fn is_pending(&self) -> bool {
        (self.interrupt_enable & self.interrupt_flag) != 0
    }
}

pub struct IME {
    is_set: bool,
    is_scheduled: bool,
}

impl IME {
    pub fn new() -> IME {
        IME {
            is_set: false,
            is_scheduled: false,
        }
    }

    pub fn schedule(&mut self) {
        self.is_scheduled = true;
    }

    pub fn reset(&mut self) {
        self.is_set = false;
        self.is_scheduled = false;
    }

    pub fn set_if_scheduled(&mut self) {
        if self.is_scheduled {
            self.is_set = true;
            self.is_scheduled = false;
        }
    }

    pub fn is_set(&self) -> bool {
        self.is_set
    }
}
