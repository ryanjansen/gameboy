use crate::utils::is_bit_set;

pub struct Joypad {
    up: bool,
    down: bool,
    left: bool,
    pub right: bool,
    pub a: bool,
    b: bool,
    start: bool,
    select: bool,
    is_select_buttons: bool,
    is_select_d_pad: bool,
}

impl Joypad {
    pub fn new() -> Joypad {
        Joypad {
            up: false,
            down: false,
            left: false,
            right: false,
            a: false,
            b: false,
            start: false,
            select: false,
            is_select_buttons: false,
            is_select_d_pad: false,
        }
    }

    pub fn set_key_pressed(&mut self, key: JoypadKey) {
        use JoypadKey::*;
        match key {
            Up => self.up = true,
            Down => self.down = true,
            Left => self.left = true,
            Right => self.right = true,
            A => self.a = true,
            B => self.b = true,
            Start => self.start = true,
            Select => self.select = true,
        }
    }

    pub fn set_key_released(&mut self, key: JoypadKey) {
        use JoypadKey::*;
        match key {
            Up => self.up = false,
            Down => self.down = false,
            Left => self.left = false,
            Right => self.right = false,
            A => self.a = false,
            B => self.b = false,
            Start => self.start = false,
            Select => self.select = false,
        }
    }

    pub fn read(&self) -> u8 {
        if self.is_select_d_pad {
            let lower_nibble = (if self.right { 0 } else { 1 })
                | (if self.left { 0 } else { 1 }) << 1
                | (if self.up { 0 } else { 1 }) << 2
                | (if self.down { 0 } else { 1 }) << 3;

            let upper_nibble = self.get_upper_nibble();

            upper_nibble | lower_nibble
        } else if self.is_select_buttons {
            let lower_nibble = (if self.a { 0 } else { 1 })
                | (if self.b { 0 } else { 1 }) << 1
                | (if self.start { 0 } else { 1 }) << 2
                | (if self.select { 0 } else { 1 }) << 3;

            let upper_nibble = self.get_upper_nibble();
            upper_nibble | lower_nibble
        } else {
            let upper_nibble = self.get_upper_nibble();
            upper_nibble | 0x0F
        }
    }

    pub fn write(&mut self, val: u8) {
        self.is_select_buttons = !is_bit_set(val, 5);
        self.is_select_d_pad = !is_bit_set(val, 4);
    }

    fn get_upper_nibble(&self) -> u8 {
        0_u8 | (if self.is_select_buttons { 0 } else { 1 }) << 5
            | (if self.is_select_d_pad { 0 } else { 1 }) << 4
    }
}

pub enum JoypadKey {
    Up,
    Down,
    Left,
    Right,
    A,
    B,
    Start,
    Select,
}
