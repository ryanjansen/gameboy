pub fn is_bit_set(val: u8, bit_index: u8) -> bool {
    (val & (1 << bit_index)) != 0
}

pub fn set_bit(val: u8, bit_index: u8) -> u8 {
    val | (1 << bit_index)
}

pub fn reset_bit(val: u8, bit_index: u8) -> u8 {
    val & !(1 << bit_index)
}
