use crate::instruction::Instruction;
use crate::register::Registers;
use std::fmt;

struct MemoryBus {
    memory: [u8; 0xFFFF],
}

impl fmt::Debug for MemoryBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MemoryBus")
            .field("length", &self.memory.len())
            .finish()
    }
}

impl MemoryBus {
    fn new(rom: [u8; 0xFFFF]) -> MemoryBus {
        MemoryBus { memory: rom }
    }

    fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }
}

#[derive(Debug)]
pub struct CPU {
    registers: Registers,
    pc: u16,
    sp: u16,
    bus: MemoryBus,
}

impl CPU {
    pub fn new(rom: [u8; 0xFFFF]) -> CPU {
        CPU {
            registers: Registers::new(),
            pc: 0x0100,
            sp: 0xFFFE,
            bus: MemoryBus::new(rom),
        }
    }

    pub fn run(&self) {}

    fn execute(&mut self, instruction: Instruction) -> u16 {
        10
    }

    fn step(&mut self) {
        let mut instruction_byte = self.bus.read_byte(self.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.bus.read_byte(self.pc + 1);
        }

        let instruction = Instruction::decode(instruction_byte, prefixed);
        let instr_length = self.execute(instruction);
        self.pc += instr_length;
    }
}
