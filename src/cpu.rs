use crate::instruction::*;
use crate::register::Registers;
use std::fmt;

struct Memory {
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
    fn new(rom: [u8; 0xFFFF]) -> Memory {
        Memory { memory: rom }
    }

    fn read_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }
}

#[derive(Debug)]
pub struct CPU {
    registers: Registers,
    memory: Memory,
}

impl CPU {
    pub fn new(rom: [u8; 0xFFFF]) -> CPU {
        CPU {
            registers: Registers::new(),
            memory: Memory::new(rom),
        }
    }

    pub fn run(&mut self) {
        loop {
            self.step();
        }
    }

    fn step(&mut self) {
        let mut instruction_byte = self.memory.read_byte(self.registers.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.memory.read_byte(self.registers.pc + 1);
        }

        let instruction = Instruction::decode(instruction_byte, prefixed);
        let instr_length = self.execute(instruction);
        self.registers.pc += instr_length;
    }

    fn execute(&self, instruction: Instruction) -> u16 {
        use Instruction::*;

        match instruction {
            NOP => 1,
            STOP => 2,
            HALT => todo!("Halt"),
            LD(dest, source) => match dest {
                Dest::Indirect(Addr::Imm16) => 10,
            },
            INVALID => panic!("Invalid instruction: {:?}", instruction),
            _ => todo!("Add catchall"),
        }
    }
}
