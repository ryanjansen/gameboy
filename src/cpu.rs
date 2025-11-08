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

    fn execute(&mut self, instruction: Instruction) -> u16 {
        use Instruction::*;

        match instruction {
            NOP => 1,
            STOP => 2,
            HALT => todo!("Halt"),
            LD(dest, src) => self.load(dest, src),
            LDH(dest, src) => self.load_with_io(dest, src),
            ADD(dest, operand) => self.add(dest, operand),
            ADC(operand) => self.adc(operand),
            SUB(operand) => self.sub(operand),
            SBC(operand) => self.sbc(operand),
            AND(operand) => self.and(operand),
            XOR(operand) => self.xor(operand),
            OR(operand) => self.or(operand),
            CP(operand) => self.cp(operand),
            INC(operand) => self.inc(operand),
            DEC(operand) => self.dec(operand),
            RLCA => self.rlca(),
            RRCA => self.rrca(),
            RLA => self.rla(),
            RRA => self.rra(),
            DAA => self.daa(),
            CPL => self.cpl(),
            SCF => self.scf(),
            CCF => self.ccp(),
            RLC(operand) => self.rlc(operand),
            RRC(operand) => self.rrc(operand),
            RL(operand) => self.rl(operand),
            SLA(operand) => self.sla(operand),
            SRA(operand) => self.sra(operand),
            SWAP(operand) => self.swap(operand),
            SRL(operand) => self.srl(operand),
            BIT(bit_index, operand) => self.bit(bit_index, operand),
            RES(bit_index, operand) => self.res(bit_index, operand),
            SET(bit_index, operand) => self.set(bit_index, operand),
            JP(tgt) => self.jump(tgt),
            JPCOND(cond, tgt) => self.jump_cond(cond, tgt),
            RET => self.ret(),
            RETI => self.reti(),
            RETCOND(cond) => self.ret_cond(cond),
            RST(tgt) => self.rst(tgt),
            POP(reg) => self.pop(reg),
            PUSH(reg) => self.push(reg),
            DI => self.di(),
            EI => self.ei(),
            INVALID => panic!("Invalid instruction: {:?}", instruction),
            _ => todo!("Add catchall"),
        }
    }

    fn get_imm8(&self) -> u8 {
        self.memory.read_byte(self.registers.pc + 1)
    }

    fn get_imm16(&self) -> u16 {
        let lower = self.memory.read_byte(self.registers.pc + 1);
        let upper = self.memory.read_byte(self.registers.pc + 2);
        (upper << 8 | lower) as u16
    }

    fn load(&mut self, dest: Dest, src: Source) -> u16 {
        // dest: [imm16], r16, r8, [r16mem], A, HL, SP
        // src: imm8, imm16, SP, A, [r16mem],
        // match dest {}
        0
    }

    fn load_with_io(&mut self, dest: Dest, src: Source) -> u16 {
        0
    }

    fn add(&mut self, dest: Dest, operand: Operand) -> u16 {
        0
    }

    fn adc(&mut self, operand: Operand) -> u16 {
        0
    }

    fn sub(&mut self, operand: Operand) -> u16 {
        0
    }

    fn sbc(&mut self, operand: Operand) -> u16 {
        0
    }

    fn and(&mut self, operand: Operand) -> u16 {
        0
    }

    fn xor(&mut self, operand: Operand) -> u16 {
        0
    }

    fn or(&mut self, operand: Operand) -> u16 {
        0
    }

    fn cp(&mut self, operand: Operand) -> u16 {
        0
    }

    fn inc(&mut self, operand: Operand) -> u16 {
        0
    }

    fn dec(&mut self, operand: Operand) -> u16 {
        0
    }

    fn rlca(&mut self) -> u16 {
        0
    }

    fn rrca(&mut self) -> u16 {
        0
    }

    fn rla(&mut self) -> u16 {
        0
    }

    fn rra(&mut self) -> u16 {
        0
    }

    fn daa(&mut self) -> u16 {
        0
    }

    fn cpl(&mut self) -> u16 {
        0
    }

    fn scf(&mut self) -> u16 {
        0
    }

    fn ccp(&mut self) -> u16 {
        0
    }

    fn rlc(&mut self, operand: Operand) -> u16 {
        0
    }

    fn rrc(&mut self, operand: Operand) -> u16 {
        0
    }

    fn rl(&mut self, operand: Operand) -> u16 {
        0
    }

    fn sla(&mut self, operand: Operand) -> u16 {
        0
    }

    fn sra(&mut self, operand: Operand) -> u16 {
        0
    }

    fn swap(&mut self, operand: Operand) -> u16 {
        0
    }

    fn srl(&mut self, operand: Operand) -> u16 {
        0
    }

    fn bit(&mut self, bit_index: BitIndex, operand: Operand) -> u16 {
        0
    }

    fn res(&mut self, bit_index: BitIndex, operand: Operand) -> u16 {
        0
    }

    fn set(&mut self, bit_index: BitIndex, operand: Operand) -> u16 {
        0
    }

    fn jump(&mut self, tgt: JumpTarget) -> u16 {
        0
    }

    fn jump_cond(&mut self, cond: Cond, tgt: JumpTarget) -> u16 {
        0
    }

    fn ret(&mut self) -> u16 {
        0
    }

    fn reti(&mut self) -> u16 {
        0
    }

    fn ret_cond(&mut self, cond: Cond) -> u16 {
        0
    }

    fn rst(&mut self, tgt: u8) -> u16 {
        0
    }

    fn pop(&mut self, reg: R16) -> u16 {
        0
    }

    fn push(&mut self, reg: R16) -> u16 {
        0
    }

    fn di(&mut self) -> u16 {
        0
    }

    fn ei(&mut self) -> u16 {
        0
    }
}
