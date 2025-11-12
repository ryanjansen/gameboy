use crate::instruction::*;
use crate::register::Registers;
use std::fmt;

type Cycles = u16;

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

    fn write_byte(&mut self, address: u16, val: u8) {
        self.memory[address as usize] = val
    }
}

pub struct Flags {
    pub zero: bool,
    pub subtract: bool,
    pub half_carry: bool,
    pub carry: bool,
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

    fn execute(&mut self, instruction: Instruction) -> Cycles {
        use Instruction::*;

        match instruction {
            NOP => 1,
            STOP => 2,
            HALT => todo!("Halt"),
            LD(dest, src) => self.load(dest, src),
            LDH(dest, src) => self.load_with_io(dest, src),
            ADD(dest, operand) => self.alu_add(dest, operand),
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

    // Helper functions

    fn get_imm8(&self) -> u8 {
        self.memory.read_byte(self.registers.pc + 1)
    }

    fn get_imm16(&self) -> u16 {
        let lower = self.memory.read_byte(self.registers.pc + 1);
        let upper = self.memory.read_byte(self.registers.pc + 2);
        (upper << 8 | lower) as u16
    }

    fn get_register_val(&self, reg: R8) -> u8 {
        use R8::*;

        match reg {
            A => self.registers.a,
            B => self.registers.b,
            C => self.registers.c,
            D => self.registers.d,
            E => self.registers.e,
            H => self.registers.h,
            L => self.registers.l,
            IndirectHL => self.memory.read_byte(self.registers.get_hl()),
        }
    }

    fn set_register(&mut self, reg: R8, val: u8) {
        use R8::*;

        match reg {
            A => self.registers.a = val,
            B => self.registers.b = val,
            C => self.registers.c = val,
            D => self.registers.d = val,
            E => self.registers.e = val,
            H => self.registers.h = val,
            L => self.registers.l = val,
            IndirectHL => self.memory.write_byte(self.registers.get_hl(), val),
        }
    }

    fn get_register_pair_val(&mut self, reg: R16) -> u16 {
        use R16::*;

        match reg {
            BC => self.registers.get_bc(),
            DE => self.registers.get_de(),
            HL => self.registers.get_hl(),
            SP => self.registers.sp,
            AF => self.registers.get_af(),
            HLI => {
                let val = self.registers.get_hl();
                self.registers.set_hl(val + 1);
                val
            }
            HLD => {
                let val = self.registers.get_hl();
                self.registers.set_hl(val - 1);
                val
            }
        }
    }

    fn set_register_pair(&mut self, reg: R16, val: u16) {
        use R16::*;

        match reg {
            BC => self.registers.set_bc(val),
            DE => self.registers.set_de(val),
            HL => self.registers.set_hl(val),
            SP => self.registers.sp = val,
            AF => panic!("Not allowed to set register AF directly"),
            HLI => {
                self.registers.set_hl(val + 1);
            }
            HLD => {
                self.registers.set_hl(val - 1);
            }
        }
    }

    fn get_addr_val(&mut self, addr: Addr) -> u16 {
        match addr {
            Addr::RegisterPair(reg) => self.get_register_pair_val(reg),
            Addr::Imm8WithIo => self.get_imm8() as u16 + 0xFF00,
            Addr::Imm16 => self.get_imm16(),
            Addr::CWithIo => self.get_register_val(R8::C) as u16 + 0xFF00,
        }
    }

    fn get_indirect_val(&mut self, addr: Addr) -> u8 {
        let addr_val = self.get_addr_val(addr);
        self.memory.read_byte(addr_val)
    }

    fn set_indirect(&mut self, addr: Addr, val: u8) {
        let addr_val = self.get_addr_val(addr);
        self.memory.write_byte(addr_val, val);
    }

    fn set_flags(&mut self, flags: Flags) {
        self.registers.set_flags(flags)
    }

    fn is_bit3_overflow(left: u8, right: u8) -> bool {
        (left & 0b00001100) & (right & 0b00001100) == 0b00001000
    }

    fn add_u16_i8(left: u16, right: i8) -> (u16, Flags) {
        let (_, is_overflow) = (left as u8).overflowing_add(right as u8);
        let sum = left.wrapping_add(right as u16);
        let is_half_carry = CPU::is_bit3_overflow(left as u8, right as u8);
        (
            sum,
            Flags {
                zero: false,
                subtract: false,
                half_carry: is_half_carry,
                carry: is_overflow,
            },
        )
    }

    // Instruction Functions

    fn load(&mut self, dest: Dest, src: Source) -> Cycles {
        // dest: [imm16], r16, r8, [r16mem], A, HL, SP
        // src: imm8, imm16, r8, SP, SP + e8, A, [r16mem],
        //
        let invalid_error_msg = format!("Invalid load instruction LD {:?} {:?}", &dest, &src);

        match (dest, src) {
            // LD r16, imm16
            (Dest::RegisterPair(reg_pair), Source::Imm16) => {
                let src_val = self.get_imm16();
                self.set_register_pair(reg_pair, src_val);
                3
            }
            // LD [r16mem], A
            (Dest::Indirect(Addr::RegisterPair(r16mem)), Source::A) => {
                let src_val = self.get_register_val(R8::A);
                self.set_indirect(Addr::RegisterPair(r16mem), src_val);
                2
            }
            // LD A, [r16mem]
            (Dest::A, Source::Indirect(Addr::RegisterPair(r16mem))) => {
                let src_val = self.get_indirect_val(Addr::RegisterPair(r16mem));
                self.set_register(R8::A, src_val);
                2
            }
            // LD [imm16], SP
            (Dest::Indirect(Addr::Imm16), Source::RegisterPair(R16::SP)) => {
                let sp_val = self.get_register_pair_val(R16::SP);
                let lower = (sp_val & 0xFF) as u8;
                let upper = ((sp_val >> 8) & 0xFF) as u8;
                let addr_val = self.get_imm16();
                self.memory.write_byte(addr_val, lower);
                self.memory.write_byte(addr_val + 1, upper);
                5
            }
            // LD r8, imm8
            (Dest::Register(reg), Source::Imm8) => {
                let src_val = self.get_imm8();
                self.set_register(reg, src_val);
                2
            }
            // LD r8, r8
            (Dest::Register(dest_reg), Source::Register(src_reg)) => {
                let src_val = self.get_register_val(src_reg);
                self.set_register(dest_reg, src_val);
                1
            }
            // LD [imm16], A
            (Dest::Indirect(Addr::Imm16), Source::A) => {
                let src_val = self.get_register_val(R8::A);
                self.set_indirect(Addr::Imm16, src_val);
                4
            }
            // LD A, [imm16]
            (Dest::A, Source::Indirect(Addr::Imm16)) => {
                let src_val = self.get_indirect_val(Addr::Imm16);
                self.set_register(R8::A, src_val);
                4
            }
            // LD HL, SP + e8
            (Dest::RegisterPair(R16::HL), Source::SPWithImmSignedOffset) => {
                let sp_val = self.get_register_pair_val(R16::SP);
                let signed_offset = self.get_imm8() as i8;
                let (sum, flags) = CPU::add_u16_i8(sp_val, signed_offset);
                self.set_register_pair(R16::HL, sum);
                self.set_flags(flags);
                3
            }
            // LD SP, HL
            (Dest::RegisterPair(R16::SP), Source::RegisterPair(R16::HL)) => {
                let src_val = self.get_register_pair_val(R16::HL);
                self.set_register_pair(R16::SP, src_val);
                2
            }
            _ => unreachable!("{}", invalid_error_msg),
        }
    }

    fn load_with_io(&mut self, dest: Dest, src: Source) -> u16 {
        0
    }

    fn alu_add(&mut self, dest: Dest, operand: Operand) -> u16 {
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
