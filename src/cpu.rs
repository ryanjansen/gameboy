use crate::instruction::*;
use crate::interrupts::{IME, Interrupt, Interrupts};
use crate::joypad::Joypad;
use crate::memory::Memory;
use crate::ppu::PPU;
use crate::register::{Flags, Registers};
use crate::timer::Timer;
use crate::utils::{reset_bit, set_bit};

pub struct CPU {
    registers: Registers,
    memory: Memory,
    pub ppu: PPU,
    timer: Timer,
    interrupts: Interrupts,
    pub joypad: Joypad,
    ime: IME,
    is_halted: bool,
    next_addr: u16,
    debugged: bool,
    watched_val: u8,
    watched_addr: u16,
}

impl CPU {
    // TODO: Add sound, mbc
    pub fn new(rom: Vec<u8>) -> CPU {
        CPU {
            registers: Registers::new(),
            memory: Memory::new(rom),
            ppu: PPU::new(),
            timer: Timer::new(),
            interrupts: Interrupts::new(),
            joypad: Joypad::new(),
            ime: IME::new(),
            is_halted: false,
            debugged: false,
            next_addr: 0x69,
            watched_val: 0x69,
            watched_addr: 0x69,
        }
    }

    fn debug_read(&self, address: u16) -> u8 {
        match address {
            0x0 => (self.timer.sysclock & 0xFF) as u8,
            0x8000..0x9FFF | 0xFE00..=0xFE9F | 0xFF40..=0xFF45 | 0xFF47..0xFF4B => {
                self.ppu.read_byte(address)
            }
            0xFF04..=0xFF07 => self.timer.read_byte(address),
            0xFF0F => self.interrupts.interrupt_flag,
            0xFFFF => self.interrupts.interrupt_enable,
            _ => self.memory.read_byte(address),
        }
    }

    pub fn debug(&mut self) {
        let val = self.debug_read(self.watched_addr);

        if !self.debugged || self.registers.pc == self.next_addr || val == self.watched_val {
            if self.registers.pc == self.next_addr {
                println!("At address {:04X}", self.next_addr);
            } else if val == self.watched_val {
                println!("Value at address {:04X} = {:02X}", self.watched_addr, val)
            }

            println!(
                "BC={:04X} DE={:04X} HL={:04X} AF={:04X} SP={:04X} PC={:04X}, PCMEM={:02X} LCDC={:02X}",
                self.registers.get_bc(),
                self.registers.get_de(),
                self.registers.get_hl(),
                self.registers.get_af(),
                self.registers.sp,
                self.registers.pc,
                self.memory.read_byte(self.registers.pc),
                self.ppu.read_byte(0xFF40),
            );

            self.debugged = true;
            println!("Enter next address to go to or inspect");

            loop {
                let mut input = String::new();
                std::io::stdin()
                    .read_line(&mut input)
                    .expect("Failed to read command");

                let (command, addr_str) = input.trim().split_at(1);

                if command == "g" {
                    let addr = u16::from_str_radix(addr_str, 16).unwrap();
                    self.next_addr = addr;
                    break;
                } else if command == "m" {
                    let address = u16::from_str_radix(addr_str, 16).unwrap();
                    let val = self.debug_read(address);
                    println!(
                        "Value at address {:04X} is {:08b} or {:02X}",
                        address, val, val
                    );
                } else if command == "w" {
                    let address = u16::from_str_radix(addr_str, 16).unwrap();
                    self.watched_addr = address;

                    println!("Enter the value to watch for:");

                    let mut input = String::new();

                    std::io::stdin()
                        .read_line(&mut input)
                        .expect("Failed to read command");

                    let val = u16::from_str_radix(input.trim(), 16).unwrap() as u8;

                    self.watched_val = val;
                    break;
                } else {
                    println!(
                        "Invalid command, use g to go to address and m to view value at address"
                    );
                }
            }
        }
    }

    fn get_instr(&mut self) -> Instruction {
        let mut instruction_byte = self.read_byte(self.registers.pc);
        let prefixed = instruction_byte == 0xCB;
        if prefixed {
            instruction_byte = self.read_byte(self.registers.pc + 1);
        }
        Instruction::decode(instruction_byte, prefixed)
    }

    pub fn step(&mut self) {
        if self.is_halted {
            self.handle_halted();
            return;
        }

        self.ime.set_if_scheduled();

        self.handle_interrupts();

        let instruction = self.get_instr();

        let InstrInfo {
            length: instr_length,
            cycles: _instr_cycles,
        } = self.execute(instruction);

        self.registers.pc += instr_length;
    }

    fn execute(&mut self, instruction: Instruction) -> InstrInfo {
        use Instruction::*;

        match instruction {
            NOP => InstrInfo {
                length: 1,
                cycles: 1,
            },
            STOP => InstrInfo {
                length: 2,
                cycles: 0,
            },
            HALT => self.halt(),
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
            CCF => self.ccf(),
            RLC(operand) => self.rlc(operand),
            RRC(operand) => self.rrc(operand),
            RL(operand) => self.rl(operand),
            RR(operand) => self.rr(operand),
            SLA(operand) => self.sla(operand),
            SRA(operand) => self.sra(operand),
            SWAP(operand) => self.swap(operand),
            SRL(operand) => self.srl(operand),
            BIT(bit_index, operand) => self.bit(bit_index, operand),
            RES(bit_index, operand) => self.res(bit_index, operand),
            SET(bit_index, operand) => self.set(bit_index, operand),
            JP(tgt) => self.jump(tgt),
            JPCOND(cond) => self.jump_cond(cond),
            JR => self.jump_relative(),
            JRCOND(cond) => self.jump_relative_cond(cond),
            CALL => self.call(),
            CALLCOND(cond) => self.call_cond(cond),
            RET => self.ret(),
            RETI => self.reti(),
            RETCOND(cond) => self.ret_cond(cond),
            RST(tgt) => self.rst(tgt),
            POP(reg) => self.pop(reg),
            PUSH(reg) => self.push(reg),
            DI => self.di(),
            EI => self.ei(),
            INVALID => panic!("Invalid instruction: {:?}", instruction),
        }
    }

    fn tick(&mut self, m_cycles: u16) {
        for _ in 0..m_cycles {
            self.timer.tick(&mut self.interrupts);
            self.ppu.tick(&mut self.interrupts);
        }
    }

    fn read_byte(&mut self, address: u16) -> u8 {
        self.tick(1);

        match address {
            0x8000..0x9FFF | 0xFE00..=0xFE9F | 0xFF40..=0xFF45 | 0xFF47..0xFF4B => {
                self.ppu.read_byte(address)
            }
            0xFF04..=0xFF07 => self.timer.read_byte(address),
            0xFF0F => self.interrupts.interrupt_flag,
            0xFFFF => self.interrupts.interrupt_enable,
            0xFF00 => self.joypad.read(),
            _ => self.memory.read_byte(address),
        }
    }

    fn write_byte(&mut self, address: u16, val: u8) {
        self.tick(1);

        match address {
            0x8000..0x9FFF | 0xFE00..=0xFE9F | 0xFF40..=0xFF45 | 0xFF47..0xFF4B => {
                self.ppu.write_byte(address, val)
            }
            0xFF46 => self.start_dma_transfer(val),
            0xFF04..=0xFF07 => self.timer.write_byte(address, val),
            0xFF0F => self.interrupts.interrupt_flag = val,
            0xFF00 => self.joypad.write(val),
            0xFFFF => self.interrupts.interrupt_enable = val,
            _ => self.memory.write_byte(address, val),
        }
    }

    fn start_dma_transfer(&mut self, val: u8) {
        let start_index = (val as u16) << 8;

        let oam = self.memory.get_dma_oam(start_index);
        self.ppu.dma(oam, &mut self.interrupts);
    }

    fn handle_halted(&mut self) {
        if self.interrupts.is_pending() {
            self.is_halted = false;
            self.handle_interrupts();
        } else {
            self.tick(1);
        }
    }

    fn execute_interrupt(&mut self, addr: u16) {
        self.tick(2);
        self.push_to_stack(self.registers.pc);
        self.registers.pc = addr;
        self.tick(1);
        self.ime.reset();
    }

    fn handle_interrupts(&mut self) {
        if !self.ime.is_set() {
            return;
        }

        let interrupt = self.interrupts.get_enabled_interrupt();

        match interrupt {
            Some(i) => match i {
                Interrupt::VBlank => {
                    self.execute_interrupt(0x40);
                }
                Interrupt::LCD => {
                    self.execute_interrupt(0x48);
                }
                Interrupt::Timer => {
                    self.execute_interrupt(0x50);
                }
                Interrupt::Serial => {
                    self.execute_interrupt(0x58);
                }
                Interrupt::Joypad => {
                    self.execute_interrupt(0x60);
                }
            },
            None => {}
        }
    }

    // Helper functions
    fn get_imm8(&mut self) -> u8 {
        self.read_byte(self.registers.pc + 1)
    }

    fn get_imm16(&mut self) -> u16 {
        let lower = self.read_byte(self.registers.pc + 1) as u16;
        let upper = self.read_byte(self.registers.pc + 2) as u16;
        upper << 8 | lower
    }

    fn get_register_val(&mut self, reg: R8) -> u8 {
        use R8::*;

        match reg {
            A => self.registers.a,
            B => self.registers.b,
            C => self.registers.c,
            D => self.registers.d,
            E => self.registers.e,
            H => self.registers.h,
            L => self.registers.l,
            IndirectHL => self.read_byte(self.registers.get_hl()),
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
            IndirectHL => self.write_byte(self.registers.get_hl(), val),
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
                self.registers.set_hl(val.wrapping_add(1));
                val
            }
            HLD => {
                let val = self.registers.get_hl();
                self.registers.set_hl(val.wrapping_sub(1));
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
            AF => self.registers.set_af(val),
            HLI => {
                self.registers.set_hl(val.wrapping_add(1));
            }
            HLD => {
                self.registers.set_hl(val.wrapping_sub(1));
            }
        }
    }

    fn get_addr_val(&mut self, addr: Addr) -> u16 {
        match addr {
            Addr::RegisterPair(reg) => self.get_register_pair_val(reg),
            Addr::Imm8WithIo => {
                let offset = self.get_imm8();
                0xFF00 | offset as u16
            }
            Addr::Imm16 => self.get_imm16(),
            Addr::CWithIo => 0xFF00 | self.get_register_val(R8::C) as u16,
        }
    }

    fn get_indirect_val(&mut self, addr: Addr) -> u8 {
        let addr_val = self.get_addr_val(addr);
        self.read_byte(addr_val)
    }

    fn set_indirect(&mut self, addr: Addr, val: u8) {
        let addr_val = self.get_addr_val(addr);
        self.write_byte(addr_val, val);
    }

    fn set_flags(&mut self, flags: Flags) {
        self.registers.set_flags(flags)
    }

    fn is_bit3_overflow(left: u8, right: u8) -> bool {
        (((left & 0xF) + (right & 0xF)) & 0x10) == 0x10
    }

    fn is_bit11_overflow(left: u16, right: u16) -> bool {
        (((left & 0x0FFF) + (right & 0x0FFF)) & 0x1000) == 0x1000
    }

    fn is_bit4_borrowed(left: u8, right: u8) -> bool {
        (left & 0x0F) < (right & 0x0F)
    }

    fn add_u8(left: u8, right: u8) -> (u8, Flags) {
        let (sum, is_overflow) = left.overflowing_add(right);
        let is_half_carry = CPU::is_bit3_overflow(left, right);
        (
            sum,
            Flags {
                zero: Some(sum == 0),
                subtract: Some(false),
                half_carry: Some(is_half_carry),
                carry: Some(is_overflow),
            },
        )
    }

    fn add_u16(left: u16, right: u16) -> (u16, Flags) {
        let (sum, is_overflow) = left.overflowing_add(right);
        let is_half_carry = CPU::is_bit11_overflow(left, right);
        (
            sum,
            Flags {
                zero: None,
                subtract: Some(false),
                half_carry: Some(is_half_carry),
                carry: Some(is_overflow),
            },
        )
    }

    fn add_u16_i8(left: u16, right: i8) -> (u16, Flags) {
        let (_, is_overflow) = (left as u8).overflowing_add(right as u8);
        let sum = left.wrapping_add(right as u16);
        let is_half_carry = CPU::is_bit3_overflow(left as u8, right as u8);
        (
            sum,
            Flags {
                zero: Some(false),
                subtract: Some(false),
                half_carry: Some(is_half_carry),
                carry: Some(is_overflow),
            },
        )
    }

    fn add_with_carry(left: u8, right: u8, is_carry: bool) -> (u8, Flags) {
        let (sum, is_overflow) = left.overflowing_add(right);
        let (final_sum, final_is_overflow) = sum.overflowing_add(is_carry as u8);
        let is_half_carry =
            CPU::is_bit3_overflow(left, right) || CPU::is_bit3_overflow(sum, is_carry as u8);
        (
            final_sum,
            Flags {
                zero: Some(final_sum == 0),
                subtract: Some(false),
                half_carry: Some(is_half_carry),
                carry: Some(final_is_overflow || is_overflow),
            },
        )
    }

    fn sub_u8(left: u8, right: u8) -> (u8, Flags) {
        let (diff, is_overflow) = left.overflowing_sub(right);
        let is_half_carry = CPU::is_bit4_borrowed(left, right);
        (
            diff,
            Flags {
                zero: Some(diff == 0),
                subtract: Some(true),
                half_carry: Some(is_half_carry),
                carry: Some(is_overflow),
            },
        )
    }

    fn sub_with_carry(left: u8, right: u8, is_carry: bool) -> (u8, Flags) {
        let carry = is_carry as u8;
        let (diff, is_overflow) = left.overflowing_sub(right);
        let (final_diff, final_is_overflow) = diff.overflowing_sub(carry);
        (
            final_diff,
            Flags {
                zero: Some(final_diff == 0),
                subtract: Some(true),
                half_carry: Some(
                    (left & 0x0F).wrapping_sub(right & 0x0F).wrapping_sub(carry) > 0x0F,
                ),
                carry: Some(final_is_overflow || is_overflow),
            },
        )
    }

    fn is_cond_true(&self, cond: Cond) -> bool {
        match cond {
            Cond::NZ => !self.registers.is_zero(),
            Cond::Z => self.registers.is_zero(),
            Cond::NC => !self.registers.is_carry(),
            Cond::C => self.registers.is_carry(),
        }
    }

    fn push_to_stack(&mut self, val: u16) {
        let lower = (val & 0x00FF) as u8;
        let upper = (val >> 8) as u8;
        self.registers.sp -= 1;
        self.write_byte(self.registers.sp, upper);
        self.registers.sp -= 1;
        self.write_byte(self.registers.sp, lower);
    }

    fn pop_from_stack(&mut self) -> u16 {
        let lower = self.read_byte(self.registers.sp) as u16;
        self.registers.sp += 1;
        let upper = self.read_byte(self.registers.sp) as u16;
        self.registers.sp += 1;
        (upper << 8) | lower
    }

    // Instruction Functions

    fn halt(&mut self) -> InstrInfo {
        self.is_halted = true;

        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn load(&mut self, dest: Dest, src: Source) -> InstrInfo {
        // dest: [imm16], r16, r8, [r16mem], A, HL, SP
        // src: imm8, imm16, r8, SP, SP + e8, A, [r16mem],

        let invalid_error_msg = format!("Invalid load instruction LD {:?} {:?}", &dest, &src);

        match (dest, src) {
            // LD r16, imm16
            (Dest::RegisterPair(reg_pair), Source::Imm16) => {
                let src_val = self.get_imm16();
                self.set_register_pair(reg_pair, src_val);
                InstrInfo {
                    length: 3,
                    cycles: 3,
                }
            }
            // LD [r16mem], A
            (Dest::Indirect(Addr::RegisterPair(r16mem)), Source::A) => {
                let src_val = self.get_register_val(R8::A);
                self.set_indirect(Addr::RegisterPair(r16mem), src_val);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            // LD A, [r16mem]
            (Dest::A, Source::Indirect(Addr::RegisterPair(r16mem))) => {
                let src_val = self.get_indirect_val(Addr::RegisterPair(r16mem));
                self.set_register(R8::A, src_val);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            // LD [imm16], SP
            (Dest::Indirect(Addr::Imm16), Source::RegisterPair(R16::SP)) => {
                let sp_val = self.get_register_pair_val(R16::SP);
                let lower = (sp_val & 0xFF) as u8;
                let upper = ((sp_val >> 8) & 0xFF) as u8;
                let addr_val = self.get_imm16();
                self.write_byte(addr_val, lower);
                self.write_byte(addr_val + 1, upper);
                InstrInfo {
                    length: 3,
                    cycles: 5,
                }
            }
            // LD r8, imm8
            (Dest::Register(reg), Source::Imm8) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 3 } else { 2 };
                let src_val = self.get_imm8();
                self.set_register(reg, src_val);
                InstrInfo { length: 2, cycles }
            }
            // LD r8, r8
            (Dest::Register(dest_reg), Source::Register(src_reg)) => {
                let src_val = self.get_register_val(src_reg);
                self.set_register(dest_reg, src_val);
                InstrInfo {
                    length: 1,
                    cycles: 1,
                }
            }
            // LD [imm16], A
            (Dest::Indirect(Addr::Imm16), Source::A) => {
                let src_val = self.get_register_val(R8::A);
                self.set_indirect(Addr::Imm16, src_val);
                InstrInfo {
                    length: 3,
                    cycles: 4,
                }
            }
            // LD A, [imm16]
            (Dest::A, Source::Indirect(Addr::Imm16)) => {
                let src_val = self.get_indirect_val(Addr::Imm16);
                self.set_register(R8::A, src_val);
                InstrInfo {
                    length: 3,
                    cycles: 4,
                }
            }
            // LD HL, SP + e8
            (Dest::RegisterPair(R16::HL), Source::SPWithImmSignedOffset) => {
                let sp_val = self.get_register_pair_val(R16::SP);
                let signed_offset = self.get_imm8() as i8;
                let (sum, flags) = CPU::add_u16_i8(sp_val, signed_offset);
                self.set_register_pair(R16::HL, sum);
                self.set_flags(flags);
                self.tick(1);
                InstrInfo {
                    length: 2,
                    cycles: 3,
                }
            }
            // LD SP, HL
            (Dest::RegisterPair(R16::SP), Source::RegisterPair(R16::HL)) => {
                let src_val = self.get_register_pair_val(R16::HL);
                self.set_register_pair(R16::SP, src_val);
                self.tick(1);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            _ => unreachable!("{}", invalid_error_msg),
        }
    }

    fn load_with_io(&mut self, dest: Dest, src: Source) -> InstrInfo {
        let invalid_error_msg = format!(
            "Invalid load with io instruction LDH {:?} {:?}",
            &dest, &src
        );

        match (dest, src) {
            (Dest::Indirect(Addr::CWithIo), Source::A) => {
                let src_val = self.get_register_val(R8::A);
                self.set_indirect(Addr::CWithIo, src_val);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            (Dest::A, Source::Indirect(Addr::CWithIo)) => {
                let src_val = self.get_indirect_val(Addr::CWithIo);
                self.set_register(R8::A, src_val);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            (Dest::Indirect(Addr::Imm8WithIo), Source::A) => {
                let src_val = self.get_register_val(R8::A);
                self.set_indirect(Addr::Imm8WithIo, src_val);
                InstrInfo {
                    length: 2,
                    cycles: 3,
                }
            }
            (Dest::A, Source::Indirect(Addr::Imm8WithIo)) => {
                let src_val = self.get_indirect_val(Addr::Imm8WithIo);
                self.set_register(R8::A, src_val);
                InstrInfo {
                    length: 2,
                    cycles: 3,
                }
            }
            _ => unreachable!("{}", invalid_error_msg),
        }
    }

    fn alu_add(&mut self, dest: Dest, operand: Operand) -> InstrInfo {
        let invalid_error_msg = format!("Invalid add instruction ADD {:?} {:?}", &dest, &operand);

        match (dest, operand) {
            (Dest::RegisterPair(R16::HL), Operand::RegisterPair(reg)) => {
                let reg_val = self.get_register_pair_val(reg);
                let hl_val = self.get_register_pair_val(R16::HL);
                let (sum, flags) = CPU::add_u16(hl_val, reg_val);
                self.set_flags(flags);
                self.set_register_pair(R16::HL, sum);
                self.tick(1);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            (Dest::A, Operand::Register(reg)) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let (sum, flags) = CPU::add_u8(a_val, reg_val);
                self.set_flags(flags);
                self.set_register(R8::A, sum);
                InstrInfo { length: 1, cycles }
            }
            (Dest::A, Operand::Imm8) => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let (sum, flags) = CPU::add_u8(a_val, imm_val);
                self.set_flags(flags);
                self.set_register(R8::A, sum);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            (Dest::RegisterPair(R16::SP), Operand::ImmSignedOffset) => {
                let signed_offset = self.get_imm8() as i8;
                let sp_val = self.get_register_pair_val(R16::SP);
                let (sum, flags) = CPU::add_u16_i8(sp_val, signed_offset);
                self.set_flags(flags);
                self.set_register_pair(R16::SP, sum);
                self.tick(2);
                InstrInfo {
                    length: 2,
                    cycles: 4,
                }
            }
            _ => unreachable!("{}", invalid_error_msg),
        }
    }

    fn adc(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let (sum, flags) = CPU::add_with_carry(imm_val, a_val, self.registers.is_carry());
                self.set_flags(flags);
                self.set_register(R8::A, sum);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let (sum, flags) = CPU::add_with_carry(reg_val, a_val, self.registers.is_carry());
                self.set_flags(flags);
                self.set_register(R8::A, sum);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for ADC"),
        }
    }

    fn sub(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let (diff, flags) = CPU::sub_u8(a_val, imm_val);
                self.set_flags(flags);
                self.set_register(R8::A, diff);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let (diff, flags) = CPU::sub_u8(a_val, reg_val);
                self.set_flags(flags);
                self.set_register(R8::A, diff);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for SUB"),
        }
    }

    fn sbc(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let (diff, flags) = CPU::sub_with_carry(a_val, imm_val, self.registers.is_carry());
                self.set_flags(flags);
                self.set_register(R8::A, diff);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let (diff, flags) = CPU::sub_with_carry(a_val, reg_val, self.registers.is_carry());
                self.set_flags(flags);
                self.set_register(R8::A, diff);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for SBC"),
        }
    }

    fn and(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let result = a_val & imm_val;
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(true),
                    carry: Some(false),
                });
                self.set_register(R8::A, result);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let result = a_val & reg_val;
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(true),
                    carry: Some(false),
                });
                self.set_register(R8::A, result);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for AND"),
        }
    }

    fn xor(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let result = a_val ^ imm_val;
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some(false),
                });
                self.set_register(R8::A, result);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let result = a_val ^ reg_val;
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some(false),
                });
                self.set_register(R8::A, result);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for xor"),
        }
    }

    fn or(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let result = a_val | imm_val;
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some(false),
                });
                self.set_register(R8::A, result);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let result = a_val | reg_val;
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some(false),
                });
                self.set_register(R8::A, result);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand {:?} for OR", operand),
        }
    }

    fn cp(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Imm8 => {
                let imm_val = self.get_imm8();
                let a_val = self.get_register_val(R8::A);
                let (_, flags) = CPU::sub_u8(a_val, imm_val);
                self.set_flags(flags);
                InstrInfo {
                    length: 2,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 2 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let a_val = self.get_register_val(R8::A);
                let (_, flags) = CPU::sub_u8(a_val, reg_val);
                self.set_flags(flags);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for CP"),
        }
    }

    fn inc(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::RegisterPair(reg) => {
                let reg_val = self.get_register_pair_val(reg);
                let result = reg_val.wrapping_add(1);
                self.set_register_pair(reg, result);

                self.tick(1);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 3 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let result = reg_val.wrapping_add(1);
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(false),
                    half_carry: Some(CPU::is_bit3_overflow(reg_val, 1)),
                    carry: None,
                });
                self.set_register(reg, result);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for INC"),
        }
    }

    fn dec(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::RegisterPair(reg) => {
                let reg_val = self.get_register_pair_val(reg);
                let result = reg_val.wrapping_sub(1);
                self.set_register_pair(reg, result);

                self.tick(1);
                InstrInfo {
                    length: 1,
                    cycles: 2,
                }
            }
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 3 } else { 1 };
                let reg_val = self.get_register_val(reg);
                let result = reg_val.wrapping_sub(1);
                self.set_flags(Flags {
                    zero: Some(result == 0),
                    subtract: Some(true),
                    half_carry: Some(CPU::is_bit4_borrowed(reg_val, 1)),
                    carry: None,
                });
                self.set_register(reg, result);
                InstrInfo { length: 1, cycles }
            }
            _ => unreachable!("Illegal operand for DEC"),
        }
    }

    fn rlca(&mut self) -> InstrInfo {
        let a_val = self.get_register_val(R8::A);
        let rotated_val = a_val.rotate_left(1);
        self.set_register(R8::A, rotated_val);
        self.set_flags(Flags {
            zero: Some(false),
            subtract: Some(false),
            half_carry: Some(false),
            carry: Some((a_val >> 7) == 1),
        });
        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn rrca(&mut self) -> InstrInfo {
        let a_val = self.get_register_val(R8::A);
        let rotated_val = a_val.rotate_right(1);
        self.set_register(R8::A, rotated_val);
        self.set_flags(Flags {
            zero: Some(false),
            subtract: Some(false),
            half_carry: Some(false),
            carry: Some((a_val & 1) == 1),
        });
        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn rla(&mut self) -> InstrInfo {
        let a_val = self.get_register_val(R8::A);
        let rotated_val = a_val << 1 | (self.registers.is_carry() as u8);
        self.set_register(R8::A, rotated_val);
        self.set_flags(Flags {
            zero: Some(false),
            subtract: Some(false),
            half_carry: Some(false),
            carry: Some((a_val >> 7) == 1),
        });
        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn rra(&mut self) -> InstrInfo {
        let a_val = self.get_register_val(R8::A);
        let rotated_val = a_val >> 1 | ((self.registers.is_carry() as u8) << 7);
        self.set_register(R8::A, rotated_val);
        self.set_flags(Flags {
            zero: Some(false),
            subtract: Some(false),
            half_carry: Some(false),
            carry: Some((a_val & 1) == 1),
        });
        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn daa(&mut self) -> InstrInfo {
        let a_val = self.get_register_val(R8::A);
        if self.registers.is_subtract() {
            let mut adj: u8 = 0;
            if self.registers.is_half_carry() {
                adj += 0x6;
            }
            if self.registers.is_carry() {
                adj += 0x60;
            }
            let (diff, flags) = CPU::sub_u8(a_val, adj);
            self.set_flags(Flags {
                zero: flags.zero,
                subtract: None,
                half_carry: Some(false),
                carry: None,
            });
            self.set_register(R8::A, diff);
        } else {
            let mut adj: u8 = 0;
            let mut is_carry = false;
            if self.registers.is_half_carry() || a_val & 0xF > 0x9 {
                adj += 0x6;
            }
            if self.registers.is_carry() || a_val > 0x99 {
                adj += 0x60;
                is_carry = true;
            }
            let (sum, flags) = CPU::add_u8(a_val, adj);
            self.set_flags(Flags {
                zero: flags.zero,
                subtract: None,
                half_carry: Some(false),
                carry: if is_carry { Some(true) } else { None },
            });
            self.set_register(R8::A, sum);
        }

        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn cpl(&mut self) -> InstrInfo {
        let reg_val = self.get_register_val(R8::A);
        self.set_register(R8::A, !reg_val);
        self.set_flags(Flags {
            zero: None,
            subtract: Some(true),
            half_carry: Some(true),
            carry: None,
        });

        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn scf(&mut self) -> InstrInfo {
        self.set_flags(Flags {
            zero: None,
            subtract: Some(false),
            half_carry: Some(false),
            carry: Some(true),
        });

        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn ccf(&mut self) -> InstrInfo {
        self.set_flags(Flags {
            zero: None,
            subtract: Some(false),
            half_carry: Some(false),
            carry: Some(!self.registers.is_carry()),
        });

        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn rlc(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let rotated_val = reg_val.rotate_left(1);
                self.set_register(reg, rotated_val);
                self.set_flags(Flags {
                    zero: Some(rotated_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val >> 7) == 1),
                });
                InstrInfo { length: 2, cycles }
            }
            _ => unreachable!("Illegal operand for RLC"),
        }
    }

    fn rrc(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let rotated_val = reg_val.rotate_right(1);
                self.set_register(reg, rotated_val);
                self.set_flags(Flags {
                    zero: Some(rotated_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val & 1) == 0x1),
                });
                InstrInfo { length: 2, cycles }
            }
            _ => unreachable!("Illegal operand for RRC"),
        }
    }

    fn rl(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let rotated_val = reg_val << 1 | (self.registers.is_carry() as u8);
                self.set_register(reg, rotated_val);
                self.set_flags(Flags {
                    zero: Some(rotated_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val >> 7) == 1),
                });
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for RL"),
        }
    }

    fn rr(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let rotated_val = reg_val >> 1 | ((self.registers.is_carry() as u8) << 7);
                self.set_register(reg, rotated_val);
                self.set_flags(Flags {
                    zero: Some(rotated_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val & 1) == 1),
                });
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for RR"),
        }
    }

    fn sla(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let shifted_val = reg_val << 1;
                self.set_register(reg, shifted_val);
                self.set_flags(Flags {
                    zero: Some(shifted_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val >> 7) == 1),
                });
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for SLA"),
        }
    }

    fn sra(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let shifted_val = ((reg_val as i8) >> 1) as u8;
                self.set_register(reg, shifted_val);
                self.set_flags(Flags {
                    zero: Some(shifted_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val & 1) == 1),
                });
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for SRA"),
        }
    }

    fn swap(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let lower = reg_val & 0x0F;
                let upper = reg_val >> 4;
                let swapped_val = (lower << 4) | upper;
                self.set_register(reg, swapped_val);
                self.set_flags(Flags {
                    zero: Some(swapped_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some(false),
                });
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for SWAP"),
        }
    }

    fn srl(&mut self, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                let shifted_val = reg_val >> 1;
                self.set_register(reg, shifted_val);
                self.set_flags(Flags {
                    zero: Some(shifted_val == 0),
                    subtract: Some(false),
                    half_carry: Some(false),
                    carry: Some((reg_val & 1) == 1),
                });

                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for SRL"),
        }
    }

    fn bit(&mut self, bit_index: BitIndex, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 3 } else { 2 };
                let reg_val = self.get_register_val(reg);
                self.set_flags(Flags {
                    zero: Some((reg_val >> bit_index & 1) == 0),
                    subtract: Some(false),
                    half_carry: Some(true),
                    carry: None,
                });

                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for BIT"),
        }
    }

    fn res(&mut self, bit_index: BitIndex, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                self.set_register(reg, reset_bit(reg_val, bit_index));
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for RES"),
        }
    }

    fn set(&mut self, bit_index: BitIndex, operand: Operand) -> InstrInfo {
        match operand {
            Operand::Register(reg) => {
                let cycles = if matches!(reg, R8::IndirectHL) { 4 } else { 2 };
                let reg_val = self.get_register_val(reg);
                self.set_register(reg, set_bit(reg_val, bit_index));
                InstrInfo { length: 2, cycles }
            }

            _ => unreachable!("Illegal operand for SET"),
        }
    }

    fn jump(&mut self, tgt: JumpTarget) -> InstrInfo {
        match tgt {
            JumpTarget::Imm16 => {
                let addr = self.get_imm16();
                self.registers.pc = addr;

                self.tick(1);
                InstrInfo {
                    length: 0, // To prevent moving PC after jump, actual is 3
                    cycles: 4,
                }
            }
            JumpTarget::HL => {
                let addr = self.get_register_pair_val(R16::HL);
                self.registers.pc = addr;
                InstrInfo {
                    length: 0,
                    cycles: 1,
                }
            }
        }
    }

    fn jump_cond(&mut self, cond: Cond) -> InstrInfo {
        if self.is_cond_true(cond) {
            let addr = self.get_imm16();
            self.registers.pc = addr;

            self.tick(1);
            InstrInfo {
                length: 0, // To prevent moving PC after jump, actual is 3
                cycles: 4,
            }
        } else {
            self.tick(2);

            InstrInfo {
                length: 3,
                cycles: 3,
            }
        }
    }

    fn jump_relative(&mut self) -> InstrInfo {
        let signed_offset = self.get_imm8() as i8;
        let (addr, _) = CPU::add_u16_i8(self.registers.pc, signed_offset);
        self.registers.pc = addr + 2; // Jump from PC addr after this instruction (2 bytes later)

        self.tick(1);
        InstrInfo {
            length: 0,
            cycles: 3,
        }
    }

    fn jump_relative_cond(&mut self, cond: Cond) -> InstrInfo {
        if self.is_cond_true(cond) {
            let signed_offset = self.get_imm8() as i8;
            let (addr, _) = CPU::add_u16_i8(self.registers.pc, signed_offset);
            self.registers.pc = addr + 2; // Jump from PC addr after this instruction (2 bytes later)

            self.tick(1);
            InstrInfo {
                length: 0,
                cycles: 3,
            }
        } else {
            self.tick(1);
            InstrInfo {
                length: 2,
                cycles: 2,
            }
        }
    }

    fn call(&mut self) -> InstrInfo {
        let new_addr = self.get_imm16();
        let curr_addr = self.registers.pc + 3;
        self.push_to_stack(curr_addr);
        self.registers.pc = new_addr;

        self.tick(1);

        InstrInfo {
            length: 0,
            cycles: 6,
        }
    }

    fn call_cond(&mut self, cond: Cond) -> InstrInfo {
        if self.is_cond_true(cond) {
            let new_addr = self.get_imm16();
            let curr_addr = self.registers.pc + 3;
            self.push_to_stack(curr_addr);
            self.registers.pc = new_addr;

            self.tick(1);

            InstrInfo {
                length: 0,
                cycles: 6,
            }
        } else {
            self.tick(2);

            InstrInfo {
                length: 3,
                cycles: 3,
            }
        }
    }

    fn ret(&mut self) -> InstrInfo {
        let addr = self.pop_from_stack();
        self.registers.pc = addr;

        self.tick(1);

        InstrInfo {
            length: 0,
            cycles: 4,
        }
    }

    fn reti(&mut self) -> InstrInfo {
        self.ime.schedule();
        self.ret()
    }

    fn ret_cond(&mut self, cond: Cond) -> InstrInfo {
        if self.is_cond_true(cond) {
            let addr = self.pop_from_stack();
            self.registers.pc = addr;

            self.tick(2);

            InstrInfo {
                length: 0,
                cycles: 5,
            }
        } else {
            self.tick(1);

            InstrInfo {
                length: 1,
                cycles: 2,
            }
        }
    }

    fn rst(&mut self, tgt: u8) -> InstrInfo {
        let curr_addr = self.registers.pc + 1;
        self.push_to_stack(curr_addr);
        self.registers.pc = tgt as u16;

        self.tick(1);

        InstrInfo {
            length: 0,
            cycles: 4,
        }
    }

    fn pop(&mut self, reg: R16) -> InstrInfo {
        let stack_val = self.pop_from_stack();
        self.set_register_pair(reg, stack_val);
        InstrInfo {
            length: 1,
            cycles: 3,
        }
    }

    fn push(&mut self, reg: R16) -> InstrInfo {
        let reg_val = self.get_register_pair_val(reg);
        self.push_to_stack(reg_val);

        self.tick(1);

        InstrInfo {
            length: 1,
            cycles: 4,
        }
    }

    fn di(&mut self) -> InstrInfo {
        self.ime.reset();
        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }

    fn ei(&mut self) -> InstrInfo {
        self.ime.schedule();
        InstrInfo {
            length: 1,
            cycles: 1,
        }
    }
}

type Cycles = u16;

struct InstrInfo {
    length: u16,
    cycles: Cycles,
}
