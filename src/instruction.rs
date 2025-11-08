#[derive(Debug)]
pub enum R8 {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
    IndirectHL,
}

#[derive(Debug)]
pub enum Addr {
    HL,
    RegisterPair(R16),
    Imm8,
    Imm16,
    C,
}

#[derive(Debug)]
pub enum R16 {
    BC,
    DE,
    HL,
    SP,
    AF,
    HLI,
    HLD,
}

#[derive(Debug)]
pub enum Cond {
    NZ,
    Z,
    NC,
    C,
}

pub type BitIndex = u8;
pub type Imm8 = u8;
pub type Imm16 = u16;
pub type ImmSignedOffset = i8;

#[derive(Debug)]
pub enum Dest {
    Register(R8),
    RegisterPair(R16),
    Indirect(Addr),
    A,
}

#[derive(Debug)]
pub enum Source {
    Register(R8),
    RegisterPair(R16),
    Imm8,
    Imm16,
    Indirect(Addr),
    A,
    SPWithImmSignedOffset,
}

#[derive(Debug)]
pub enum Operand {
    Register(R8),
    RegisterPair(R16),
    Imm8,
    ImmSignedOffset,
    A,
}

#[derive(Debug)]
pub enum JumpTarget {
    HL,
    Imm16,
    ImmSignedOffset,
}

pub type Vec = u8;

#[derive(Debug)]
pub enum Instruction {
    LD(Dest, Source),
    LDH(Dest, Source),
    ADC(Operand),
    ADD(Dest, Operand),
    CP(Operand),
    DEC(Operand),
    INC(Operand),
    SBC(Operand),
    SUB(Operand),
    AND(Operand),
    OR(Operand),
    XOR(Operand),
    BIT(BitIndex, Operand),
    RES(BitIndex, Operand),
    SET(BitIndex, Operand),
    RLCA,
    RRCA,
    RLA,
    RRA,
    CPL,
    SCF,
    CCF,
    RL(Operand),
    RLC(Operand),
    RR(Operand),
    RRC(Operand),
    SLA(Operand),
    SRA(Operand),
    SRL(Operand),
    SWAP(Operand),
    CALL(JumpTarget),
    CALLCOND(Cond, JumpTarget),
    JP(JumpTarget),
    JPCOND(Cond, JumpTarget),
    JR(JumpTarget),
    JRCOND(Cond, JumpTarget),
    RET,
    RETCOND(Cond),
    RETI,
    RST(Vec),
    POP(R16),
    PUSH(R16),
    DI,
    EI,
    HALT,
    DAA,
    NOP,
    STOP,
    INVALID,
}

impl Instruction {
    pub fn decode(byte: u8, prefixed: bool) -> Instruction {
        if prefixed {
            Self::prefixed(byte)
        } else {
            Self::unprefixed(byte)
        }
    }

    fn unprefixed(byte: u8) -> Instruction {
        use Instruction::*;
        match byte {
            0o000 => NOP,
            0o020 => STOP,
            0o166 => HALT,
            // LD [imm16], SP
            0o010 => LD(Dest::Indirect(Addr::Imm16), Source::RegisterPair(R16::SP)),
            // LD r16, imm16
            0o001 | 0o021 | 0o041 | 0o061 => LD(
                Dest::RegisterPair(get_register_pair(byte >> 4 & 0x03)),
                Source::Imm16,
            ),
            // LD [r16mem], A
            0o002 | 0o022 | 0o042 | 0o62 => LD(
                Dest::Indirect(Addr::RegisterPair(get_register_pair_mem(byte >> 4 & 0x03))),
                Source::A,
            ),
            // LD A, [r16mem]
            0o012 | 0o032 | 0o052 | 0o072 => LD(
                Dest::A,
                Source::Indirect(Addr::RegisterPair(get_register_pair_mem(byte >> 4 & 0x03))),
            ),
            // LD r8, imm8
            0o016 | 0o026 | 0o036 | 0o046 | 0o056 | 0o066 | 0o076 => {
                LD(Dest::Register(get_register(byte >> 3 & 0x07)), Source::Imm8)
            }
            // LD r8, r8
            0o100..=0o177 => LD(
                Dest::Register(get_register(byte >> 3 & 0x07)),
                Source::Register(get_register(byte & 0x07)),
            ),
            // LD [imm16], A
            0o352 => LD(Dest::Indirect(Addr::Imm16), Source::A),
            // LD A, [imm16]
            0o372 => LD(Dest::A, Source::Indirect(Addr::Imm16)),
            // LD HL, SP + e8
            0o370 => LD(Dest::RegisterPair(R16::HL), Source::SPWithImmSignedOffset),
            // LD SP, HL
            0o371 => LD(Dest::RegisterPair(R16::SP), Source::RegisterPair(R16::HL)),
            // LDH [imm8], A
            0o340 => LDH(Dest::Indirect(Addr::Imm8), Source::A),
            // LDH A, [imm8]
            0o360 => LDH(Dest::A, Source::Indirect(Addr::Imm8)),
            // LDH [C], A
            0o342 => LDH(Dest::Indirect(Addr::C), Source::A),
            // LDH A, [C]
            0o362 => LDH(Dest::A, Source::Indirect(Addr::C)),
            // ADD, ADC, SUB, SBC, AND, XOR, OR, CP  A, r8
            0o200..=0o277 => get_alu_reg_instr(byte >> 3 & 0x07, byte & 0x07),
            // ADD, ADC, SUB, SBC, AND, XOR, OR, CP  A, imm8
            0o306 | 0o316 | 0o326 | 0o336 | 0o346 | 0o356 | 0o366 | 0o376 => {
                get_alu_imm_instr(byte >> 3 & 0x07)
            }
            // ADD HL, r16
            0o011 | 0o031 | 0o051 | 0o071 => ADD(
                Dest::RegisterPair(R16::HL),
                Operand::RegisterPair(get_register_pair(byte & 0x07)),
            ),
            // ADD SP, e8
            0o350 => ADD(Dest::RegisterPair(R16::SP), Operand::ImmSignedOffset),
            // INC r8
            0o004 | 0o014 | 0o024 | 0o034 | 0o044 | 0o054 | 0o064 | 0o074 => {
                INC(Operand::Register(get_register(byte >> 3 & 0x07)))
            }
            // DEC r8
            0o005 | 0o015 | 0o025 | 0o035 | 0o045 | 0o055 | 0o065 | 0o075 => {
                DEC(Operand::Register(get_register(byte >> 3 & 0x07)))
            }
            // INC r16
            0o003 | 0o023 | 0o043 | 0o063 => {
                INC(Operand::RegisterPair(get_register_pair(byte >> 4 & 0x03)))
            }
            // DEC r16
            0o013 | 0o033 | 0o053 | 0o073 => {
                DEC(Operand::RegisterPair(get_register_pair(byte >> 4 & 0x03)))
            }
            // RLCA, RRCA, RLA, RRA, DAA, CPL, SCF, CCF
            0o007 | 0o017 | 0o027 | 0o037 | 0o047 | 0o057 | 0o067 | 0o077 => {
                get_flag_instr(byte >> 3 & 0x07)
            }
            // JP imm16
            0o303 => JP(JumpTarget::Imm16),
            // JP HL
            0o351 => JP(JumpTarget::HL),
            // JP cond imm16
            0o302 | 0o312 | 0o322 | 0o332 => JPCOND(get_cond(byte >> 3 & 0x03), JumpTarget::Imm16),
            // JR e8
            0o030 => JR(JumpTarget::ImmSignedOffset),
            // JR cond e8
            0o040 | 0o050 | 0o060 | 0o070 => {
                JRCOND(get_cond(byte >> 3 & 0x03), JumpTarget::ImmSignedOffset)
            }
            // CALL imm16
            0o315 => CALL(JumpTarget::Imm16),
            // CALL cond imm16
            0o304 | 0o314 | 0o324 | 0o334 => {
                CALLCOND(get_cond(byte >> 3 & 0x03), JumpTarget::Imm16)
            }
            // RET
            0o311 => RET,
            // RETI
            0o331 => RETI,
            // RET cond
            0o300 | 0o310 | 0o320 | 0o330 => RETCOND(get_cond(byte >> 3 & 0x03)),
            // RST
            0o307 | 0o317 | 0o327 | 0o337 | 0o347 | 0o357 | 0o367 | 0o377 => {
                RST((byte >> 3 & 0x07) * 8)
            }
            // POP r16stk
            0o301 | 0o321 | 0o341 | 0o361 => POP(get_register_pair_stk(byte >> 4 & 0x03)),
            // PUSH r16stk
            0o305 | 0o325 | 0o345 | 0o365 => PUSH(get_register_pair_stk(byte >> 4 & 0x03)),
            0o363 => DI,
            0o373 => EI,
            _ => INVALID,
        }
    }

    fn prefixed(byte: u8) -> Instruction {
        use Instruction::*;

        match byte {
            0o000..=0o007 => RLC(Operand::Register(get_register(byte & 0x07))),
            0o010..=0o017 => RRC(Operand::Register(get_register(byte & 0x07))),
            0o020..=0o027 => RL(Operand::Register(get_register(byte & 0x07))),
            0o030..=0o037 => RR(Operand::Register(get_register(byte & 0x07))),
            0o040..=0o047 => SLA(Operand::Register(get_register(byte & 0x07))),
            0o050..=0o057 => SRA(Operand::Register(get_register(byte & 0x07))),
            0o060..=0o067 => SWAP(Operand::Register(get_register(byte & 0x07))),
            0o070..=0o077 => SRL(Operand::Register(get_register(byte & 0x07))),
            0o100..=0o177 => BIT(
                byte >> 3 & 0x07,
                Operand::Register(get_register(byte & 0x07)),
            ),
            0o200..=0o277 => RES(
                byte >> 3 & 0x07,
                Operand::Register(get_register(byte & 0x07)),
            ),
            0o300..=0o377 => SET(
                byte >> 3 & 0x07,
                Operand::Register(get_register(byte & 0x07)),
            ),
        }
    }
}

fn get_register(index: u8) -> R8 {
    use R8::*;

    match index {
        0 => B,
        1 => C,
        2 => D,
        3 => E,
        4 => H,
        5 => L,
        6 => IndirectHL,
        7 => A,
        _ => unreachable!("Invalid index for register"),
    }
}

fn get_register_pair(index: u8) -> R16 {
    use R16::*;

    match index {
        0 => BC,
        1 => DE,
        2 => HL,
        3 => SP,
        _ => unreachable!("Invalid index for register pair"),
    }
}

fn get_register_pair_mem(index: u8) -> R16 {
    use R16::*;

    match index {
        0 => BC,
        1 => DE,
        2 => HLI,
        3 => HLD,
        _ => unreachable!("Invalid index for register pair mem"),
    }
}

fn get_register_pair_stk(index: u8) -> R16 {
    use R16::*;

    match index {
        0 => BC,
        1 => DE,
        2 => HL,
        3 => AF,
        _ => unreachable!("Invalid index for register pair stk"),
    }
}

fn get_cond(index: u8) -> Cond {
    use Cond::*;

    match index {
        0 => NZ,
        1 => Z,
        2 => NC,
        3 => C,
        _ => unreachable!("Invalid index for cond"),
    }
}

fn get_alu_reg_instr(op_index: u8, reg_index: u8) -> Instruction {
    use Instruction::{ADC, ADD, AND, CP, OR, SBC, SUB, XOR};

    match op_index {
        0b000 => ADD(Dest::A, Operand::Register(get_register(reg_index))),
        0b001 => ADC(Operand::Register(get_register(reg_index))),
        0b010 => SUB(Operand::Register(get_register(reg_index))),
        0b011 => SBC(Operand::Register(get_register(reg_index))),
        0b100 => AND(Operand::Register(get_register(reg_index))),
        0b101 => XOR(Operand::Register(get_register(reg_index))),
        0b110 => OR(Operand::Register(get_register(reg_index))),
        0b111 => CP(Operand::Register(get_register(reg_index))),
        _ => unreachable!("Invalid index for alu op"),
    }
}

fn get_alu_imm_instr(op_index: u8) -> Instruction {
    use Instruction::{ADC, ADD, AND, CP, OR, SBC, SUB, XOR};

    match op_index {
        0b000 => ADD(Dest::A, Operand::Imm8),
        0b001 => ADC(Operand::Imm8),
        0b010 => SUB(Operand::Imm8),
        0b011 => SBC(Operand::Imm8),
        0b100 => AND(Operand::Imm8),
        0b101 => XOR(Operand::Imm8),
        0b110 => OR(Operand::Imm8),
        0b111 => CP(Operand::Imm8),
        _ => unreachable!("Invalid index for alu op"),
    }
}

fn get_flag_instr(instr_index: u8) -> Instruction {
    use Instruction::{CCF, CPL, DAA, RLA, RLCA, RRA, RRCA, SCF};

    match instr_index {
        0b000 => RLCA,
        0b001 => RRCA,
        0b010 => RLA,
        0b011 => RRA,
        0b100 => DAA,
        0b101 => CPL,
        0b110 => SCF,
        0b111 => CCF,
        _ => unreachable!("Invalid index for flag instruction"),
    }
}
