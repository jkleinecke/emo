#![allow(non_upper_case_globals)]

use std::fmt;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Operation {
    ADC,AND,ASL,BCC,BCS,BEQ,BIT,BMI,BNE,
    BPL,BRK,BVC,BVS,CLC,CLD,CLI,CLV,CMP,
    CPX,CPY,DEC,DEX,DEY,EOR,INC,INX,INY,
    JMP,JSR,LDA,LDX,LDY,LSR,NOP,ORA,PHA,
    PHP,PLA,PLP,ROL,ROR,RTI,RTS,SBC,SEC,
    SED,SEI,STA,STX,STY,TAX,TAY,TSX,TXA,
    TXS,TYA,
    // "Extra" opcodes
    KIL,ISC,DCP,AXS,LAS,LAX,SHA,SAX,XAA,
    SHX,RRA,TAS,SHY,ARR,SRE,ALR,RLA,ANC,
    SLO,
}

impl fmt::Display for Operation
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "{:?}", self)
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AddressingMode {
    Immediate = 0,
    ZeroPage = 1,
    ZeroPageX = 2,
    ZeroPageY = 3,
    Absolute = 4,
    AbsoluteX = 5,
    AbsoluteY = 6,
    Indirect = 7,
    IndirectX = 8,
    IndirectY = 9,
    Relative = 10,
    Accumulator = 11,
    Implicit = 12,
}

pub const ADDR_OPSIZE_TABLE: [u8;13] =
    [
        2,      // immediate
        2,2,2,  // zero page
        3,3,3,  // absolute
        3,2,2,  // indirect
        2,      // relative
        1,1     // accumulator
    ];

use AddressingMode::*;
use Operation::*;

impl fmt::Display for AddressingMode
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result 
    {
        write!(f, "{}",
        match self {
            Absolute => "abs",
            Accumulator => "acc",
            Immediate => "imm",
            Implicit => "imp",
            IndirectX => "izx",
            IndirectY => "izy",
            ZeroPage => "zp",
            ZeroPageX => "zpx",
            ZeroPageY => "zpy",
            Relative => "rel",
            AbsoluteX => "abx",
            AbsoluteY => "aby",
            Indirect => "ind",
        })
    }
}

type CycleCount = u8;

//
pub const abs: AddressingMode = Absolute;
pub const acc: AddressingMode = Accumulator;
pub const imm: AddressingMode = Immediate;
pub const imp: AddressingMode = Implicit;
pub const izx: AddressingMode = IndirectX;
pub const izy: AddressingMode = IndirectY;
pub const zp: AddressingMode = ZeroPage;
pub const zpx: AddressingMode = ZeroPageX;
pub const zpy: AddressingMode = ZeroPageY;
pub const rel: AddressingMode = Relative;
pub const abx: AddressingMode = AbsoluteX;
pub const aby: AddressingMode = AbsoluteY;
pub const ind: AddressingMode = Indirect;

// Opcode table: http://www.oxyron.de/html/opcodes02.html
pub const OPCODE_TABLE: [(Operation, AddressingMode, CycleCount, CycleCount);256] =
    // TODO Audit each record to see that it was input correctly
    // (Operation, addressing mode, clock cycles, oops clock cycles if page boundary crossed)
    [//  x0            x1            x2            x3            x4            x5            x6            x7            x8            x9            xA            xB            xC            xD            xE            xF
/* 0x */(BRK,imp,7,0),(ORA,izx,6,0),(KIL,imp,0,0),(SLO,izx,8,0),(NOP,zp, 3,0),(ORA,zp, 3,0),(ASL,zp, 5,0),(SLO,zp, 5,0),(PHP,imp,3,0),(ORA,imm,2,0),(ASL,acc,2,0),(ANC,imm,2,0),(NOP,abs,4,0),(ORA,abs,4,0),(ASL,abs,6,0),(SLO,abs,6,0),
/* 1x */(BPL,rel,2,1),(ORA,izy,5,1),(KIL,imp,0,0),(SLO,izy,8,0),(NOP,zpx,4,0),(ORA,zpx,4,0),(ASL,zpx,6,0),(SLO,zpx,6,0),(CLC,imp,2,0),(ORA,aby,4,1),(NOP,imp,2,0),(SLO,aby,7,0),(NOP,abx,4,1),(ORA,abx,4,1),(ASL,abx,7,0),(SLO,abx,7,0),
/* 2x */(JSR,abs,6,0),(AND,izx,6,0),(KIL,imp,0,0),(RLA,izx,8,0),(BIT,zp, 3,0),(AND,zp, 3,0),(ROL,zp, 5,0),(RLA,zp, 5,0),(PLP,imp,4,0),(AND,imm,2,0),(ROL,acc,2,0),(ANC,imm,2,0),(BIT,abs,4,0),(AND,abs,4,0),(ROL,abs,6,0),(RLA,abs,6,0),
/* 3x */(BMI,rel,2,1),(AND,izy,5,1),(KIL,imp,0,0),(RLA,izy,8,0),(NOP,zpx,4,0),(AND,zpx,4,0),(ROL,zpx,6,0),(RLA,zpx,6,0),(SEC,imp,2,0),(AND,aby,4,1),(NOP,imp,2,0),(RLA,aby,7,0),(NOP,abx,4,1),(AND,abx,4,1),(ROL,abx,7,0),(RLA,abx,7,0),
/* 4x */(RTI,imp,6,0),(EOR,izx,6,0),(KIL,imp,0,0),(SRE,izx,8,0),(NOP,zp, 3,0),(EOR,zp, 3,0),(LSR,zp, 5,0),(SRE,zp, 5,0),(PHA,imp,3,0),(EOR,imm,2,0),(LSR,imp,2,0),(ALR,imm,2,0),(JMP,abs,3,0),(EOR,abs,4,0),(LSR,abs,6,0),(SRE,abs,6,0),
/* 5x */(BVC,rel,2,1),(EOR,izy,5,1),(KIL,imp,0,0),(SRE,izy,8,0),(NOP,zpx,4,0),(EOR,zpx,4,0),(LSR,zpx,6,0),(SRE,zpx,6,0),(CLI,imp,2,0),(EOR,aby,4,1),(NOP,imp,2,0),(SRE,aby,7,0),(NOP,abx,4,1),(EOR,abx,4,1),(LSR,abx,7,0),(SRE,abx,7,0),
/* 6x */(RTS,imp,6,0),(ADC,izx,6,0),(KIL,imp,0,0),(RRA,izx,8,0),(NOP,zp, 3,0),(ADC,zp, 3,0),(ROR,zp, 5,0),(RRA,zp, 5,0),(PLA,imp,4,0),(ADC,imm,2,0),(ROR,imp,2,0),(ARR,imm,2,0),(JMP,ind,5,0),(ADC,abs,4,0),(ROR,abs,6,0),(RRA,abs,6,0),
/* 7x */(BVS,rel,2,1),(ADC,izy,5,1),(KIL,imp,0,0),(RRA,izy,8,0),(NOP,zpx,4,0),(ADC,zpx,4,0),(ROR,zpx,6,0),(RRA,zpx,6,0),(SEI,imp,2,0),(ADC,aby,4,1),(NOP,imp,2,0),(RRA,aby,7,0),(NOP,abx,4,1),(ADC,abx,4,1),(ROR,abx,7,0),(RRA,abx,7,0),
/* 8x */(NOP,imm,2,0),(STA,izx,6,0),(NOP,imm,2,0),(SAX,izx,6,0),(STY,zp, 3,0),(STA,zp, 3,0),(STX,zp, 3,0),(SAX,zp, 3,0),(DEY,imp,2,0),(NOP,imm,2,0),(TXA,imp,2,0),(XAA,imm,2,1),(STY,abs,4,0),(STA,abs,4,0),(STX,abs,4,0),(SAX,abs,4,0),
/* 9x */(BCC,rel,2,1),(STA,izy,6,0),(KIL,imp,0,0),(SHA,izy,6,0),(STY,zpx,4,0),(STA,zpx,4,0),(STX,zpy,4,0),(SAX,zpy,4,0),(TYA,imp,2,0),(STA,aby,5,0),(TXS,imp,2,0),(TAS,aby,5,0),(SHY,abx,5,0),(STA,abx,5,0),(SHX,aby,5,0),(SHA,aby,5,0),
/* Ax */(LDY,imm,2,0),(LDA,izx,6,0),(LDX,imm,2,0),(LAX,izx,6,0),(LDY,zp, 3,0),(LDA,zp, 3,0),(LDX,zp, 3,0),(LAX,zp, 3,0),(TAY,imp,2,0),(LDA,imm,2,0),(TAX,imp,2,0),(LAX,imm,2,0),(LDY,abs,4,0),(LDA,abs,4,0),(LDX,abs,4,0),(LAX,abs,4,0),
/* Bx */(BCS,rel,2,1),(LDA,izy,5,1),(KIL,imp,0,0),(LAX,izy,5,1),(LDY,zpx,4,0),(LDA,zpx,4,0),(LDX,zpy,4,0),(LAX,zpy,4,0),(CLV,imp,2,0),(LDA,aby,4,1),(TSX,imp,2,0),(LAS,aby,4,1),(LDY,abx,4,1),(LDA,abx,4,1),(LDX,aby,4,1),(LAX,aby,4,1),
/* Cx */(CPY,imm,2,0),(CMP,izx,6,0),(NOP,imm,2,0),(DCP,izx,8,0),(CPY,zp, 3,0),(CMP,zp, 3,0),(DEC,zp, 5,0),(DCP,zp, 5,0),(INY,imp,2,0),(CMP,imm,2,0),(DEX,imp,2,0),(AXS,imm,2,0),(CPY,abs,4,0),(CMP,abs,4,0),(DEC,abs,6,0),(DCP,abs,6,0),
/* Dx */(BNE,rel,2,1),(CMP,izy,5,1),(KIL,imp,0,0),(DCP,izy,8,0),(NOP,zpx,4,0),(CMP,zpx,4,0),(DEC,zpx,6,0),(DCP,zpx,6,0),(CLD,imp,2,0),(CMP,aby,4,1),(NOP,imp,2,0),(DCP,aby,7,0),(NOP,abx,4,1),(CMP,abx,4,1),(DEC,abx,7,0),(DCP,abx,7,0),
/* Ex */(CPX,imm,2,0),(SBC,izx,6,0),(NOP,imm,2,0),(ISC,izx,8,0),(CPX,zp, 3,0),(SBC,zp, 3,0),(INC,zp, 5,0),(ISC,zp, 5,0),(INX,imp,2,0),(SBC,imm,2,0),(NOP,imp,2,0),(SBC,imm,2,0),(CPX,abs,4,0),(SBC,abs,4,0),(INC,abs,6,0),(ISC,abs,6,0),
/* Fx */(BEQ,rel,2,1),(SBC,izy,5,1),(KIL,imp,0,0),(ISC,izy,8,0),(NOP,zpx,4,0),(SBC,zpx,4,0),(INC,zpx,6,0),(ISC,zpx,6,0),(SED,imp,2,0),(SBC,aby,4,1),(NOP,imp,2,0),(ISC,aby,7,0),(NOP,abx,4,1),(SBC,abx,4,1),(INC,abx,7,0),(ISC,abx,7,0),
    ];

#[derive(Debug)]
pub struct DecodedOp
{
    pub ir: u8,

    pub opcode: Operation,
    pub addr_mode: AddressingMode,
    pub opsize: u8,
    
    pub cycles: u8,
    pub oops: bool,

}

impl DecodedOp
{
    pub fn from_ir(ir: u8) -> Self
    {
        let (operation, addr_mode, c1, c2) = OPCODE_TABLE[ir as usize];
        let opsize = match ir {
            0x00 => 2,  // break is size 2 despite the implied addressing mode
            _ => ADDR_OPSIZE_TABLE[addr_mode as usize]
        };

        DecodedOp {
            ir,  // read direct from memory, just in case we have the opsize incorrect
            opcode: operation,
            addr_mode,
            opsize,
            cycles: c1,
            oops: match c2 {
                0 => false,
                _ => true,
            }
        }
    }
}