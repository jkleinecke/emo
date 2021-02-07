#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::bitflags::*;
use crate::common::{test_bit, Clocked};
use crate::bus::{BusInterface, BusControlStatus};

pub const PC_START: u16 = 0xFFFC;
const SP_START: u16 = 0xFD;

#[derive(Copy, Clone, Debug)]
enum Operation {
    ADC,AND,ASL,BCC,BCS,BEQ,BIT,BMI,BNE,
    BPL,BRK,BVC,BVS,CLC,CLD,CLI,CLV,CMP,
    CPX,CPY,DEC,DEX,DEY,EOR,INC,INX,INY,
    JMP,JSR,LDA,LDX,LDY,LSR,NOP,ORA,PHA,
    PHP,PLA,PLP,ROL,ROR,RTI,RTS,SBC,SEC,
    SED,SEI,STA,STX,STY,TAX,TAY,TSX,TXA,
    TXS,TYA,
    // "Extra" opcodes
    KIL,ISC,DCP,AXS,LAS,LAX,AHX,SAX,XAA,
    SHX,RRA,TAS,SHY,ARR,SRE,ALR,RLA,ANC,
    SLO,
}

#[derive(Copy, Clone, Debug)]
enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndirectX,
    IndirectY,
    Relative,
    Accumulator,
    Implicit,
}

bitflags! {
    #[derive(Default)]
    pub struct ProcessorStatus: u8 {
        const Carry       = 0b00000001;
        const Zero        = 0b00000010;
        const Interrupt   = 0b00000100;
        const Decimal     = 0b00001000;

        const Break       = 0b00010000;
        const Unused      = 0b00100000;
        const Overflow    = 0b01000000;
        const Negative    = 0b10000000;
    }
}

impl ProcessorStatus {
    pub fn get_push_value(&self, from_irq:bool) -> u8 {
        let mut pv = self.bits | ProcessorStatus::Break.bits ;        
        
        if from_irq == false {
            pv |= ProcessorStatus::Decimal.bits;
        }

        return pv;
    }

    pub fn reset() -> ProcessorStatus {
        ProcessorStatus::Break | ProcessorStatus::Unused | ProcessorStatus::Interrupt   // 0x34
    }
}

impl BusInterface for EmoC6502 {
    fn get_control_status(&self) -> BusControlStatus {
        self.bus_ctrl
    }
    fn get_address(&self) -> u16 {
        self.address
    }
    fn get_data(&self) -> u8 {
        self.data
    }
    fn set_data(&mut self, v:u8) {
        self.data = v;
    }
}

pub struct EmoC6502 {
    pub a: u8,                      // Accumulator Register
    pub x: u8,                      // X Register
    pub y: u8,                      // Y Register
    pub sp: u8,                     // Stack Pointer
    pub pc: u16,                    // Program Counter
    
    pub status: ProcessorStatus,    // Status Register

    iclocks:u8,                     // total clocks for the instruction
    instr: u8,                      // active instruction register
    
    // bus interface
    bus_ctrl: BusControlStatus,     // The R/W control flag
    address: u16,                   // The address pinout...
    data: u8,                       // The data pinout

    // incoming signals
    pub reset:bool,                 // <- reset interrupt
    pub nmi:bool,                   // <- nmi interrupt
    pub irq:bool,                   // <- irq interrupt

    // outgoing signals
    pub oe1:bool,                   // -> Controller 1 dump
    pub oe2:bool,                   // -> Controller 2 dump
}

impl Clocked for EmoC6502
{
    fn clock(&mut self) 
    {
        // First handle interrupts
        if self.status.contains(ProcessorStatus::Interrupt) // just assuming this means we need to handle the interrupts
        {
            // and assuming we're only handling reset for now..
            self.instr = BRK;
            self.iclocks = 2;
        }

        // Actual Clock Cycle
        match self.iclocks {

            1 => // store instruction + fetch address mode
            {
                self.instr = self.data;     // store data bus value as the opcode 
                self.fetch(self.pc) ;           // get next value at program counter
                self.pc += 1 ;
            }
            2 ..= 7 => // store address mode + handle instruction
            {
                // just for LDA
                assert!(self.instr == 0x00);
                let operation = OPCODE_TABLE[self.instr as usize];   // decode the instruction
                //assert!(operation.Operation == LDA);

                match self.iclocks {
                    2 => { /* push pc hi on stack */ }
                    3 => { /* push pc lo on stack */ }
                    4 => { /* push status on stack, reset, fetch pc lo*/ }
                    5 => { /* save pc lo and fetch pc hi */ }
                    6 => { /* save pc hi and fetch next instr */ }
                }

                // translate addressing mode

                // finally perform the operation
                self.a = self.data;

                // and fetch the next instruction
                self.fetch_next_instr();
            }
            0 => // fetch next instruction, only occurs on first CPU cycle
            {
                self.fetch_next_instr() ;           // get next value at program counter
            }
            _ => // Houston... we have a problem.  panic
            {
                panic!("Incorrect instruction opcode handling, did not reset cycle counter properly");
            } 
        }

        self.iclocks += 1;  // always increment the iclocks count

        // Check for interrupt
    }
}

impl EmoC6502 {
    pub fn new() -> Self {
        EmoC6502 {
            a: 0,
            x: 0,
            y: 0,
            sp: SP_START,
            pc: PC_START,

            status: ProcessorStatus::reset(), 

            iclocks:0,
            instr:0,

            bus_ctrl:BusControlStatus::Read,
            address:0,
            data:0,

            reset:false,
            nmi:false,
            irq:false,

            oe1:false,
            oe2:false,
        }
    }

    fn fetch_next_instr(&mut self) {
        self.fetch(self.pc);
        self.pc += 1;
        self.iclocks = 0;
    }

    fn store(&mut self, addr: u16, v: u8) {
        self.bus_ctrl = BusControlStatus::Write;
        self.address = addr;
        self.data = v;
    }

    fn fetch(&mut self, addr: u16) {
        self.bus_ctrl = BusControlStatus::Read;     // redundant, read should still be set
        self.address = addr;
    }

    fn update_status(&mut self, result:u8)
    {
        self.status.set(ProcessorStatus::Zero, result == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(result,7));
    }

    // interrupts
    fn reset(&mut self)
    {
        self.pc = PC_START;
        self.status = ProcessorStatus::reset();
        self.iclocks = 0;
        self.reset = false;
    }
}


use AddressingMode::*;
use Operation::*;


type CycleCount = u8;

//
const abs: AddressingMode = Absolute;
const acc: AddressingMode = Accumulator;
const imm: AddressingMode = Immediate;
const imp: AddressingMode = Implicit;
const izx: AddressingMode = IndirectX;
const izy: AddressingMode = IndirectY;
const zp: AddressingMode = ZeroPage;
const zpx: AddressingMode = ZeroPageX;
const zpy: AddressingMode = ZeroPageY;
const rel: AddressingMode = Relative;
const abx: AddressingMode = AbsoluteX;
const aby: AddressingMode = AbsoluteY;
const ind: AddressingMode = Indirect;

// Opcode table: http://www.oxyron.de/html/opcodes02.html
const OPCODE_TABLE: [(Operation, AddressingMode, CycleCount, CycleCount);256] =
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
/* 9x */(BCC,rel,2,1),(STA,izy,6,0),(KIL,imp,0,0),(AHX,izy,6,0),(STY,zpx,4,0),(STA,zpx,4,0),(STX,zpy,4,0),(SAX,zpy,4,0),(TYA,imp,2,0),(STA,aby,5,0),(TXS,imp,2,0),(TAS,aby,5,0),(SHY,abx,5,0),(STA,abx,5,0),(SHX,aby,5,0),(AHX,aby,5,0),
/* Ax */(LDY,imm,2,0),(LDA,izx,6,0),(LDX,imm,2,0),(LAX,izx,6,0),(LDY,zp, 3,0),(LDA,zp, 3,0),(LDX,zp, 3,0),(LAX,zp, 3,0),(TAY,imp,2,0),(LDA,imm,2,0),(TAX,imp,2,0),(LAX,imm,2,0),(LDY,abs,4,0),(LDA,abs,4,0),(LDX,abs,4,0),(LAX,abs,4,0),
/* Bx */(BCS,rel,2,1),(LDA,izy,5,1),(KIL,imp,0,0),(LAX,izy,5,1),(LDY,zpx,4,0),(LDA,zpx,4,0),(LDX,zpy,4,0),(LAX,zpy,4,0),(CLV,imp,2,0),(LDA,aby,4,1),(TSX,imp,2,0),(LAS,aby,4,1),(LDY,abx,4,1),(LDA,abx,4,1),(LDX,aby,4,1),(LAX,aby,4,1),
/* Cx */(CPY,imm,2,0),(CMP,izx,6,0),(NOP,imm,2,0),(DCP,izx,8,0),(CPY,zp, 3,0),(CMP,zp, 3,0),(DEC,zp, 5,0),(DCP,zp, 5,0),(INY,imp,2,0),(CMP,imm,2,0),(DEX,imp,2,0),(AXS,imm,2,0),(CPY,abs,4,0),(CMP,abs,4,0),(DEC,abs,6,0),(DCP,abs,6,0),
/* Dx */(BNE,rel,2,1),(CMP,izy,5,1),(KIL,imp,0,0),(DCP,izy,8,0),(NOP,zpx,4,0),(CMP,zpx,4,0),(DEC,zpx,6,0),(DCP,zpx,6,0),(CLD,imp,2,0),(CMP,aby,4,1),(NOP,imp,2,0),(DCP,aby,7,0),(NOP,abx,4,1),(CMP,abx,4,1),(DEC,abx,7,0),(DCP,abx,7,0),
/* Ex */(CPX,imm,2,0),(SBC,izx,6,0),(NOP,imm,2,0),(ISC,izx,8,0),(CPX,zp, 3,0),(SBC,zp, 3,0),(INC,zp, 5,0),(ISC,zp, 5,0),(INX,imp,2,0),(SBC,imm,2,0),(NOP,imp,2,0),(SBC,imm,2,0),(CPX,abs,4,0),(SBC,abs,4,0),(INC,abs,6,0),(ISC,abs,6,0),
/* Fx */(BEQ,rel,2,1),(SBC,izy,5,1),(KIL,imp,0,0),(ISC,izy,8,0),(NOP,zpx,4,0),(SBC,zpx,4,0),(INC,zpx,6,0),(ISC,zpx,6,0),(SED,imp,2,0),(SBC,aby,4,1),(NOP,imp,2,0),(ISC,aby,7,0),(NOP,abx,4,1),(SBC,abx,4,1),(INC,abx,7,0),(ISC,abx,7,0),
    ];

    //0 { "BRK", &a::BRK, &a::IMM, 7 },{ "ORA", &a::ORA, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::ZP0, 3 },{ "ASL", &a::ASL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHP", &a::PHP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::IMM, 2 },{ "ASL", &a::ASL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABS, 4 },{ "ASL", &a::ASL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
    //1 { "BPL", &a::BPL, &a::REL, 2 },{ "ORA", &a::ORA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ZPX, 4 },{ "ASL", &a::ASL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLC", &a::CLC, &a::IMP, 2 },{ "ORA", &a::ORA, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABX, 4 },{ "ASL", &a::ASL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
    //2 { "JSR", &a::JSR, &a::ABS, 6 },{ "AND", &a::AND, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "BIT", &a::BIT, &a::ZP0, 3 },{ "AND", &a::AND, &a::ZP0, 3 },{ "ROL", &a::ROL, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLP", &a::PLP, &a::IMP, 4 },{ "AND", &a::AND, &a::IMM, 2 },{ "ROL", &a::ROL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "BIT", &a::BIT, &a::ABS, 4 },{ "AND", &a::AND, &a::ABS, 4 },{ "ROL", &a::ROL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
    //3 { "BMI", &a::BMI, &a::REL, 2 },{ "AND", &a::AND, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ZPX, 4 },{ "ROL", &a::ROL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEC", &a::SEC, &a::IMP, 2 },{ "AND", &a::AND, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ABX, 4 },{ "ROL", &a::ROL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
    //4 { "RTI", &a::RTI, &a::IMP, 6 },{ "EOR", &a::EOR, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "EOR", &a::EOR, &a::ZP0, 3 },{ "LSR", &a::LSR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHA", &a::PHA, &a::IMP, 3 },{ "EOR", &a::EOR, &a::IMM, 2 },{ "LSR", &a::LSR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::ABS, 3 },{ "EOR", &a::EOR, &a::ABS, 4 },{ "LSR", &a::LSR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
    //5 { "BVC", &a::BVC, &a::REL, 2 },{ "EOR", &a::EOR, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ZPX, 4 },{ "LSR", &a::LSR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLI", &a::CLI, &a::IMP, 2 },{ "EOR", &a::EOR, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ABX, 4 },{ "LSR", &a::LSR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
    //6 { "RTS", &a::RTS, &a::IMP, 6 },{ "ADC", &a::ADC, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ADC", &a::ADC, &a::ZP0, 3 },{ "ROR", &a::ROR, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLA", &a::PLA, &a::IMP, 4 },{ "ADC", &a::ADC, &a::IMM, 2 },{ "ROR", &a::ROR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::IND, 5 },{ "ADC", &a::ADC, &a::ABS, 4 },{ "ROR", &a::ROR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
    //7 { "BVS", &a::BVS, &a::REL, 2 },{ "ADC", &a::ADC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ZPX, 4 },{ "ROR", &a::ROR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEI", &a::SEI, &a::IMP, 2 },{ "ADC", &a::ADC, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ABX, 4 },{ "ROR", &a::ROR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
    //8 { "???", &a::NOP, &a::IMP, 2 },{ "STA", &a::STA, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZP0, 3 },{ "STA", &a::STA, &a::ZP0, 3 },{ "STX", &a::STX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "DEY", &a::DEY, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 2 },{ "TXA", &a::TXA, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "STY", &a::STY, &a::ABS, 4 },{ "STA", &a::STA, &a::ABS, 4 },{ "STX", &a::STX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
    //9 { "BCC", &a::BCC, &a::REL, 2 },{ "STA", &a::STA, &a::IZY, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZPX, 4 },{ "STA", &a::STA, &a::ZPX, 4 },{ "STX", &a::STX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "TYA", &a::TYA, &a::IMP, 2 },{ "STA", &a::STA, &a::ABY, 5 },{ "TXS", &a::TXS, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::NOP, &a::IMP, 5 },{ "STA", &a::STA, &a::ABX, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::XXX, &a::IMP, 5 },
    //A { "LDY", &a::LDY, &a::IMM, 2 },{ "LDA", &a::LDA, &a::IZX, 6 },{ "LDX", &a::LDX, &a::IMM, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "LDY", &a::LDY, &a::ZP0, 3 },{ "LDA", &a::LDA, &a::ZP0, 3 },{ "LDX", &a::LDX, &a::ZP0, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "TAY", &a::TAY, &a::IMP, 2 },{ "LDA", &a::LDA, &a::IMM, 2 },{ "TAX", &a::TAX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "LDY", &a::LDY, &a::ABS, 4 },{ "LDA", &a::LDA, &a::ABS, 4 },{ "LDX", &a::LDX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
    //B { "BCS", &a::BCS, &a::REL, 2 },{ "LDA", &a::LDA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "LDY", &a::LDY, &a::ZPX, 4 },{ "LDA", &a::LDA, &a::ZPX, 4 },{ "LDX", &a::LDX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "CLV", &a::CLV, &a::IMP, 2 },{ "LDA", &a::LDA, &a::ABY, 4 },{ "TSX", &a::TSX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 4 },{ "LDY", &a::LDY, &a::ABX, 4 },{ "LDA", &a::LDA, &a::ABX, 4 },{ "LDX", &a::LDX, &a::ABY, 4 },{ "???", &a::XXX, &a::IMP, 4 },
    //C { "CPY", &a::CPY, &a::IMM, 2 },{ "CMP", &a::CMP, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPY", &a::CPY, &a::ZP0, 3 },{ "CMP", &a::CMP, &a::ZP0, 3 },{ "DEC", &a::DEC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INY", &a::INY, &a::IMP, 2 },{ "CMP", &a::CMP, &a::IMM, 2 },{ "DEX", &a::DEX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "CPY", &a::CPY, &a::ABS, 4 },{ "CMP", &a::CMP, &a::ABS, 4 },{ "DEC", &a::DEC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
    //D { "BNE", &a::BNE, &a::REL, 2 },{ "CMP", &a::CMP, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ZPX, 4 },{ "DEC", &a::DEC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLD", &a::CLD, &a::IMP, 2 },{ "CMP", &a::CMP, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ABX, 4 },{ "DEC", &a::DEC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
    //E { "CPX", &a::CPX, &a::IMM, 2 },{ "SBC", &a::SBC, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPX", &a::CPX, &a::ZP0, 3 },{ "SBC", &a::SBC, &a::ZP0, 3 },{ "INC", &a::INC, &a::ZP0, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INX", &a::INX, &a::IMP, 2 },{ "SBC", &a::SBC, &a::IMM, 2 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::SBC, &a::IMP, 2 },{ "CPX", &a::CPX, &a::ABS, 4 },{ "SBC", &a::SBC, &a::ABS, 4 },{ "INC", &a::INC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
    //F { "BEQ", &a::BEQ, &a::REL, 2 },{ "SBC", &a::SBC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ZPX, 4 },{ "INC", &a::INC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SED", &a::SED, &a::IMP, 2 },{ "SBC", &a::SBC, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ABX, 4 },{ "INC", &a::INC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },