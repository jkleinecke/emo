#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::bitflags::*;
use crate::common::{test_bit, ternary, Clocked, WORD};
use crate::bus::{BusInterface, BusControlStatus};
use crate::operations::*;

pub const PC_START: u16 = 0xFFFC;
const SP_START: u8 = 0xFD;
const OP_BRK: u8 = 0x00;


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

    pub fn initial() -> ProcessorStatus {
        ProcessorStatus::Break | ProcessorStatus::Unused | ProcessorStatus::Interrupt   // 0x34
    }
}

impl BusInterface for EmoC6502 {
    fn get_control_status(&self) -> BusControlStatus {

        ternary(self.reset              // don't allow writes on reset
            , BusControlStatus::Read    // If you can't write, you must be reading .. signal can only be one or the other
            , self.bus_ctrl)        
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

    // internal registers
    iclocks:u8,                     // total clocks for the instruction
    instr: u8,                      // active instruction register
    al: u8,                         // alu register
    
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
        // 1. Handle any cpu interrupts
        // Interrupts are handled on the next instruction fetch - See clock 0
        let mut needs_instr = false;

        // Actual Clock Cycle
        match self.iclocks 
        {

            0 => // store instruction + fetch address mode
            {
                // store the fetched instruction
                self.instr = ternary(self.is_interrupted(), OP_BRK, self.data); // all interrupts are handled as BRK
                self.fetch_pc();    // always fetch the operand too
            }
            _ => // store address mode + handle instruction
            {
                let opcode = OPCODE_TABLE[self.instr as usize];   // decode the instruction
                let fetched = self.data;

                // translate addressing mode
                let operand:u8 = match opcode {
                    (_,AddressingMode::Implicit,_,_) => fetched,
                    (_,AddressingMode::Immediate,_,_) => fetched,
                    _ => todo!("Not implemented address modes!"),
                };

                // finally perform the operation
                let remaining_clocks = match opcode {
                    (Operation::BRK, ..) => self.exec_op_brk(operand),
                    (Operation::LDA, ..) => self.exec_op_lda(operand),
                    _ => todo!("Not implemented instruction")
                };

                // and fetch the next instruction
                needs_instr = remaining_clocks == 0;    // no more clocks, get the next instruction
            }
        }

        if needs_instr {
            self.fetch_pc();
            self.iclocks = 0;
        }
        else {
            self.iclocks += 1;  // inc the instruction clock count
        }

        // Check for interrupt
        self.poll_interrupts();
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

            status: ProcessorStatus::initial(), 

            iclocks:0,
            instr:0,
            al:0,

            bus_ctrl:BusControlStatus::Read,
            address:0,
            data:0,

            reset:true,
            nmi:false,
            irq:false,

            oe1:false,
            oe2:false,
        }
    }

    fn fetch_pc(&mut self) {
        self.fetch(self.pc);
        self.pc += 1;
    }

    fn fetch(&mut self, addr: u16) {
        self.bus_ctrl = BusControlStatus::Read;     // redundant, read should still be set
        self.address = addr;
    }

    fn store(&mut self, addr: u16, v: u8) {
        self.bus_ctrl = BusControlStatus::Write;
        self.address = addr;
        self.data = v;
    }

    fn update_status(&mut self, result:u8)
    {
        self.status.set(ProcessorStatus::Zero, result == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(result,7));
    }

    fn is_interrupted(&self) -> bool {
        self.status.contains(ProcessorStatus::Interrupt)
    }

    fn poll_interrupts(&mut self) 
    {
        if self.status.contains(ProcessorStatus::Interrupt) == false    // make sure we aren't already in an interrupt
            && ( self.reset || self.nmi || self.irq )
        {
            // set the interrupt flag and reset the signals
            self.status.set(ProcessorStatus::Interrupt, true);
            self.nmi = false;
            self.irq = false;
        }
    }

    fn exec_op_brk(&mut self, operand: u8) -> u8
    {
        match self.iclocks {
            1 => { /* push pc hi on stack */ }
            2 => { /* push pc lo on stack */ }
            3 => { /* push status on stack, */ }
            4 => { 
                /* reset, fetch pc lo */
                self.pc = PC_START;
                self.status.remove(ProcessorStatus::Interrupt);
                self.reset = false;

                self.fetch_pc();
            }
            5 => { 
                /* save pc lo and fetch pc hi */ 
                self.al = self.data;
                self.fetch_pc();
            }
            6 => { 
                /* save pc hi and fetch next instr */
                self.pc = WORD::make(self.data, self.al);
                return 0;
            }
            _ => panic!("Incorrect instruction opcode handling, did not reset instr cycle counter properly")
        }

        return 1;           // todo: make this more accurate
    }

    fn exec_op_lda(&mut self, operand: u8) -> u8
    {
        self.a = operand;
        self.update_status(operand);
        return 0;           // we're done, no more cycles
    }
}

