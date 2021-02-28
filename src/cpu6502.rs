#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::bitflags::*;
use crate::common::{test_bit, ternary, WORD};
use crate::bus::Bus;
use crate::system::Clocked;
use crate::operations::*;
use std::rc::Rc;
use std::cell::RefCell;

/***********************************************************
 * 
 * Emulates the 2A03 NES CPU 
 * 
 * Variant of the 65c02, omits the decimal BCD mode entirely
 * 
 * The actual chip included the sound hardware which is not
 * being emulated here and instead will be treated as a 
 * separate module on the bus. 
 * 
 ***********************************************************/

pub const PC_START: u16 = 0xFFFC;

const STACK_BASE: u16 = 0x100;
const STACK_START: u8 = 0xFF;

const OP_BRK: u8 = 0x00;

bitflags! {
    #[derive(Default)]
    pub struct ProcessorStatus: u8 {
        const Carry       = 0b00000001;
        const Zero        = 0b00000010;
        const Interrupt   = 0b00000100;
        const Decimal     = 0b00001000;         // Not really used

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
        ProcessorStatus::Unused | ProcessorStatus::Interrupt   // 0x34
    }
}

pub struct Cpu6502 {
    pub a: u8,                      // Accumulator Register
    pub x: u8,                      // X Register
    pub y: u8,                      // Y Register
    pub sp: u8,                     // Stack Pointer
    pc: u16,                        // Internal Program Counter Register
    
    pub status: ProcessorStatus,    // Status Register

    // internal registers
    pub ir_cycles: u8,
    ir: u8,                         // active instruction register
    ad: u16,                        // ADL / ADH internal register

    // bus module
    pub bus: Box<Bus>,

    // outgoing signals
    pub oe1:bool,                   // -> Controller 1 dump
    pub oe2:bool,                   // -> Controller 2 dump
}

impl Clocked for Cpu6502
{
    fn clock(&mut self) 
    {
        // todo - Handle any cpu interrupts

        if self.ir_cycles > 0
        {
            // wait the appropriate number of cycles
            // for the last instruction that was executed

            self.ir_cycles -= 1;
            return;             
        }

        // Fetch the IR
        self.ir = self.mem_fetch(self.pc);
        self.inc_pc();

        // Decode the IR
        let (operation, addr_mode, c1, c2) = OPCODE_TABLE[self.ir as usize];
        self.ir_cycles = c1;


        // Execute the IR operation
        self.execute_opcode(operation, addr_mode);
    }
}

impl Cpu6502 {
    pub fn new(bus:Box<Bus>) -> Self {
        Cpu6502 {
            a: 0,
            x: 0,
            y: 0,
            sp: STACK_START,
            pc: PC_START,

            status: ProcessorStatus::initial(), 
            
            ir_cycles: 0,
            ir:0,
            ad: 0,

            bus,

            oe1:false,
            oe2:false,
        }
    }

    pub fn reset(&mut self) {
        /* push pc hi on stack */
        self.stack_push(self.pc.hi());
        /* push pc lo on stack */
        self.stack_push(self.pc.lo());
        
        /* push status on stack, */
        let status = self.status.bits | ProcessorStatus::Unused.bits;
        
        //--TODO!
        /*** At this point, the signal status determines which interrupt vector is used ***/
        self.stack_push(status);
        
        self.status.set(ProcessorStatus::Interrupt, false) ;
        
        self.pc = self.mem_fetch16(PC_START);
        
        self.ir_cycles =7;
    }

    pub fn nmi(&mut self) {
        panic!();
    }

    pub fn irq(&mut self) {
        panic!();
    }
    
    // I/O Memory Bus

    fn inc_pc(&mut self) {
        self.pc = self.pc.wrapping_add(1);
    }

    fn mem_fetch(&mut self, addr: u16) -> u8 {
        self.bus.load(addr)
    }

    fn mem_store(&mut self, addr: u16, v: u8) {
        self.bus.store(addr,v);
    }

    fn mem_fetch16(&mut self, addr: u16) -> u16 {
        let lo = self.bus.load(addr);
        let hi = self.bus.load(addr + 1);
        u16::make(hi, lo)
    }

    fn mem_store16(&mut self, addr: u16, v: u16) {
        self.bus.store(addr, v.lo());
        self.bus.store(addr+1, v.hi());
    }

    fn stack_push(&mut self, v:u8) 
    {
        let addr = STACK_BASE + self.sp as u16;    // calc the stack pointer address
        self.bus.store(addr, v) ;               // queue up the bus write
        self.sp -= 1;                       // decrement the stack pointer
    }

    fn stack_pop(&mut self) -> u8
    {
        self.sp += 1;                       // increment the stack pointer
        let addr = STACK_BASE + self.sp as u16;    // calc the stack pointer address
        self.bus.load(addr)                   // bus read
    }

    // Status Flags

    fn update_status(&mut self, result:u8)
    {
        self.status.set(ProcessorStatus::Zero, result == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(result,7));
    }

    // Interrupt checks

    fn is_interrupted(&self) -> bool {
        self.status.contains(ProcessorStatus::Interrupt)
    }
    
    /**************************
     * Memory Addressing Modes
     **************************/

    fn fetch_operand_address(&mut self, mode: AddressingMode) -> u16
    {
        //--TODO: Handle oops cycle when we cross a page..

        // We could utilize a common ALU method for adding that would
        // have a side-effect of changing the status flags automatically
        // for us... 
        //
        // then the oops cycle could be triggered by the overflow flag..

        match mode {
            AddressingMode::Immediate => self.pc,
            AddressingMode::ZeroPage => self.mem_fetch(self.pc) as u16,
            AddressingMode::ZeroPageX => {
                let pos = self.mem_fetch(self.pc);
                let addr = pos.wrapping_add(self.x) as u16;
                addr
            }
            AddressingMode::ZeroPageY => {
                let pos = self.mem_fetch(self.pc);
                let addr = pos.wrapping_add(self.y) as u16;
                addr
            }
            AddressingMode::Absolute => self.mem_fetch16(self.pc),
            AddressingMode::AbsoluteX => {
                let base = self.mem_fetch16(self.pc);
                self.inc_pc();
                let addr = base.wrapping_add(self.x as u16);
                addr
            }
            AddressingMode::AbsoluteY => {
                let base = self.mem_fetch16(self.pc);
                self.inc_pc();
                let addr = base.wrapping_add(self.y as u16);
                addr
            }
            AddressingMode::Indirect => {
                let base = self.mem_fetch(self.pc);
                let lo = self.mem_fetch(base as u16);
                let hi = self.mem_fetch(base.wrapping_add(1) as u16);
                u16::make(hi,lo)
            },
            AddressingMode::IndirectX => {
                let base = self.mem_fetch(self.pc);
                let ptr = base.wrapping_add(self.x);
                let lo = self.mem_fetch(ptr as u16);
                let hi = self.mem_fetch(ptr.wrapping_add(1) as u16);
                u16::make(hi, lo)
            }
            AddressingMode::IndirectY => {
                let base = self.mem_fetch(self.pc);
                let lo = self.mem_fetch(base as u16);
                let hi = self.mem_fetch(base.wrapping_add(1) as u16);
                let ptr_base = u16::make(hi,lo);
                let ptr = ptr_base.wrapping_add(self.y as u16);
                ptr
            },
            _ => todo!("Not implemented address modes!"),
        }
    }

    /*************************
     * Instruction Execution
     *************************/

    fn execute_opcode(&mut self, opcode: Operation, addr_mode: AddressingMode)
    {
        match opcode {
            Operation::BRK => self.op_brk(addr_mode),
            Operation::TAX => self.op_tax(addr_mode),
            Operation::LDA => self.op_lda(addr_mode),
            _ => todo!("Not implemented instruction")
        };
    }

    fn op_brk(&mut self, addr_mode: AddressingMode)
    {
        /* push pc hi on stack */
        self.stack_push(self.pc.hi());
        /* push pc lo on stack */
        self.stack_push(self.pc.lo());
        /* push status on stack, */
        let status = self.status.bits | ProcessorStatus::Unused.bits;
            
            // if self.nmi || self.irq {
            //     status |= ProcessorStatus::Interrupt.bits;
            // }
            // else if self.reset == false {
            //     status |= ProcessorStatus::Break.bits;
            // }                    
            
        //--TODO!
        /*** At this point, the signal status determines which interrupt vector is used ***/
        self.stack_push(status);

        /* reset, fetch pc lo */
        self.pc = PC_START;
        // self.status.set(ProcessorStatus::Interrupt, self.nmi || self.irq) ;

        self.pc = self.mem_fetch16(PC_START);
    }

    fn op_tax(&mut self, addr_mode: AddressingMode)
    {
        self.inc_pc();  // all instructions must move the pc forward 1, no matter if they use it

        self.x = self.a ;
        self.update_status(self.x) ;
    }

    fn op_lda(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.a = self.mem_fetch(addr);
        self.inc_pc();

        self.update_status(self.a);
    }
}

