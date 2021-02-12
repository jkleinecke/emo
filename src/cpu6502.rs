#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::bitflags::*;
use crate::common::{test_bit, ternary, Clocked, WORD};
use crate::bus::{BusInterface, BusControlStatus};
use crate::operations::*;

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

impl BusInterface for Cpu6502 {
    fn get_control_status(&self) -> BusControlStatus {

        ternary(self.reset              // don't allow writes on reset
            , BusControlStatus::Read    // If you can't write, you must be reading .. signal can only be one or the other
            , self.bus_ctrl)        
    }
    fn get_address(&self) -> u16 {
        self.bus_address
    }
    fn bus_data(&self) -> u8 {
        self.bus_data
    }
    fn set_bus_data(&mut self, v:u8) {
        self.bus_data = v;
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
    ir_cycles: u8,
    ir: u8,                         // active instruction register
    ad: u16,                        // ADL / ADH internal register

    // bus interface
    bus_ctrl: BusControlStatus,     // The R/W control flag
    bus_address: u16,               // The address pinout...
    bus_data: u8,                   // The data pinout

    // incoming signals
    pub sync:bool,                  // <- fetch next ir, also indicates it is safe to handle interrupt

    pub reset:bool,                 // <- reset interrupt
    pub nmi:bool,                   // <- nmi interrupt
    pub irq:bool,                   // <- irq interrupt

    // outgoing signals
    pub oe1:bool,                   // -> Controller 1 dump
    pub oe2:bool,                   // -> Controller 2 dump
}

impl Clocked for Cpu6502
{
    fn clock(&mut self) 
    {
        if self.sync
        {
            // 1. Handle any cpu interrupts
            let is_interrupted = self.check_interrupts();
            // store the fetched bus data as the new ir
            let opcode = ternary(is_interrupted, OP_BRK, self.bus_data());  // replace the IR with BRK if an interrupt has been requested

            self.ir = opcode ;
            self.ir_cycles = 0 ;
            self.sync = false;  // reset the sync signal
        }
        
        // decode the opcode
        let (operation, addr_mode, c1, c2) = OPCODE_TABLE[self.ir as usize];

        self.execute_opcode(operation, addr_mode);
        
        // if the sync signal is set, let's verify the clock cycles
        assert!(self.ir_cycles < (c1 + c2)); 
        self.ir_cycles += 1;

    }
}

impl Cpu6502 {
    pub fn new() -> Self {
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
            
            bus_ctrl:BusControlStatus::Read,
            bus_address:0,
            bus_data:0,

            sync:true,
            reset:true,
            nmi:false,
            irq:false,

            oe1:false,
            oe2:false,
        }
    }

    fn fetch_ir(&mut self) {
        self.fetch_pc();
        self.sync = true;
    }

    fn fetch_pc(&mut self) {
        self.fetch(self.pc);
        self.pc = self.pc.wrapping_add(1);
    }

    fn fetch(&mut self, addr: u16) {
        self.bus_ctrl = BusControlStatus::Read;     // redundant, read should still be set
        self.bus_address = addr;
    }

    fn store(&mut self, addr: u16, v: u8) {
        self.bus_ctrl = BusControlStatus::Write;
        self.bus_address = addr;
        self.set_bus_data(v);
    }

    fn stack_push(&mut self, v:u8) 
    {
        let addr = STACK_BASE + self.sp as u16;    // calc the stack pointer address
        self.store(addr, v) ;               // queue up the bus write
        self.sp -= 1;                       // decrement the stack pointer
    }

    fn stack_pop(&mut self)
    {
        self.sp += 1;                       // increment the stack pointer
        let addr = STACK_BASE + self.sp as u16;    // calc the stack pointer address
        self.fetch(addr) ;                  // queue up the bus read
    }

    fn update_status(&mut self, result:u8)
    {
        self.status.set(ProcessorStatus::Zero, result == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(result,7));
    }

    fn is_interrupted(&self) -> bool {
        self.status.contains(ProcessorStatus::Interrupt)
    }

    fn check_interrupts(&mut self) -> bool
    {
        (self.status.contains(ProcessorStatus::Interrupt) == false && self.irq)    // make sure we aren't already in an interrupt
        || self.nmi                                                                // can't be disabled
        || self.reset                                                              // treat resets the same as an interrupt 
    }
    
    /**************************
     * Memory Addressing Modes
     **************************/

    fn fetch_operand(&mut self, mode: AddressingMode) -> Option<u8>
    {
        // 6502 bus always reads the bus address, so the next program counter
        // address has already been read by the time we get here.  Which means
        // that any additional fetches done by this function start at byte 3 of
        // the instruction and won't be available until cycle 3

        match mode {
            AddressingMode::Implicit => Some(0),                      // Implicit means the instruction doesn't use anything
            AddressingMode::Immediate => Some(self.bus_data()),        // Immediate means to use the fetched bus value directly
            AddressingMode::Accumulator => Some(self.a),               // this mode just means to use the accumulator directly
            _ => self.fetch_operand_from_address(mode),
        }
    }

    fn fetch_operand_from_address(&mut self, mode: AddressingMode) -> Option<u8>
    {
        //--TODO: Handle oops cycle when we cross a page..

        match mode {
            AddressingMode::ZeroPage => self.fetch_operand_zp(),
            AddressingMode::ZeroPageX => self.fetch_operand_zpx(),
            AddressingMode::ZeroPageY => self.fetch_operand_zpy(),
            AddressingMode::Absolute => self.fetch_operand_abs(),
            AddressingMode::AbsoluteX => self.fetch_operand_absx(),
            AddressingMode::AbsoluteY => self.fetch_operand_absy(),
            AddressingMode::Indirect => self.fetch_operand_ind(),
            AddressingMode::IndirectX => self.fetch_operand_izx(),
            AddressingMode::IndirectY => self.fetch_operand_izy(),
            _ => todo!("Not implemented address modes!"),
        }
    }

    fn fetch_operand_zp(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { self.fetch(self.bus_data() as u16); None },
            2 => Some(self.bus_data()),
            _ => panic!(),
        }
    }

    fn fetch_operand_zpx(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { self.fetch(self.bus_data() as u16); None }
            // value + x
            // Force to stay in 0 page by wrapping 
            2 => { self.fetch(self.bus_data().wrapping_add(self.x) as u16); None }    
            3 => { Some(self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_zpy(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { self.fetch(self.bus_data() as u16); None }
            // value + x
            // Force to stay in 0 page by wrapping 
            2 => { self.fetch(self.bus_data().wrapping_add(self.y) as u16); None }    
            3 => { Some(self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_abs(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { self.ad.set_lo(self.bus_data()); self.fetch_pc(); None }
            2 => { self.ad.set_hi(self.bus_data()); self.fetch( self.ad); None }
            3 => { Some(self.bus_data()) }
            _ => { panic!() }
        }
    }
    
    fn fetch_operand_absx(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { self.ad.set_lo(self.bus_data()); self.fetch_pc(); None }
            2 => { 
                self.ad.set_hi(self.bus_data());
                let addr = self.ad.wrapping_add(self.x as u16);
                self.fetch(addr);
                self.status.set(ProcessorStatus::Overflow, self.ad.hi() != addr.hi());  // oops flag
                None
            }
            3 => { 
                if self.status.contains(ProcessorStatus::Overflow) == false {   //oops cycle check
                    return Some(self.bus_data());
                }
                None
            }
            4 => {
                self.status.remove(ProcessorStatus::Overflow);
                Some(self.bus_data())   // oops cycle
            }
            _ => { panic!() }
        }
    }

    fn fetch_operand_absy(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { self.ad.set_lo(self.bus_data()); self.fetch_pc(); None }
            2 => { 
                self.ad.set_hi(self.bus_data());
                let addr = self.ad.wrapping_add(self.y as u16);
                self.fetch(addr);
                self.status.set(ProcessorStatus::Overflow, self.ad.hi() != addr.hi());  // oops flag
                None
            }
            3 => { 
                if self.status.contains(ProcessorStatus::Overflow) == false {   //oops cycle check
                    return Some(self.bus_data());
                }
                None
            }
            4 => {
                self.status.remove(ProcessorStatus::Overflow);
                Some(self.bus_data())   // oops cycle
            }
            _ => { panic!() }
        }
    }

    fn fetch_operand_ind(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { 
                self.ad = self.bus_data() as u16;
                self.fetch(self.ad as u16); 
                None
            }    
            2 => { 
                self.fetch(self.ad.wrapping_add(1) as u16);
                self.ad.set_lo(self.bus_data());   // lo byte
                None
            }
            3 => 
            { 
                self.ad.set_hi(self.bus_data());
                self.fetch(self.ad);
                None
            }
            4 => { Some(self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_izx(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { 
                self.ad = self.bus_data().wrapping_add(self.x) as u16;
                None
            }
            2 => { 
                self.fetch(self.ad); 
                None
            }    
            3 => { 
                self.fetch(self.ad.wrapping_add(1) as u16);
                self.ad.set_lo(self.bus_data());   // lo byte
                None
            }
            4 => 
            { 
                self.ad.set_hi(self.bus_data());
                self.fetch(self.ad);
                None
            }
            5 => { Some(self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_izy(&mut self) -> Option<u8>
    {
        match self.ir_cycles {
            1 => { 
                self.ad = self.bus_data() as u16;
                self.fetch(self.ad as u16); 
                None
            }    
            2 => { 
                self.fetch(self.ad.wrapping_add(1) as u16);
                self.ad.set_lo(self.bus_data());   // lo byte
                None
            }
            3 => 
            { 
                self.ad.set_hi(self.bus_data());
                let addr = self.ad.wrapping_add(self.y as u16);
                self.fetch(addr);
                self.status.set(ProcessorStatus::Overflow, self.ad.hi() != addr.hi());  // oops flag
                None
            }
            4 => { 
                if self.status.contains(ProcessorStatus::Overflow) == false {   //oops cycle check
                    return Some(self.bus_data());
                }
                None
            }
            5 => {
                self.status.remove(ProcessorStatus::Overflow);
                Some(self.bus_data())   // oops cycle
            }
            _ => { panic!() }
        }
    }

    /*************************
     * Instruction Execution
     *************************/

    fn execute_opcode(&mut self, opcode: Operation, addr_mode: AddressingMode)
    {
        if self.ir_cycles == 0
        {
            self.fetch_pc();
            return;             // just fetch the next program byte..
        }
        
        match opcode {
            Operation::BRK => self.op_brk(addr_mode),
            Operation::TAX => self.op_tax(addr_mode),
            Operation::LDA => self.op_lda(addr_mode),
            _ => todo!("Not implemented instruction")
        };
    }

    fn op_brk(&mut self, addr_mode: AddressingMode)
    {
        match self.ir_cycles {
            1 => { 
                /* push pc hi on stack */
                self.stack_push(self.pc.hi());
            }
            2 => { 
                /* push pc lo on stack */
                self.stack_push(self.pc.lo());
            }
            3 => { 
                /* push status on stack, */
                let mut status = self.status.bits | ProcessorStatus::Unused.bits;
                
                if self.nmi || self.irq {
                    status |= ProcessorStatus::Interrupt.bits;
                }
                else if self.reset == false {
                    status |= ProcessorStatus::Break.bits;
                }                    
                
                //--TODO!
                /*** At this point, the signal status determines which interrupt vector is used ***/
                self.stack_push(status);
            }
            4 => { 
                /* reset, fetch pc lo */
                self.pc = PC_START;
                self.status.set(ProcessorStatus::Interrupt, self.nmi || self.irq) ;
                self.reset = false;
                self.nmi = false;
                self.irq = false;

                self.fetch_pc();
            }
            5 => { 
                /* save pc lo and fetch pc hi */ 
                self.ad.set_lo(self.bus_data());
                self.fetch_pc();
            }
            6 => { 
                /* save pc hi and fetch next instr */
                self.ad.set_hi(self.bus_data());
                self.pc = self.ad;

                self.fetch_ir();    // done, fetch the next instruction
            }
            _ => panic!("Incorrect instruction opcode handling, did not reset instr cycle counter properly")
        }
    }

    fn op_tax(&mut self, addr_mode: AddressingMode)
    {
        self.x = self.a ;
        self.update_status(self.x) ;
        
        self.fetch_ir();
    }

    fn op_lda(&mut self, addr_mode: AddressingMode)
    {
        let operand = self.fetch_operand(addr_mode);

        if operand.is_some()
        {
            self.a = operand.unwrap();
            self.update_status(self.a);

            self.fetch_ir();
        }
    }
}

