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

impl BusInterface for Cpu2A03 {
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

pub struct Cpu2A03 {
    pub a: u8,                      // Accumulator Register
    pub x: u8,                      // X Register
    pub y: u8,                      // Y Register
    pub sp: u8,                     // Stack Pointer
    pub pc: u16,                    // Program Counter
    
    pub status: ProcessorStatus,    // Status Register

    // internal registers
    iclocks:u8,                     // total clocks for the instruction
    instr: u8,                      // active instruction register
    temp_addr: u8,                  // scratch register for some of the addressing modes **Only necessary 
                                    //      because I'm lazy and wanted to write generic address translations 
                                    //      instead of special for each instruction


    // bus interface
    bus_ctrl: BusControlStatus,     // The R/W control flag
    bus_address: u16,               // The address pinout...
    bus_data: u8,                   // The data pinout

    // incoming signals
    pub reset:bool,                 // <- reset interrupt
    pub nmi:bool,                   // <- nmi interrupt
    pub irq:bool,                   // <- irq interrupt

    // outgoing signals
    pub oe1:bool,                   // -> Controller 1 dump
    pub oe2:bool,                   // -> Controller 2 dump
}

impl Clocked for Cpu2A03
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
                self.instr = ternary(self.is_interrupted(), OP_BRK, self.bus_data()); // all interrupts are handled as BRK
                self.fetch_pc();    // always fetch the operand too
            }
            _ => // store address mode + handle instruction
            {
                let (opcode, address_mode, c1, c2) = OPCODE_TABLE[self.instr as usize];   // decode the instruction
                let (is_fetch_done, operand) = self.fetch_operand(address_mode);     // translate the addressing mode

                if is_fetch_done
                {
                    // finally perform the operation
                    let is_done = self.execute_opcode(opcode, operand) ;

                    // and fetch the next instruction
                    needs_instr = is_done;    // no more clocks, get the next instruction

                    //--TODO: verify clock cycles are correct
                }
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

impl Cpu2A03 {
    pub fn new() -> Self {
        Cpu2A03 {
            a: 0,
            x: 0,
            y: 0,
            sp: STACK_START,
            pc: PC_START,

            status: ProcessorStatus::initial(), 

            iclocks:0,
            instr:0,
            temp_addr: 0,
            
            bus_ctrl:BusControlStatus::Read,
            bus_address:0,
            bus_data:0,

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

    fn poll_interrupts(&mut self) 
    {
        if self.status.contains(ProcessorStatus::Interrupt) == false    // make sure we aren't already in an interrupt
            && ( self.reset || self.nmi || self.irq )
        {
            // set the interrupt flag and reset the signals
            self.status.set(ProcessorStatus::Interrupt, true);
        }
    }
    
    /**************************
     * Memory Addressing Modes
     **************************/

    fn fetch_operand(&mut self, mode: AddressingMode) -> (bool, u8)
    {
        // 6502 bus always reads the bus address, so the next program counter
        // address has already been read by the time we get here.  Which means
        // that any additional fetches done by this function start at byte 3 of
        // the instruction and won't be available until cycle 3

        match mode {
            AddressingMode::Implicit => (true, 0),                      // Implicit means the instruction doesn't use anything
            AddressingMode::Immediate => (true,self.bus_data()),        // Immediate means to use the fetched bus value directly
            AddressingMode::Accumulator => (true,self.a),               // this mode just means to use the accumulator directly
            _ => self.fetch_operand_from_address(mode),
        }
    }

    fn fetch_operand_from_address(&mut self, mode: AddressingMode) -> (bool, u8)
    {
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

    fn fetch_operand_zp(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.fetch(self.bus_data() as u16); (false,0) }
            2 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_zpx(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.fetch(self.bus_data() as u16); (false,0) }
            // value + x
            // Force to stay in 0 page by wrapping 
            2 => { self.fetch(self.bus_data().wrapping_add(self.x) as u16); (false,0) }    
            3 => { (true, self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_zpy(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.fetch(self.bus_data() as u16); (false,0) }
            // value + x
            // Force to stay in 0 page by wrapping 
            2 => { self.fetch(self.bus_data().wrapping_add(self.y) as u16); (false,0) }    
            3 => { (true, self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_abs(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.temp_addr = self.bus_data(); self.fetch_pc(); (false,0) }
            2 => { self.fetch( WORD::make(self.bus_data(), self.temp_addr)); (false,0) }
            3 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }
    
    fn fetch_operand_absx(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.temp_addr = self.bus_data(); self.fetch_pc(); (false,0) }
            2 => { self.fetch( WORD::make(self.bus_data(), self.temp_addr).wrapping_add(self.x as u16)); (false,0) }
            3 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_absy(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.temp_addr = self.bus_data(); self.fetch_pc(); (false,0) }
            2 => { self.fetch( WORD::make(self.bus_data(), self.temp_addr).wrapping_add(self.y as u16)); (false,0) }
            3 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_ind(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.fetch(self.bus_data() as u16); (false,0) }
            2 => { 
                self.temp_addr = self.bus_data();
                self.fetch(self.temp_addr as u16); 
                (false,0)
            }    
            3 => { 
                self.fetch(self.temp_addr.wrapping_add(1) as u16);
                self.temp_addr = self.bus_data();   // lo byte
                (false,0)
            }
            4 => 
            { 
                let ptr = WORD::make(self.bus_data(), self.temp_addr);
                self.fetch(ptr);
                (false,0)
            }
            5 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_izx(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.fetch(self.bus_data() as u16); (false,0) }
            2 => { 
                self.temp_addr = self.bus_data().wrapping_add(self.x);
                self.fetch(self.temp_addr as u16); 
                (false,0)
            }    
            3 => { 
                self.fetch(self.temp_addr.wrapping_add(1) as u16);
                self.temp_addr = self.bus_data();   // lo byte
                (false,0)
            }
            4 => 
            { 
                let ptr = WORD::make(self.bus_data(), self.temp_addr);
                self.fetch(ptr);
                (false,0)
            }
            5 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }

    fn fetch_operand_izy(&mut self) -> (bool, u8)
    {
        match self.iclocks {
            1 => { self.fetch(self.bus_data() as u16); (false,0) }
            2 => { 
                self.temp_addr = self.bus_data();
                self.fetch(self.temp_addr as u16); 
                (false,0)
            }    
            3 => { 
                self.fetch(self.temp_addr.wrapping_add(1) as u16);
                self.temp_addr = self.bus_data();   // lo byte
                (false,0)
            }
            4 => 
            { 
                let ptr = WORD::make(self.bus_data(), self.temp_addr).wrapping_add(self.y as u16);
                self.fetch(ptr);
                (false,0)
            }
            5 => { (true,self.bus_data()) }
            _ => { panic!() }
        }
    }

    /*************************
     * Instruction Execution
     *************************/

    fn execute_opcode(&mut self, opcode: Operation, operand: u8) -> bool
    {
        match opcode {
            Operation::BRK => self.op_brk(),
            Operation::TAX => self.op_tax(),
            Operation::LDA => self.op_lda(operand),
            _ => todo!("Not implemented instruction")
        }
    }

    fn op_brk(&mut self) -> bool
    {
        match self.iclocks {
            1 => { 
                /* push pc hi on stack */
                self.stack_push(WORD::hi(self.pc));
            }
            2 => { 
                /* push pc lo on stack */
                self.stack_push(WORD::lo(self.pc));
            }
            3 => { 
                /* push status on stack, */
                let status = self.status.bits 
                    | ternary(self.is_interrupted(),    // d flag is only set on an interrupt
                                (ProcessorStatus::Decimal | ProcessorStatus::Break).bits,
                                ProcessorStatus::Break.bits);
                
                //--TODO!
                /*** At this point, the signal status determines which interrupt vector is used ***/
                self.stack_push(status);
            }
            4 => { 
                /* reset, fetch pc lo */
                self.pc = PC_START;
                self.status.insert(ProcessorStatus::Interrupt);
                self.reset = false;
                self.nmi = false;
                self.irq = false;

                self.fetch_pc();
            }
            5 => { 
                /* save pc lo and fetch pc hi */ 
                self.temp_addr = self.bus_data();
                self.fetch_pc();
            }
            6 => { 
                /* save pc hi and fetch next instr */
                self.pc = WORD::make(self.bus_data(), self.temp_addr);
                return true;        // break is over.. boo
            }
            _ => panic!("Incorrect instruction opcode handling, did not reset instr cycle counter properly")
        }

        return false;           // not done yet
    }

    fn op_tax(&mut self) -> bool
    {
        self.x = self.a ;
        self.update_status(self.x) ;
        return true;            // we're done
    }

    fn op_lda(&mut self, operand: u8) -> bool
    {
        self.a = operand;
        self.update_status(operand);
        return true;           // we're done
    }
}

