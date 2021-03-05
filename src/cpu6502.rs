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

    halted: bool,                   // fake register to show that we should be done

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
        self.ir_cycles = c1 - 1 ;   // minus 1 for "this" cycle

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

            halted: false,

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

    pub fn did_halt(&self) -> bool {
        self.halted
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
            AddressingMode::Immediate => {
                let addr = self.pc;
                self.inc_pc();
                addr
            }
            AddressingMode::ZeroPage => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                self.a as u16
            }
            AddressingMode::ZeroPageX => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                self.alu_add(self.x);
                self.a as u16
            }
            AddressingMode::ZeroPageY => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                self.alu_add(self.y);
                self.a as u16
            }
            AddressingMode::Absolute => {
                let lo = self.mem_fetch(self.pc);
                self.inc_pc();
                self.a = self.mem_fetch(self.pc);     // just making sure the registers are used properly here
                self.inc_pc();
                u16::make(self.a, lo)
            }
            AddressingMode::AbsoluteX => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                self.alu_add(self.x);
                let lo = self.a;

                self.a = self.mem_fetch(self.pc);
                self.inc_pc();

                if self.status.contains(ProcessorStatus::Carry)
                {
                    // this is the oops cycle work here.. the lo-byte
                    // addition overflowed which means we need to spend
                    // an extra cycle adding the carry to the hi-byte
                    self.alu_add(1);        // adding 0 works because all add operations already include the carry
                    self.ir_cycles += 1;    // add the oops cycle into it
                }

                u16::make(self.a, lo)
            }
            AddressingMode::AbsoluteY => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                self.alu_add(self.y);
                let lo = self.a;

                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                
                if self.status.contains(ProcessorStatus::Carry)
                {
                    // this is the oops cycle work here.. the lo-byte
                    // addition overflowed which means we need to spend
                    // an extra cycle adding the carry to the hi-byte
                    self.alu_add(1);        // add the carry
                    self.ir_cycles += 1;    // add the oops cycle into it
                }

                u16::make(self.a, lo)
            }
            // AddressingMode::Indirect => {
            //     let base = self.mem_fetch(self.pc);
            //     let lo = self.mem_fetch(base as u16);
            //     let hi = self.mem_fetch(base.wrapping_add(1) as u16);
            //     u16::make(hi,lo)
            // },
            AddressingMode::IndirectX => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();
                self.alu_add(self.x);

                let lo = self.mem_fetch(self.a as u16);
                self.alu_add(1);
                let hi = self.mem_fetch(self.a as u16);
                u16::make(hi, lo)
            }
            AddressingMode::IndirectY => {
                self.a = self.mem_fetch(self.pc);
                self.inc_pc();

                let lo = self.mem_fetch(self.a as u16);
                self.alu_add(1);
                let hi = self.mem_fetch(self.a as u16);

                self.a = lo;
                self.alu_add(self.y);
                let ptr_lo = self.a;
                self.a = hi;

                if self.status.contains(ProcessorStatus::Carry)
                {
                    // this is the oops cycle work here.. the lo-byte
                    // addition overflowed which means we need to spend
                    // an extra cycle adding the carry to the hi-byte
                    self.alu_add(1);        // add the carry
                    self.ir_cycles += 1;    // add the oops cycle into it
                }

                u16::make(self.a,ptr_lo)
            },
            _ => panic!("address mode {:?} is not supported", mode),
        }
    }

    fn fetch_jmp_address(&mut self) -> u16
    {
        self.a = self.mem_fetch(self.pc);
        self.inc_pc();

        self.alu_add(self.pc.lo());

        u16::make(self.pc.hi(), self.a)
    }

    /*************************
     * ALU operations
     *************************/

    fn alu_add(&mut self, v:u8)
    {
        // a rather ambitious attempt to generalize the alu operations with
        // the status flag updates
        //
        // Important to note that the result does not actually wrap around.
        //  A second add operation is required to account for the carry bit
        //
        // CPU status flags are then set to indicate
        //  zero, 
        //  negative, 
        //  another carry was generated
        //  signed overflow occurred if the values are interpreted as signed

        let result = self.a as u16 + v as u16;
        let signed_overflow = !(self.a ^ v) & (self.a ^ result.lo());

        self.a = result.lo();

        self.status.set(ProcessorStatus::Zero, self.a == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
        self.status.set(ProcessorStatus::Carry, result.hi() > 0);
        self.status.set(ProcessorStatus::Overflow, test_bit(signed_overflow, 7));
    }

    fn alu_sub(&mut self, v:u8)
    {
        // further ambitious bullshit from me...

        self.status.set(ProcessorStatus::Carry, v <= self.a);

        let result = self.a.wrapping_sub(v);
        let signed_overflow = !(self.a ^ v) & (self.a ^ result);

        self.status.set(ProcessorStatus::Overflow, false);
        self.status.set(ProcessorStatus::Zero, result == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(signed_overflow,7));

        self.a = result;
    }

    fn alu_and(&mut self, v:u8)
    {
        self.a &= v;

        self.alu_status();
    }

    fn alu_xor(&mut self, v:u8)
    {
        self.a ^= v;

        self.alu_status();
    }

    fn alu_or(&mut self, v:u8)
    {
        self.a |= v;

        self.alu_status();
    }

    fn alu_lsr(&mut self)
    {
        self.status.set(ProcessorStatus::Overflow, false);
        self.status.set(ProcessorStatus::Carry, test_bit(self.a, 0));

        self.a >>= 1;
        
        self.status.set(ProcessorStatus::Zero, self.a == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
    }

    // fn alu_asr(&mut self)
    // {
    //     self.status.set(ProcessorStatus::Overflow, false);
    //     self.status.set(ProcessorStatus::Carry, test_bit(self.a, 0));

    //     self.a = (self.a >> 1) | (self.a & 0x80);   // preserve the high-bit negative sign
        
    //     self.status.set(ProcessorStatus::Zero, self.a == 0);
    //     self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
    // }

    fn alu_asl(&mut self)
    {
        self.status.set(ProcessorStatus::Overflow, false);
        self.status.set(ProcessorStatus::Carry, test_bit(self.a, 7));

        self.a <<= 1;
        
        self.status.set(ProcessorStatus::Zero, self.a == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
    }

    fn alu_rol(&mut self)
    {
        let carry = self.status.contains(ProcessorStatus::Carry) ;

        self.status.set(ProcessorStatus::Overflow, false);
        self.status.set(ProcessorStatus::Carry, test_bit(self.a, 7));

        self.a <<= 1;
        if carry {
            self.a = self.a.wrapping_add(1);
        }
        
        self.status.set(ProcessorStatus::Zero, self.a == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
    }

    fn alu_ror(&mut self)
    {
        let carry = self.status.contains(ProcessorStatus::Carry) ;

        self.status.set(ProcessorStatus::Overflow, false);
        self.status.set(ProcessorStatus::Carry, test_bit(self.a, 0));

        self.a >>= 1;
        if carry {
            self.a += self.a.wrapping_add(0x80);
        }
        
        self.status.set(ProcessorStatus::Zero, self.a == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
    }

    fn alu_status(&mut self)
    {
        self.status.set(ProcessorStatus::Overflow, false);
        self.status.set(ProcessorStatus::Carry, false);
        self.status.set(ProcessorStatus::Zero, self.a == 0);
        self.status.set(ProcessorStatus::Negative, test_bit(self.a,7));
    }

    fn alu_compare(&mut self, v1: u8, v2: u8)
    {
        self.status.set(ProcessorStatus::Negative, test_bit(v1.wrapping_sub(v2), 7));
        self.status.set(ProcessorStatus::Zero, v1 == v2);
        self.status.set(ProcessorStatus::Carry, v1 >= v2);
    }

    /*************************
     * Instruction Execution
     *************************/

    fn execute_opcode(&mut self, opcode: Operation, addr_mode: AddressingMode)
    {
        match opcode {
            Operation::ADC => self.op_adc(addr_mode),
            Operation::AND => self.op_and(addr_mode),
            Operation::ASL => self.op_asl(addr_mode),
            Operation::BCC => self.op_bcc(addr_mode),
            Operation::BCS => self.op_bcs(addr_mode),
            Operation::BEQ => self.op_beq(addr_mode),
            Operation::BIT => self.op_bit(addr_mode),
            Operation::BMI => self.op_bmi(addr_mode),
            Operation::BNE => self.op_bne(addr_mode),
            Operation::BPL => self.op_bpl(addr_mode),
            Operation::BRK => self.op_brk(addr_mode),
            Operation::BVC => self.op_bvc(addr_mode),
            Operation::BVS => self.op_bvs(addr_mode),
            Operation::CLC => self.op_clc(addr_mode),
            Operation::CLD => self.op_cld(addr_mode),
            Operation::CLI => self.op_cli(addr_mode),
            Operation::CLV => self.op_clv(addr_mode),
            Operation::CMP => self.op_cmp(addr_mode),
            Operation::CPX => self.op_cpx(addr_mode),
            Operation::CPY => self.op_cpy(addr_mode),
            Operation::DEC => self.op_dec(addr_mode),
            Operation::DEX => self.op_dex(addr_mode),
            Operation::DEY => self.op_dey(addr_mode),
            Operation::EOR => self.op_eor(addr_mode),
            Operation::INC => self.op_inc(addr_mode),
            Operation::INX => self.op_inx(addr_mode),
            Operation::INY => self.op_iny(addr_mode),
            Operation::JMP => self.op_jmp(addr_mode),
            Operation::JSR => self.op_jsr(addr_mode),
            Operation::LDA => self.op_lda(addr_mode),
            Operation::LDX => self.op_ldx(addr_mode),
            Operation::LDY => self.op_ldy(addr_mode),
            Operation::LSR => self.op_lsr(addr_mode),
            Operation::NOP => self.op_nop(addr_mode),
            Operation::ORA => self.op_ora(addr_mode),
            Operation::PHA => self.op_pha(addr_mode),
            Operation::PHP => self.op_php(addr_mode),
            Operation::PLA => self.op_pla(addr_mode),
            Operation::PLP => self.op_plp(addr_mode),
            Operation::ROL => self.op_rol(addr_mode),
            Operation::ROR => self.op_ror(addr_mode),
            Operation::RTI => self.op_rti(addr_mode),
            Operation::RTS => self.op_rts(addr_mode),
            Operation::SBC => self.op_sbc(addr_mode),
            Operation::SEC => self.op_sec(addr_mode),
            Operation::SED => self.op_sed(addr_mode),
            Operation::SEI => self.op_sei(addr_mode),
            Operation::STA => self.op_sta(addr_mode),
            Operation::STX => self.op_stx(addr_mode),
            Operation::STY => self.op_sty(addr_mode),
            Operation::TAX => self.op_tax(addr_mode),
            Operation::TAY => self.op_tay(addr_mode),
            Operation::TSX => self.op_tsx(addr_mode),
            Operation::TXA => self.op_txa(addr_mode),
            Operation::TXS => self.op_txs(addr_mode),
            Operation::TYA => self.op_tya(addr_mode),
            // "Extra" opcodes
            Operation::KIL => self.op_kil(addr_mode),
            Operation::ISC => self.op_isc(addr_mode),
            Operation::DCP => self.op_dcp(addr_mode),
            Operation::AXS => self.op_axs(addr_mode),
            Operation::LAS => self.op_las(addr_mode),
            Operation::LAX => self.op_lax(addr_mode),
            Operation::SHA => self.op_sha(addr_mode),
            Operation::SAX => self.op_sax(addr_mode),
            Operation::XAA => self.op_xaa(addr_mode),
            Operation::SHX => self.op_shx(addr_mode),
            Operation::RRA => self.op_rra(addr_mode),
            Operation::TAS => self.op_tas(addr_mode),
            Operation::SHY => self.op_shy(addr_mode),
            Operation::ARR => self.op_arr(addr_mode),
            Operation::SRE => self.op_sre(addr_mode),
            Operation::ALR => self.op_alr(addr_mode),
            Operation::RLA => self.op_rla(addr_mode),
            Operation::ANC => self.op_anc(addr_mode),
            Operation::SLO => self.op_slo(addr_mode),

            //_ => todo!("Not implemented instruction")
        };
    }

    fn op_adc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr).wrapping_add(ternary(self.status.contains(ProcessorStatus::Carry), 1u8, 0u8));
        
        self.alu_add(value);
    }

    fn op_and(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.alu_and(value);
    }

    fn op_asl(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a = value;
        self.alu_asl();

        // now write the value back out
        self.mem_store(addr, self.a);
    }

    fn op_bcc(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();
        
        if self.status.contains(ProcessorStatus::Carry) == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bcs(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();
        
        if self.status.contains(ProcessorStatus::Carry)
        {
            self.pc = jmp_addr;
        }
    }

    fn op_beq(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();
        
        if self.status.contains(ProcessorStatus::Zero)
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bit(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.status.set(ProcessorStatus::Zero, (self.a & value) == 0);
    }

    fn op_bmi(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();

        if self.status.contains(ProcessorStatus::Negative)
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bne(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();
        
        if self.status.contains(ProcessorStatus::Zero) == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bpl(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();

        if self.status.contains(ProcessorStatus::Negative) == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_brk(&mut self, addr_mode: AddressingMode)
    {
        /* push pc hi on stack */
        self.stack_push(self.pc.hi());
        /* push pc lo on stack */
        self.stack_push(self.pc.lo());
        /* push status on stack, */
        let status = self.status.bits | ProcessorStatus::Unused.bits | ProcessorStatus::Break.bits;
            
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

    fn op_bvc(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();
        
        if self.status.contains(ProcessorStatus::Overflow) == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bvs(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_jmp_address();
        
        if self.status.contains(ProcessorStatus::Overflow)
        {
            self.pc = jmp_addr;
        }
    }

    fn op_clc(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Carry, false);
    }

    fn op_cld(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Decimal, false);
    }

    fn op_cli(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Interrupt, false);
    }

    fn op_clv(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Overflow, false);
    }

    fn op_cmp(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.alu_compare(self.a, value);
    }

    fn op_cpx(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.alu_compare(self.x, value);
    }

    fn op_cpy(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.alu_compare(self.y, value);
    }

    fn op_dec(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a = value;
        self.alu_sub(1);

        self.mem_store(addr, self.a);
    }

    fn op_dex(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.x;
        self.alu_sub(1);
        self.x = self.a;        
    }

    fn op_dey(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.y;
        self.alu_sub(1);
        self.y = self.a;        
    }

    fn op_eor(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.alu_xor(value);
    }

    fn op_inc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a = value;
        self.alu_add(1);

        self.mem_store(addr, self.a);
    }

    fn op_inx(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.x;
        self.alu_add(1);
        self.x = self.a;
    }

    fn op_iny(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.x;
        self.alu_add(1);
        self.x = self.a;
    }

    fn op_jmp(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);

        // pray that my addressing code handles the "6502" 0xFF rollover bug
        // properly...
        self.pc = addr;
    }

    fn op_jsr(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.pc = self.pc.wrapping_sub(1);
        self.stack_push(self.pc.hi());
        self.stack_push(self.pc.lo());

        self.pc = addr;
    }

    fn op_lda(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.a = self.mem_fetch(addr);

        self.alu_status();
    }

    fn op_ldx(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.a = self.mem_fetch(addr);
        self.x = self.a;

        self.alu_status();
    }

    fn op_ldy(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.a = self.mem_fetch(addr);
        self.y = self.a;

        self.alu_status();
    }

    fn op_lsr(&mut self, addr_mode: AddressingMode)
    {
        if addr_mode == AddressingMode::Accumulator
        {
            self.alu_lsr();
        }
        else
        {
            let addr = self.fetch_operand_address(addr_mode);
            let value = self.mem_fetch(addr);

            self.a = value;
            self.alu_lsr();

            self.mem_store(addr, self.a);
        }
    }

    fn op_nop(&mut self, addr_mode: AddressingMode)
    {
        // No-Op
        // aka do nothing at all but sit and spin
    }

    fn op_ora(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.alu_or(value);
    }

    fn op_pha(&mut self, addr_mode: AddressingMode)
    {
        self.stack_push(self.a) ;
    }

    fn op_php(&mut self, addr_mode: AddressingMode)
    {
        let value = self.status.bits | ProcessorStatus::Break.bits;

        self.stack_push(value);
    }

    fn op_pla(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.stack_pop();
    }

    fn op_plp(&mut self, addr_mode: AddressingMode)
    {
        self.status.bits = (self.status.bits & 0x30) | (self.stack_pop() & 0xCF);
    }

    fn op_rol(&mut self, addr_mode: AddressingMode)
    {
        if addr_mode == AddressingMode::Accumulator
        {
            self.alu_rol();
        }
        else 
        {
            let addr = self.fetch_operand_address(addr_mode);
            let value = self.mem_fetch(addr);

            self.a = value;
            self.alu_rol();

            self.mem_store(addr, self.a) ;
        }
    }

    fn op_ror(&mut self, addr_mode: AddressingMode)
    {
        if addr_mode == AddressingMode::Accumulator
        {
            self.alu_ror();
        }
        else
        {
            let addr = self.fetch_operand_address(addr_mode);
            let value = self.mem_fetch(addr);

            self.a = value;
            self.alu_ror();

            self.mem_store(addr, self.a) ;
        }
    }

    fn op_rti(&mut self, addr_mode: AddressingMode)
    {
        self.status.bits = (self.status.bits & 0x30) | (self.stack_pop() & 0xCF);
        let lo = self.stack_pop();
        let hi = self.stack_pop();

        self.pc = u16::make(hi, lo);
    }

    fn op_rts(&mut self, addr_mode: AddressingMode)
    {
        let lo = self.stack_pop();
        let hi = self.stack_pop();

        self.pc = u16::make(hi, lo).wrapping_add(1);
    }

    fn op_sbc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr).wrapping_sub(ternary(self.status.contains(ProcessorStatus::Carry), 0u8, 1u8));

        self.alu_sub(value);
    }

    fn op_sec(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Carry, true);
    }

    fn op_sed(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Decimal, true);
    }

    fn op_sei(&mut self, addr_mode: AddressingMode)
    {
        self.status.set(ProcessorStatus::Interrupt, true);
    }

    fn op_sta(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.mem_store(addr, self.a);
    }
    
    fn op_stx(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.mem_store(addr, self.x);
    }

    fn op_sty(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.mem_store(addr, self.y);
    }
    
    fn op_tax(&mut self, addr_mode: AddressingMode)
    {
        self.x = self.a;
    }

    fn op_tay(&mut self, addr_mode: AddressingMode)
    {
        self.y = self.a;
    }

    fn op_tsx(&mut self, addr_mode: AddressingMode)
    {
        self.x = self.sp;

        self.status.set(ProcessorStatus::Negative, test_bit(self.x, 7));
        self.status.set(ProcessorStatus::Zero, self.x == 0);
    }

    fn op_txa(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.x;
        self.alu_status();
    }

    fn op_txs(&mut self, addr_mode: AddressingMode)
    {
        self.sp = self.x;
    }

    fn op_tya(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.y;
        self.alu_status();
    }

    //--TODO: "Extra" opcodes
    fn op_kil(&mut self, addr_mode: AddressingMode)
    {
        // just halt on this one
        self.halted = true;
    }

    fn op_isc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        let result = value.wrapping_add(1) + 
                ternary(self.status.contains(ProcessorStatus::Carry), 0, 1) // inverse of carry...
            ;

        self.alu_sub(result);

        self.mem_store(addr, self.a);
    }

    fn op_dcp(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr).wrapping_sub(1);

        self.alu_compare(self.a, value);
    }

    fn op_axs(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.x & self.a;

        self.mem_store(addr, value);
    }

    fn op_las(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= value;
        self.x = self.a;
        self.sp = self.a;

        self.alu_status();
    }

    fn op_lax(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a = value;
        self.x = value;

        self.alu_status();
    }

    fn op_sha(&mut self, addr_mode: AddressingMode)
    {
        // docs say this is an unstable instruction..
        // (follows logic)
        // You don't say...

        let addr = self.fetch_operand_address(addr_mode);
        let mut value = self.a & self.x;
        value &= addr.hi().wrapping_add(1);

        self.mem_store(addr.wrapping_add(self.y as u16), value);
    }

    fn op_sax(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.x & self.a;

        self.mem_store(addr, value);
    }

    fn op_xaa(&mut self, addr_mode: AddressingMode)
    {
        // no telling if this is right.. docs don't agree and I'm not sure it matters

        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= self.x & value;
        
        self.alu_status();
    }

    fn op_shx(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let result = self.x & (addr.hi().wrapping_add(1));

        self.mem_store(addr.wrapping_add(self.y as u16), result);
    }

    fn op_rra(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let mut value = self.mem_fetch(addr);

        let carry = self.status.contains(ProcessorStatus::Carry) ;

        value >>= 1;
        let new_carry = test_bit(value, 0);
        self.status.set(ProcessorStatus::Carry, new_carry);
        
        if carry {
            value += self.a.wrapping_add(0x80);
        }

        // run the normal add w/ carry operation
        if new_carry {
            value += 1;
        }

        self.alu_add(value);
    }

    fn op_tas(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);

        self.sp = self.a & self.x;
        let result = self.sp & (addr.hi().wrapping_add(1));

        self.mem_store(addr.wrapping_add(self.y as u16), result);
    }

    fn op_shy(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);

        let result = self.y.wrapping_add(addr.hi().wrapping_add(1));

        self.mem_store(addr.wrapping_add(self.x as u16), result);
    }

    fn op_arr(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        let mut result = self.a & value;
        result >>= 1;
        
        if self.status.contains(ProcessorStatus::Carry) {
            result |= 0b10000000;
        }

        self.a = result;
        self.alu_status();
    }

    fn op_sre(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a ^= value >> 1;
        self.alu_status();

        self.status.set(ProcessorStatus::Carry, value & 0x01 != 0);
    }

    fn op_alr(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= value;
        self.alu_lsr();
    }

    fn op_rla(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let mut value = self.mem_fetch(addr);
        let set_carry = test_bit(value,7);

        if self.status.contains(ProcessorStatus::Carry) {
            value = value.wrapping_add( value.wrapping_add(1) );
        }
        else {
            value <<= 1;
        }

        self.a &= value;
        self.alu_status();
        self.status.set(ProcessorStatus::Carry, set_carry);
    }

    fn op_anc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= value;
        let hibit = test_bit(self.a, 7);
        
        self.status.set(ProcessorStatus::Carry, hibit);
        self.status.set(ProcessorStatus::Negative, hibit);
    }

    fn op_slo(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let mut value = self.mem_fetch(addr);
        let set_carry = test_bit(value,7);

        value <<= 1;
        self.a |= value;

        self.alu_status();
        self.status.set(ProcessorStatus::Carry, set_carry);
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn cpu_alu_add() 
    {
        let mut cpu = Cpu6502::new(Box::new(Bus::new()));   
        
        cpu.a = 13;
        cpu.alu_add(211);
        
        assert_eq!(cpu.a, 224);
        assert_eq!(cpu.status.contains(ProcessorStatus::Carry), false);
    }
    
    #[test]
    fn cpu_alu_add_carry() 
    {
        let mut cpu = Cpu6502::new(Box::new(Bus::new()));   
        
        cpu.a = 254;
        cpu.alu_add(6);
        
        assert_eq!(cpu.a, 4);
        assert_eq!(cpu.status.contains(ProcessorStatus::Carry), true);
    }
    
    #[test]
    fn cpu_alu_add_carry_clear() 
    {
        let mut cpu = Cpu6502::new(Box::new(Bus::new()));   
        
        cpu.a = 254;
        cpu.alu_add(6); // 4 + carry
        cpu.alu_add(6); 
        
        assert_eq!(cpu.a, 10);
        assert_eq!(cpu.status.contains(ProcessorStatus::Carry), false);
    }
}