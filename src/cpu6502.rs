#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::common::{BitTest, WORD};
use crate::mapper::Mapper;
use crate::system::Clocked;
use crate::operations::*;
use std::cell::RefCell;
use std::rc::Rc;
use std::fmt;

macro_rules! ternary {
    ($condition: expr, $_true: expr, $_false: expr) => {
        if $condition { $_true } else { $_false }
    };
}

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

const BIT_C: u8   = 0;
const BIT_Z: u8   = 1;
const BIT_I: u8   = 2;
const BIT_D: u8   = 3;         // Not really used

const BIT_B: u8   = 4;
const BIT_U: u8   = 5;
const BIT_V: u8   = 6;
const BIT_N: u8   = 7;

#[derive(Default)]
pub struct Status {
    pub flags: u8,
}

impl fmt::Debug for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let n = ternary!(self.flags.on(BIT_N), 'N', 'n');
        let v = ternary!(self.flags.on(BIT_V), 'V', 'v');
        let b = ternary!(self.flags.on(BIT_B), 'B', 'b');
        let d = ternary!(self.flags.on(BIT_D), 'D', 'd');
        let i = ternary!(self.flags.on(BIT_I), 'I', 'i');
        let z = ternary!(self.flags.on(BIT_Z), 'Z', 'z');
        let c = ternary!(self.flags.on(BIT_C), 'C', 'c');

        write!(f, "{}{}-{}{}{}{}{}", n,v,b,d,i,z,c)
    }
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let n = ternary!(self.flags.on(BIT_N), 'N', 'n');
        let v = ternary!(self.flags.on(BIT_V), 'V', 'v');
        let b = ternary!(self.flags.on(BIT_B), 'B', 'b');
        let d = ternary!(self.flags.on(BIT_D), 'D', 'd');
        let i = ternary!(self.flags.on(BIT_I), 'I', 'i');
        let z = ternary!(self.flags.on(BIT_Z), 'Z', 'z');
        let c = ternary!(self.flags.on(BIT_C), 'C', 'c');

        write!(f, "{}{}-{}{}{}{}{}", n,v,b,d,i,z,c)
    }
}

impl Status {
    pub fn initial() -> Self {
        Status { flags: 0x34 }
    }

    pub fn carry(&self) -> bool { self.flags.on(BIT_C) }
    pub fn zero(&self) -> bool { self.flags.on(BIT_Z) }
    pub fn interrupt(&self) -> bool { self.flags.on(BIT_I) }
    pub fn decimal(&self) -> bool { self.flags.on(BIT_D) }
    pub fn r#break(&self) -> bool { self.flags.on(BIT_B) }
    pub fn overflow(&self) -> bool { self.flags.on(BIT_V) }
    pub fn negative(&self) -> bool { self.flags.on(BIT_N) }

    pub fn set_carry(&mut self, v:bool) { self.flags.set(BIT_C,v); }
    pub fn set_zero(&mut self, v:bool) { self.flags.set(BIT_Z,v); }
    pub fn set_interrupt(&mut self, v:bool) { self.flags.set(BIT_I,v); }
    pub fn set_decimal(&mut self, v:bool) { self.flags.set(BIT_D,v); }
    pub fn set_break(&mut self, v:bool) { self.flags.set(BIT_B,v); }
    pub fn set_overflow(&mut self, v:bool) { self.flags.set(BIT_V,v); }
    pub fn set_negative(&mut self, v:bool) { self.flags.set(BIT_N,v); }
}

pub struct Cpu6502 {
    pub a: u8,                      // Accumulator Register
    pub x: u8,                      // X Register
    pub y: u8,                      // Y Register
    pub sp: u8,                     // Stack Pointer
    pub pc: u16,                        // Internal Program Counter Register
    
    pub status: Status,    // Status Register

    // internal registers
    pub ir_cycles: u8,
    pub ir: u8,                         // active instruction register
    
    halted: bool,                   // fake register to show that we should be done

    // bus module
    pub mapper: Rc<RefCell<dyn Mapper>>,

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
    pub fn new(mapper: Rc<RefCell<dyn Mapper>>) -> Self {
        Cpu6502 {
            a: 0,
            x: 0,
            y: 0,
            sp: STACK_START,
            pc: PC_START,

            status: Status::initial(), 
            
            ir_cycles: 0,
            ir:0,

            halted: false,

            mapper,
            
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
        let status = self.status.flags | BIT_U;
        
        //--TODO!
        /*** At this point, the signal status determines which interrupt vector is used ***/
        self.stack_push(status);
        
        self.status.set_interrupt(false) ;
        
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
        self.mapper.borrow_mut().read(addr)
    }

    fn mem_store(&mut self, addr: u16, v: u8) {
        self.mapper.borrow_mut().write(addr, v)
    }

    fn mem_fetch16(&mut self, addr: u16) -> u16 {
        let mut bus = self.mapper.borrow_mut();
        let lo = bus.read(addr);
        let hi = bus.read(addr + 1);
        u16::make(hi, lo)
    }

    fn mem_store16(&mut self, addr: u16, v: u16) {
        let mut bus = self.mapper.borrow_mut();
        bus.write(addr, v.lo());
        bus.write(addr+1, v.hi());
    }

    fn stack_push(&mut self, v:u8) 
    {
        let addr = STACK_BASE + self.sp as u16;    // calc the stack pointer address
        self.mapper.borrow_mut().write(addr, v) ;               // queue up the bus write
        self.sp -= 1;                       // decrement the stack pointer
    }

    fn stack_pop(&mut self) -> u8
    {
        self.sp += 1;                       // increment the stack pointer
        let addr = STACK_BASE + self.sp as u16;    // calc the stack pointer address
        self.mapper.borrow_mut().read(addr)                   // bus read
    }

    // Status Flags

    fn update_status(&mut self, result:u8)
    {
        self.status.set_zero(result == 0);
        self.status.set_negative(result.on(7));
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
                let lo = self.mem_fetch(self.pc);
                self.inc_pc();
                lo as u16
            }
            AddressingMode::ZeroPageX => {
                let lo = self.mem_fetch(self.pc);
                self.inc_pc();
                lo.wrapping_add(self.x) as u16
            }
            AddressingMode::ZeroPageY => {
                let lo = self.mem_fetch(self.pc);
                self.inc_pc();
                lo.wrapping_add(self.y) as u16
            }
            AddressingMode::Absolute => {
                let lo = self.mem_fetch(self.pc);
                self.inc_pc();
                let hi = self.mem_fetch(self.pc);     // just making sure the registers are used properly here
                self.inc_pc();
                u16::make(hi, lo)
            }
            AddressingMode::AbsoluteX => {
                let ptr = self.mem_fetch(self.pc);
                self.inc_pc();
                let base = ptr as u16 + self.x as u16;
                let carry = base.hi() > 0;
                
                let mut hi = self.mem_fetch(self.pc);
                self.inc_pc();

                if carry
                {
                    // this is the oops cycle work here.. the lo-byte
                    // addition overflowed which means we need to spend
                    // an extra cycle adding the carry to the hi-byte
                    hi = hi.wrapping_add(1);
                    self.ir_cycles += 1;    // add the oops cycle into it
                }

                u16::make(hi, base.lo())
            }
            AddressingMode::AbsoluteY => {
                let ptr = self.mem_fetch(self.pc);
                self.inc_pc();
                let base = ptr as u16 + self.y as u16;
                let carry = base.hi() > 0;
                
                let mut hi = self.mem_fetch(self.pc);
                self.inc_pc();

                if carry
                {
                    // this is the oops cycle work here.. the lo-byte
                    // addition overflowed which means we need to spend
                    // an extra cycle adding the carry to the hi-byte
                    hi = hi.wrapping_add(1);
                    self.ir_cycles += 1;    // add the oops cycle into it
                }

                u16::make(hi, base.lo())
            }
             AddressingMode::Indirect => {
                // Only used by the JMP instruction on the 6502
                let base_lo = self.mem_fetch(self.pc);
                self.inc_pc();
                let base_hi = self.mem_fetch(self.pc);
                self.inc_pc();

                let lo = self.mem_fetch(u16::make(base_hi,base_lo));
                
                // There is a bug in the 6502 that causes this addressing mode to work improperly in some cases.
                // If the jump operation is accessing the last byte of a page (ie, $xxFF), then the high byte will
                // be accessed at $00 of that page, instead of $00 of the next page.
                // eg, JMP ($21FF) will grab the low byte from $21FF and the high byte from $2100 instead of $2200 as
                // would be expected. 
                let hi = self.mem_fetch(u16::make(base_hi,base_lo.wrapping_add(1)));  
                u16::make(hi,lo)
            },
            AddressingMode::IndirectX => {
                let base = self.mem_fetch(self.pc);
                self.inc_pc();
                let addr = base.wrapping_add(self.x);

                let lo = self.mem_fetch(addr as u16);
                let hi = self.mem_fetch(addr as u16 + 1);
                u16::make(hi, lo)
            }
            AddressingMode::IndirectY => {
                let ptr = self.mem_fetch(self.pc);
                self.inc_pc();

                let base = self.mem_fetch(ptr as u16) as u16 + self.y as u16;
                let mut hi = self.mem_fetch(ptr as u16 + 1);

                let carry = base.hi() > 0;

                if carry
                {
                    // this is the oops cycle work here.. the lo-byte
                    // addition overflowed which means we need to spend
                    // an extra cycle adding the carry to the hi-byte
                    hi = hi.wrapping_add(1); // add the carry
                    self.ir_cycles += 1;     // add the oops cycle into it
                }

                u16::make(hi, base.lo())
            },
            AddressingMode::Relative => {
                let offset = self.mem_fetch(self.pc);
                
                let addr = self.pc + offset as u16;

                self.inc_pc();
                addr
            }
            _ => panic!("address mode {:?} is not supported", mode),
        }
    }

    /*************************
     * ALU operations
     *************************/

    fn alu_add(&mut self, a:u8, b:u8) -> u8
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

        let result = a as u16 + b as u16 + self.status.carry() as u16;
        let signed_overflow = !(a ^ b) & (a ^ result.lo());

        self.status.set_carry(result.hi() > 0);
        self.status.set_overflow(signed_overflow.bit(7));

        result.lo()
    }

    fn alu_sub(&mut self, a:u8, b:u8) -> u8
    {
        // further ambitious bullshit from me...
        // two's complement based subtraction is 
        // the same as a + !b, so just use the add routine

        self.alu_add(a, !b)
    }

    fn alu_lsr(&mut self, a:u8) -> u8
    {
        self.status.set_carry(a.on(0));

        a >> 1
    }

    fn alu_asl(&mut self, a:u8) -> u8
    {
        self.status.set_carry(a.on(7));

        a << 1
    }

    fn alu_rol(&mut self, a:u8) -> u8
    {
        let carry = self.status.carry() ;
        self.status.set_carry(a.on(7));

        (a << 1).wrapping_add(ternary!(carry,1,0))
    }

    fn alu_ror(&mut self, a:u8) -> u8
    {
        let carry = self.status.carry() ;

        self.status.set_carry(a.on(0));

        (a >> 1).wrapping_add(ternary!(carry,0x80,0))
    }

    fn alu_compare(&mut self, v1: u8, v2: u8)
    {
        self.status.set_negative(v1.wrapping_sub(v2).on(7));
        self.status.set_zero(v1 == v2);
        self.status.set_carry(v1 >= v2);
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
        let value = self.mem_fetch(addr);
        
        self.a = self.alu_add(self.a, value);
        self.a = self.alu_add(self.a, 0);       // invoke 2nd time with 0 to catch the carry bit

        self.update_status(self.a);
    }

    fn op_and(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= value;

        self.update_status(self.a);
    }

    fn op_asl(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        let result = self.alu_asl(value);
        self.update_status(value);

        // now write the value back out
        self.mem_store(addr, result);
    }

    fn op_bcc(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_operand_address(addr_mode);
        
        if self.status.carry() == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bcs(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_operand_address(addr_mode);
        
        if self.status.carry()
        {
            self.pc = jmp_addr;
        }
    }

    fn op_beq(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr =self.fetch_operand_address(addr_mode);
        
        if self.status.zero()
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bit(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.status.set_zero((self.a & value) == 0);
    }

    fn op_bmi(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_operand_address(addr_mode);

        if self.status.negative()
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bne(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_operand_address(addr_mode);
        
        if self.status.zero() == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bpl(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_operand_address(addr_mode);

        if self.status.negative() == false
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
        let status = self.status.flags | BIT_U | BIT_B;
            
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
        let jmp_addr = self.fetch_operand_address(addr_mode);
        
        if self.status.overflow() == false
        {
            self.pc = jmp_addr;
        }
    }

    fn op_bvs(&mut self, addr_mode: AddressingMode)
    {
        let jmp_addr = self.fetch_operand_address(addr_mode);
        
        if self.status.overflow()
        {
            self.pc = jmp_addr;
        }
    }

    fn op_clc(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_carry(false);
    }

    fn op_cld(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_decimal(false);
    }

    fn op_cli(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_interrupt(false);
    }

    fn op_clv(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_overflow(false);
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

        let result = value.wrapping_sub(1);
        self.update_status(result);

        self.mem_store(addr, result);
    }

    fn op_dex(&mut self, addr_mode: AddressingMode)
    {
        self.x = self.x.wrapping_sub(1);
        self.update_status(self.x);
    }

    fn op_dey(&mut self, addr_mode: AddressingMode)
    {
        self.y = self.y.wrapping_sub(1);
        self.update_status(self.y);
    }

    fn op_eor(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a ^= value;
        self.update_status(self.a);
    }

    fn op_inc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        let result = value.wrapping_add(1);
        self.update_status(result);

        self.mem_store(addr, result);
    }

    fn op_inx(&mut self, addr_mode: AddressingMode)
    {
        self.x = self.x.wrapping_add(1);
        self.update_status(self.x);
    }

    fn op_iny(&mut self, addr_mode: AddressingMode)
    {
        self.y = self.y.wrapping_add(1);
        self.update_status(self.y);
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

        self.update_status(self.a);
    }

    fn op_ldx(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.x = self.mem_fetch(addr);
        
        self.update_status(self.x);
    }

    fn op_ldy(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        self.y = self.mem_fetch(addr);
        
        self.update_status(self.y);
    }

    fn op_lsr(&mut self, addr_mode: AddressingMode)
    {
        if addr_mode == AddressingMode::Accumulator
        {
            self.a = self.alu_lsr(self.a);
            self.update_status(self.a);
        }
        else
        {
            let addr = self.fetch_operand_address(addr_mode);
            let value = self.mem_fetch(addr);

            let result = self.alu_lsr(value);
            self.update_status(result);

            self.mem_store(addr, result);
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

        self.a |= value;
        self.update_status(self.a);
    }

    fn op_pha(&mut self, addr_mode: AddressingMode)
    {
        self.stack_push(self.a) ;
    }

    fn op_php(&mut self, addr_mode: AddressingMode)
    {
        let value = self.status.flags | BIT_B;

        self.stack_push(value);
    }

    fn op_pla(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.stack_pop();
    }

    fn op_plp(&mut self, addr_mode: AddressingMode)
    {
        self.status.flags = (self.status.flags & 0x30) | (self.stack_pop() & 0xCF);
    }

    fn op_rol(&mut self, addr_mode: AddressingMode)
    {
        if addr_mode == AddressingMode::Accumulator
        {
            self.a = self.alu_rol(self.a);
            self.update_status(self.a);
        }
        else 
        {
            let addr = self.fetch_operand_address(addr_mode);
            let value = self.mem_fetch(addr);

            let result = self.alu_rol(value);
            self.update_status(result);

            self.mem_store(addr, result) ;
        }
    }

    fn op_ror(&mut self, addr_mode: AddressingMode)
    {
        if addr_mode == AddressingMode::Accumulator
        {
            self.a = self.alu_ror(self.a);
            self.update_status(self.a);
        }
        else
        {
            let addr = self.fetch_operand_address(addr_mode);
            let value = self.mem_fetch(addr);

            let result = self.alu_ror(value);
            self.update_status(result);

            self.mem_store(addr, result) ;
        }
    }

    fn op_rti(&mut self, addr_mode: AddressingMode)
    {
        self.status.flags = (self.status.flags & 0x30) | (self.stack_pop() & 0xCF);
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
        let value = self.mem_fetch(addr).wrapping_sub(ternary!(self.status.carry(), 0u8, 1u8));

        self.a = self.alu_sub(self.a, value);
        self.a = self.alu_sub(self.a, 0);       // 2nd time will handle the carry 
    }

    fn op_sec(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_carry(true);
    }

    fn op_sed(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_decimal(true);
    }

    fn op_sei(&mut self, addr_mode: AddressingMode)
    {
        self.status.set_interrupt(true);
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

        self.update_status(self.x);
    }

    fn op_txa(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.x;
        self.update_status(self.a);
    }

    fn op_txs(&mut self, addr_mode: AddressingMode)
    {
        self.sp = self.x;
    }

    fn op_tya(&mut self, addr_mode: AddressingMode)
    {
        self.a = self.y;
        self.update_status(self.a);
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

        let result = value.wrapping_add(1);
        self.status.flags.flip(BIT_C);  // flip the carry bit for !c logic

        self.a = self.alu_sub(self.a, result);
        self.update_status(self.a);

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

        self.update_status(self.a);
    }

    fn op_lax(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a = value;
        self.x = value;

        self.update_status(self.a);
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
        
        self.update_status(self.a);
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
        let value = self.mem_fetch(addr);

        let result = self.alu_ror(value);
        self.a = self.alu_add(self.a, result);
        self.update_status(self.a);
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

        self.a &= value;
        self.a = self.alu_ror(self.a);
        self.update_status(self.a);
    }

    fn op_sre(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a ^= self.alu_lsr(value);
        self.update_status(self.a);
    }

    fn op_alr(&mut self, addr_mode: AddressingMode)
    {
        //--TODO: Optimize this to utilize the immediate value
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= value;
        self.a = self.alu_lsr(self.a);

        self.update_status(self.a);
    }

    fn op_rla(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);
        
        self.a &= self.alu_rol(value);
        self.update_status(self.a);
    }

    fn op_anc(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a &= value;
        let hibit = self.a.on(7);
        
        self.status.set_carry(hibit);
        self.status.set_negative(hibit);
    }

    fn op_slo(&mut self, addr_mode: AddressingMode)
    {
        let addr = self.fetch_operand_address(addr_mode);
        let value = self.mem_fetch(addr);

        self.a |= self.alu_asl(value);
        self.update_status(self.a);
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::mapper::Ram;

    fn create_cpu() -> Cpu6502 {
        let ram = Rc::new(RefCell::new(Ram::new()));
        Cpu6502::new(ram)
    }

    #[test]
    fn cpu_alu_add() 
    {
        let mut cpu = create_cpu();   
        
        let result = cpu.alu_add(13, 211);
        
        assert_eq!(result, 224);
        assert_eq!(cpu.status.carry(), false);
    }
    
    #[test]
    fn cpu_alu_add_carry() 
    {
        let mut cpu = create_cpu();   
        
        let result = cpu.alu_add(254, 6);
        
        assert_eq!(result, 4);
        assert_eq!(cpu.status.carry(), true);
    }
    
    #[test]
    fn cpu_alu_add_carry_clear() 
    {
        let mut cpu = create_cpu();   
        
        let temp = cpu.alu_add(254,6); // 4 + carry
        let result = cpu.alu_add(temp, 6); 
        
        assert_eq!(result, 11);
        assert_eq!(cpu.status.carry(), false);
    }
}