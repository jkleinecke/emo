#![allow(non_upper_case_globals)]

use std::fmt;
use super::{WORD,Word,Byte,BitTest,CpuContext,StatusRegister};

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
pub const OPCODE_TABLE: [(Operation, AddressingMode, CycleCount, CycleCount, fn(&mut CpuContext));256] =
    // TODO Audit each record to see that it was input correctly
    // (Operation, addressing mode, clock cycles, oops clock cycles if page boundary crossed)
    [//  x0                  x1                   x2                   x3                   x4                   x5                   x6                   x7                   x8                   x9                   xA                   xB                   xC                   xD                   xE                   xF
/* 0x */(BRK,imp,7,0,op_brk),(ORA,izx,6,0,op_ora),(KIL,imp,0,0,op_kil),(SLO,izx,8,0,op_slo),(NOP,zp, 3,0,op_nop),(ORA,zp, 3,0,op_ora),(ASL,zp, 5,0,op_asl),(SLO,zp, 5,0,op_slo),(PHP,imp,3,0,op_php),(ORA,imm,2,0,op_ora),(ASL,acc,2,0,op_asl),(ANC,imm,2,0,op_anc),(NOP,abs,4,0,op_nop),(ORA,abs,4,0,op_ora),(ASL,abs,6,0,op_asl),(SLO,abs,6,0,op_slo),
/* 1x */(BPL,rel,2,1,op_bpl),(ORA,izy,5,1,op_ora),(KIL,imp,0,0,op_kil),(SLO,izy,8,0,op_slo),(NOP,zpx,4,0,op_nop),(ORA,zpx,4,0,op_ora),(ASL,zpx,6,0,op_asl),(SLO,zpx,6,0,op_slo),(CLC,imp,2,0,op_clc),(ORA,aby,4,1,op_ora),(NOP,imp,2,0,op_nop),(SLO,aby,7,0,op_slo),(NOP,abx,4,1,op_nop),(ORA,abx,4,1,op_ora),(ASL,abx,7,0,op_asl),(SLO,abx,7,0,op_slo),
/* 2x */(JSR,abs,6,0,op_jsr),(AND,izx,6,0,op_and),(KIL,imp,0,0,op_kil),(RLA,izx,8,0,op_rla),(BIT,zp, 3,0,op_bit),(AND,zp, 3,0,op_and),(ROL,zp, 5,0,op_rol),(RLA,zp, 5,0,op_rla),(PLP,imp,4,0,op_plp),(AND,imm,2,0,op_and),(ROL,acc,2,0,op_rol),(ANC,imm,2,0,op_anc),(BIT,abs,4,0,op_bit),(AND,abs,4,0,op_and),(ROL,abs,6,0,op_rol),(RLA,abs,6,0,op_rla),
/* 3x */(BMI,rel,2,1,op_bmi),(AND,izy,5,1,op_and),(KIL,imp,0,0,op_kil),(RLA,izy,8,0,op_rla),(NOP,zpx,4,0,op_nop),(AND,zpx,4,0,op_and),(ROL,zpx,6,0,op_rol),(RLA,zpx,6,0,op_rla),(SEC,imp,2,0,op_sec),(AND,aby,4,1,op_and),(NOP,imp,2,0,op_nop),(RLA,aby,7,0,op_rla),(NOP,abx,4,1,op_nop),(AND,abx,4,1,op_and),(ROL,abx,7,0,op_rol),(RLA,abx,7,0,op_rla),
/* 4x */(RTI,imp,6,0,op_rti),(EOR,izx,6,0,op_eor),(KIL,imp,0,0,op_kil),(SRE,izx,8,0,op_sre),(NOP,zp, 3,0,op_nop),(EOR,zp, 3,0,op_eor),(LSR,zp, 5,0,op_lsr),(SRE,zp, 5,0,op_sre),(PHA,imp,3,0,op_pha),(EOR,imm,2,0,op_eor),(LSR,imp,2,0,op_lsr),(ALR,imm,2,0,op_alr),(JMP,abs,3,0,op_jmp),(EOR,abs,4,0,op_eor),(LSR,abs,6,0,op_lsr),(SRE,abs,6,0,op_sre),
/* 5x */(BVC,rel,2,1,op_bvc),(EOR,izy,5,1,op_eor),(KIL,imp,0,0,op_kil),(SRE,izy,8,0,op_sre),(NOP,zpx,4,0,op_nop),(EOR,zpx,4,0,op_eor),(LSR,zpx,6,0,op_lsr),(SRE,zpx,6,0,op_sre),(CLI,imp,2,0,op_cli),(EOR,aby,4,1,op_eor),(NOP,imp,2,0,op_nop),(SRE,aby,7,0,op_sre),(NOP,abx,4,1,op_nop),(EOR,abx,4,1,op_eor),(LSR,abx,7,0,op_lsr),(SRE,abx,7,0,op_sre),
/* 6x */(RTS,imp,6,0,op_rts),(ADC,izx,6,0,op_adc),(KIL,imp,0,0,op_kil),(RRA,izx,8,0,op_rra),(NOP,zp, 3,0,op_nop),(ADC,zp, 3,0,op_adc),(ROR,zp, 5,0,op_ror),(RRA,zp, 5,0,op_rra),(PLA,imp,4,0,op_pla),(ADC,imm,2,0,op_adc),(ROR,imp,2,0,op_ror),(ARR,imm,2,0,op_arr),(JMP,ind,5,0,op_jmp),(ADC,abs,4,0,op_adc),(ROR,abs,6,0,op_ror),(RRA,abs,6,0,op_rra),
/* 7x */(BVS,rel,2,1,op_bvs),(ADC,izy,5,1,op_adc),(KIL,imp,0,0,op_kil),(RRA,izy,8,0,op_rra),(NOP,zpx,4,0,op_nop),(ADC,zpx,4,0,op_adc),(ROR,zpx,6,0,op_ror),(RRA,zpx,6,0,op_rra),(SEI,imp,2,0,op_sei),(ADC,aby,4,1,op_adc),(NOP,imp,2,0,op_nop),(RRA,aby,7,0,op_rra),(NOP,abx,4,1,op_nop),(ADC,abx,4,1,op_adc),(ROR,abx,7,0,op_ror),(RRA,abx,7,0,op_rra),
/* 8x */(NOP,imm,2,0,op_nop),(STA,izx,6,0,op_sta),(NOP,imm,2,0,op_nop),(SAX,izx,6,0,op_sax),(STY,zp, 3,0,op_sty),(STA,zp, 3,0,op_sta),(STX,zp, 3,0,op_stx),(SAX,zp, 3,0,op_sax),(DEY,imp,2,0,op_dey),(NOP,imm,2,0,op_nop),(TXA,imp,2,0,op_txa),(XAA,imm,2,1,op_xaa),(STY,abs,4,0,op_sty),(STA,abs,4,0,op_sta),(STX,abs,4,0,op_stx),(SAX,abs,4,0,op_sax),
/* 9x */(BCC,rel,2,1,op_bcc),(STA,izy,6,0,op_sta),(KIL,imp,0,0,op_kil),(SHA,izy,6,0,op_sha),(STY,zpx,4,0,op_sty),(STA,zpx,4,0,op_sta),(STX,zpy,4,0,op_stx),(SAX,zpy,4,0,op_sax),(TYA,imp,2,0,op_tya),(STA,aby,5,0,op_sta),(TXS,imp,2,0,op_txs),(TAS,aby,5,0,op_tas),(SHY,abx,5,0,op_shy),(STA,abx,5,0,op_sta),(SHX,aby,5,0,op_shx),(SHA,aby,5,0,op_sha),
/* Ax */(LDY,imm,2,0,op_ldy),(LDA,izx,6,0,op_lda),(LDX,imm,2,0,op_ldx),(LAX,izx,6,0,op_lax),(LDY,zp, 3,0,op_ldy),(LDA,zp, 3,0,op_lda),(LDX,zp, 3,0,op_ldx),(LAX,zp, 3,0,op_lax),(TAY,imp,2,0,op_tay),(LDA,imm,2,0,op_lda),(TAX,imp,2,0,op_tax),(LAX,imm,2,0,op_lax),(LDY,abs,4,0,op_ldy),(LDA,abs,4,0,op_lda),(LDX,abs,4,0,op_ldx),(LAX,abs,4,0,op_lax),
/* Bx */(BCS,rel,2,1,op_bcs),(LDA,izy,5,1,op_lda),(KIL,imp,0,0,op_kil),(LAX,izy,5,1,op_lax),(LDY,zpx,4,0,op_ldy),(LDA,zpx,4,0,op_lda),(LDX,zpy,4,0,op_ldx),(LAX,zpy,4,0,op_lax),(CLV,imp,2,0,op_clv),(LDA,aby,4,1,op_lda),(TSX,imp,2,0,op_tsx),(LAS,aby,4,1,op_las),(LDY,abx,4,1,op_ldy),(LDA,abx,4,1,op_lda),(LDX,aby,4,1,op_ldx),(LAX,aby,4,1,op_lax),
/* Cx */(CPY,imm,2,0,op_cpy),(CMP,izx,6,0,op_cmp),(NOP,imm,2,0,op_nop),(DCP,izx,8,0,op_dcp),(CPY,zp, 3,0,op_cpy),(CMP,zp, 3,0,op_cmp),(DEC,zp, 5,0,op_dec),(DCP,zp, 5,0,op_dcp),(INY,imp,2,0,op_iny),(CMP,imm,2,0,op_cmp),(DEX,imp,2,0,op_dex),(AXS,imm,2,0,op_axs),(CPY,abs,4,0,op_cpy),(CMP,abs,4,0,op_cmp),(DEC,abs,6,0,op_dec),(DCP,abs,6,0,op_dcp),
/* Dx */(BNE,rel,2,1,op_bne),(CMP,izy,5,1,op_cmp),(KIL,imp,0,0,op_kil),(DCP,izy,8,0,op_dcp),(NOP,zpx,4,0,op_nop),(CMP,zpx,4,0,op_cmp),(DEC,zpx,6,0,op_dec),(DCP,zpx,6,0,op_dcp),(CLD,imp,2,0,op_cld),(CMP,aby,4,1,op_cmp),(NOP,imp,2,0,op_nop),(DCP,aby,7,0,op_dcp),(NOP,abx,4,1,op_nop),(CMP,abx,4,1,op_cmp),(DEC,abx,7,0,op_dec),(DCP,abx,7,0,op_dcp),
/* Ex */(CPX,imm,2,0,op_cpx),(SBC,izx,6,0,op_sbc),(NOP,imm,2,0,op_nop),(ISC,izx,8,0,op_isc),(CPX,zp, 3,0,op_cpx),(SBC,zp, 3,0,op_sbc),(INC,zp, 5,0,op_inc),(ISC,zp, 5,0,op_isc),(INX,imp,2,0,op_inx),(SBC,imm,2,0,op_sbc),(NOP,imp,2,0,op_nop),(SBC,imm,2,0,op_sbc),(CPX,abs,4,0,op_cpx),(SBC,abs,4,0,op_sbc),(INC,abs,6,0,op_inc),(ISC,abs,6,0,op_isc),
/* Fx */(BEQ,rel,2,1,op_beq),(SBC,izy,5,1,op_sbc),(KIL,imp,0,0,op_kil),(ISC,izy,8,0,op_isc),(NOP,zpx,4,0,op_nop),(SBC,zpx,4,0,op_sbc),(INC,zpx,6,0,op_inc),(ISC,zpx,6,0,op_isc),(SED,imp,2,0,op_sed),(SBC,aby,4,1,op_sbc),(NOP,imp,2,0,op_nop),(ISC,aby,7,0,op_isc),(NOP,abx,4,1,op_nop),(SBC,abx,4,1,op_sbc),(INC,abx,7,0,op_inc),(ISC,abx,7,0,op_isc),
    ];

/**************************
 * Memory Addressing Modes
 **************************/

fn fetch_operand_address(cpu:&mut CpuContext) -> Word
{
    //--TODO: Handle oops cycle when we cross a page..

    // We could utilize a common ALU method for adding that would
    // have a side-effect of changing the status flags automatically
    // for us... 
    //
    // then the oops cycle could be triggered by the overflow flag..

    match cpu.instruction.addr_mode {
        AddressingMode::Immediate => {
            let addr = cpu.regs.pc.wrapping_add(1);
            
            addr
        }
        AddressingMode::ZeroPage => {
            let lo = cpu.readfrom_pc(1);
            lo as Word
        }
        AddressingMode::ZeroPageX => {
            let lo = cpu.readfrom_pc(1);
            
            lo.wrapping_add(cpu.regs.x) as Word
        }
        AddressingMode::ZeroPageY => {
            let lo = cpu.readfrom_pc(1);
            
            lo.wrapping_add(cpu.regs.y) as Word
        }
        AddressingMode::Absolute => {
            let lo = cpu.readfrom_pc(1);
            let hi = cpu.readfrom_pc(2);     // just making sure the registers are used properly here
            
            Word::make(hi, lo)
        }
        AddressingMode::AbsoluteX => {
            let ptr = cpu.readfrom_pc(1);
            
            let base = ptr as Word + cpu.regs.x as Word;
            let carry = base.hi() > 0;
            
            let mut hi = cpu.readfrom_pc(2);
            
            if carry
            {
                // this is the oops cycle work here.. the lo-byte
                // addition overflowed which means we need to spend
                // an extra cycle adding the carry to the hi-byte
                hi = hi.wrapping_add(1);
                cpu.oops = true;
            }

            Word::make(hi, base.lo())
        }
        AddressingMode::AbsoluteY => {
            let ptr = cpu.readfrom_pc(1);

            let base = ptr as Word + cpu.regs.y as Word;
            let carry = base.hi() > 0;
            
            let mut hi = cpu.readfrom_pc(2);

            if carry
            {
                // this is the oops cycle work here.. the lo-byte
                // addition overflowed which means we need to spend
                // an extra cycle adding the carry to the hi-byte
                hi = hi.wrapping_add(1);
                cpu.oops = true;
            }

            Word::make(hi, base.lo())
        }
        AddressingMode::Indirect => {
            // Only used by the JMP instruction on the 6502
            let base_lo = cpu.readfrom_pc(1);
            let base_hi = cpu.readfrom_pc(2);
            
            let lo = cpu.memory.read(Word::make(base_hi,base_lo));
            
            // There is a bug in the 6502 that causes this addressing mode to work improperly in some cases.
            // If the jump operation is accessing the last byte of a page (ie, $xxFF), then the high byte will
            // be accessed at $00 of that page, instead of $00 of the next page.
            // eg, JMP ($21FF) will grab the low byte from $21FF and the high byte from $2100 instead of $2200 as
            // would be expected. 
            let hi = cpu.memory.read(Word::make(base_hi,base_lo.wrapping_add(1)));  
            Word::make(hi,lo)
        },
        AddressingMode::IndirectX => {
            let base = cpu.readfrom_pc(1);
            let addr = base.wrapping_add(cpu.regs.x);

            let lo = cpu.memory.read(addr as Word);
            let hi = cpu.memory.read(addr as Word + 1);
            Word::make(hi, lo)
        }
        AddressingMode::IndirectY => {
            let ptr = cpu.readfrom_pc(1);

            let base = cpu.memory.read(ptr as Word) as Word + cpu.regs.y as Word;
            let mut hi = cpu.memory.read(ptr as Word + 1);

            let carry = base.hi() > 0;

            if carry
            {
                // this is the oops cycle work here.. the lo-byte
                // addition overflowed which means we need to spend
                // an extra cycle adding the carry to the hi-byte
                hi = hi.wrapping_add(1); // add the carry
                cpu.oops = true;
            }

            Word::make(hi, base.lo())
        },
        AddressingMode::Relative => {
            let offset = cpu.readfrom_pc(1);

            // account for the byte we just read since the inc doesn't happen until
            // after the instruction is done executing
            let pc = cpu.regs.pc + 1;   
            
            let addr = match offset.bit(7) {      // offset can be negative
                true => pc - !offset as Word, // if negative, subtract the inverse (two's complement)  
                false => pc + offset as Word + 1, // otherwise just add the offset
            };

            addr
        }
        _ => panic!("address mode {:?} is not supported", cpu.instruction.addr_mode),
    }
}


pub fn op_adc(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);
    
    cpu.regs.ac = cpu.alu_add(cpu.regs.ac, value);
    
    cpu.update_status(cpu.regs.ac);
}

pub fn op_and(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac &= value;

    cpu.update_status(cpu.regs.ac);
}

pub fn op_asl(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    let result = cpu.alu_asl(value);
    cpu.update_status(value);

    // now write the value back out
    cpu.memory.write(addr, result);
}

pub fn op_bcc(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);
    
    if cpu.regs.p.carry == false
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_bcs(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);
    
    if cpu.regs.p.carry
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_beq(cpu: &mut CpuContext)
{
    let jmp_addr =fetch_operand_address(cpu);
    
    if cpu.regs.p.zero
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_bit(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.p.zero = (cpu.regs.ac & value) == 0;
}

pub fn op_bmi(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);

    if cpu.regs.p.negative
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_bne(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);
    
    if cpu.regs.p.zero == false
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_bpl(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);

    if cpu.regs.p.negative == false
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_brk(cpu: &mut CpuContext)
{
    /* push pc hi on stack */
    cpu.stack_push(cpu.regs.pc.hi());
    /* push pc lo on stack */
    cpu.stack_push(cpu.regs.pc.lo());
    /* push status on stack, */
    let mut status = cpu.regs.p;
    
    if cpu.reset || cpu.nmi || cpu.irq {
        // disable interrupts now
        cpu.regs.p.interrupt = true;       
    }
    else {
        status.r#break = true;
    }
        
    /*** At this point, the signal status determines which interrupt vector is used ***/
    cpu.stack_push(status.to_byte());

    /* reset, fetch pc lo */
    if cpu.reset {
        cpu.jump_pc(Word::make(cpu.memory.read(0xFFFD),cpu.memory.read(0xFFFC)));
    }
    else if cpu.nmi {
        cpu.jump_pc(Word::make(cpu.memory.read(0xFFFB),cpu.memory.read(0xFFFA)));
    }
    else {
        cpu.jump_pc(Word::make(cpu.memory.read(0xFFFF),cpu.memory.read(0xFFFE)));
    }

    cpu.increment_programcounter = false;
}

pub fn op_bvc(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);
    
    if cpu.regs.p.overflow == false
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_bvs(cpu: &mut CpuContext)
{
    let jmp_addr = fetch_operand_address(cpu);
    
    if cpu.regs.p.overflow
    {
        cpu.jump_pc(jmp_addr);
    }
}

pub fn op_clc(cpu: &mut CpuContext)
{
    cpu.regs.p.carry = false;
}

pub fn op_cld(cpu: &mut CpuContext)
{
    cpu.regs.p.decimal = false;
}

pub fn op_cli(cpu: &mut CpuContext)
{
    cpu.regs.p.interrupt = false;
}

pub fn op_clv(cpu: &mut CpuContext)
{
    cpu.regs.p.overflow = false;
}

pub fn op_cmp(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.alu_compare(cpu.regs.ac, value);
}

pub fn op_cpx(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.alu_compare(cpu.regs.x, value);
}

pub fn op_cpy(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.alu_compare(cpu.regs.y, value);
}

pub fn op_dec(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    let result = value.wrapping_sub(1);
    cpu.update_status(result);

    cpu.memory.write(addr, result);
}

pub fn op_dex(cpu: &mut CpuContext)
{
    cpu.regs.x = cpu.regs.x.wrapping_sub(1);
    cpu.update_status(cpu.regs.x);
}

pub fn op_dey(cpu: &mut CpuContext)
{
    cpu.regs.y = cpu.regs.y.wrapping_sub(1);
    cpu.update_status(cpu.regs.y);
}

pub fn op_eor(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac ^= value;
    cpu.update_status(cpu.regs.ac);
}

pub fn op_inc(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    let result = value.wrapping_add(1);
    cpu.update_status(result);

    cpu.memory.write(addr, result);
}

pub fn op_inx(cpu: &mut CpuContext)
{
    cpu.regs.x = cpu.regs.x.wrapping_add(1);
    cpu.update_status(cpu.regs.x);
}

pub fn op_iny(cpu: &mut CpuContext)
{
    cpu.regs.y = cpu.regs.y.wrapping_add(1);
    cpu.update_status(cpu.regs.y);
}

pub fn op_jmp(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);

    // pray that my addressing code handles the "6502" 0xFF rollover bug
    // properly...
    cpu.jump_pc(addr);
}

pub fn op_jsr(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let pc = cpu.regs.pc.wrapping_add(cpu.instruction.opsize as u16 - 1);
    cpu.stack_push(pc.hi());
    cpu.stack_push(pc.lo());

    cpu.jump_pc(addr);
}

pub fn op_lda(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    cpu.regs.ac = cpu.memory.read(addr);

    cpu.update_status(cpu.regs.ac);
}

pub fn op_ldx(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    cpu.regs.x = cpu.memory.read(addr);
    
    cpu.update_status(cpu.regs.x);
}

pub fn op_ldy(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    cpu.regs.y = cpu.memory.read(addr);
    
    cpu.update_status(cpu.regs.y);
}

pub fn op_lsr(cpu: &mut CpuContext)
{
    match cpu.instruction.addr_mode {
        AddressingMode::Accumulator | AddressingMode::Implicit => {
            cpu.regs.ac = cpu.alu_lsr(cpu.regs.ac);
            cpu.update_status(cpu.regs.ac);
        },
        _ => {
            let addr = fetch_operand_address(cpu);
            let value = cpu.memory.read(addr);

            let result = cpu.alu_lsr(value);
            cpu.update_status(result);

            cpu.memory.write(addr, result);
        }
    }
}

pub fn op_nop(cpu: &mut CpuContext)
{
    // No-Op
    // aka do nothing at all but sit and spin
}

pub fn op_ora(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac |= value;
    cpu.update_status(cpu.regs.ac);
}

pub fn op_pha(cpu: &mut CpuContext)
{
    cpu.stack_push(cpu.regs.ac) ;
}

pub fn op_php(cpu: &mut CpuContext)
{
    let value = cpu.regs.p.to_byte() | 0x10;

    cpu.stack_push(value);
}

pub fn op_pla(cpu: &mut CpuContext)
{
    cpu.regs.ac = cpu.stack_pop();
}

pub fn op_plp(cpu: &mut CpuContext)
{
    cpu.regs.p = StatusRegister::from((cpu.regs.p.to_byte() & 0x30) | (cpu.stack_pop() & 0xCF));
}

pub fn op_rol(cpu: &mut CpuContext)
{
    if cpu.instruction.addr_mode == AddressingMode::Accumulator
    {
        cpu.regs.ac = cpu.alu_rol(cpu.regs.ac);
        cpu.update_status(cpu.regs.ac);
    }
    else 
    {
        let addr = fetch_operand_address(cpu);
        let value = cpu.memory.read(addr);

        let result = cpu.alu_rol(value);
        cpu.update_status(result);

        cpu.memory.write(addr, result) ;
    }
}

pub fn op_ror(cpu: &mut CpuContext)
{
    if cpu.instruction.addr_mode == AddressingMode::Accumulator
    {
        cpu.regs.ac = cpu.alu_ror(cpu.regs.ac);
        cpu.update_status(cpu.regs.ac);
    }
    else
    {
        let addr = fetch_operand_address(cpu);
        let value = cpu.memory.read(addr);

        let result = cpu.alu_ror(value);
        cpu.update_status(result);

        cpu.memory.write(addr, result) ;
    }
}

pub fn op_rti(cpu: &mut CpuContext)
{
    cpu.regs.p = StatusRegister::from((cpu.regs.p.to_byte() & 0x30) | (cpu.stack_pop() & 0xCF));
    let lo = cpu.stack_pop();
    let hi = cpu.stack_pop();

    cpu.jump_pc(Word::make(hi, lo));
}

pub fn op_rts(cpu: &mut CpuContext)
{
    let lo = cpu.stack_pop();
    let hi = cpu.stack_pop();

    cpu.jump_pc(Word::make(hi, lo).wrapping_add(1));
}

pub fn op_sbc(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr).wrapping_sub(ternary!(cpu.regs.p.carry, 0u8, 1u8));

    cpu.regs.ac = cpu.alu_sub(cpu.regs.ac, value);
    
    cpu.update_status(cpu.regs.ac);
}

pub fn op_sec(cpu: &mut CpuContext)
{
    cpu.regs.p.carry = true;
}

pub fn op_sed(cpu: &mut CpuContext)
{
    cpu.regs.p.decimal = true;
}

pub fn op_sei(cpu: &mut CpuContext)
{
    cpu.regs.p.interrupt = true;
}

pub fn op_sta(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    cpu.memory.write(addr, cpu.regs.ac);
}

pub fn op_stx(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    cpu.memory.write(addr, cpu.regs.x);
}

pub fn op_sty(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    cpu.memory.write(addr, cpu.regs.y);
}

pub fn op_tax(cpu: &mut CpuContext)
{
    cpu.regs.x = cpu.regs.ac;
}

pub fn op_tay(cpu: &mut CpuContext)
{
    cpu.regs.y = cpu.regs.ac;
}

pub fn op_tsx(cpu: &mut CpuContext)
{
    cpu.regs.x = cpu.regs.sp;

    cpu.update_status(cpu.regs.x);
}

pub fn op_txa(cpu: &mut CpuContext)
{
    cpu.regs.ac = cpu.regs.x;
    cpu.update_status(cpu.regs.ac);
}

pub fn op_txs(cpu: &mut CpuContext)
{
    cpu.regs.sp = cpu.regs.x;
}

pub fn op_tya(cpu: &mut CpuContext)
{
    cpu.regs.ac = cpu.regs.y;
    cpu.update_status(cpu.regs.ac);
}

//--TODO: "Extra" opcodes
pub fn op_kil(cpu: &mut CpuContext)
{
    // just halt on this one
    cpu.halt = true;
}

pub fn op_isc(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    let result = value.wrapping_add(1);
    cpu.regs.p.carry = !cpu.regs.p.carry;  // flip the carry bit for !c logic

    cpu.regs.ac = cpu.alu_sub(cpu.regs.ac, result);
    cpu.update_status(cpu.regs.ac);

    cpu.memory.write(addr, cpu.regs.ac);
}

pub fn op_dcp(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr).wrapping_sub(1);

    cpu.alu_compare(cpu.regs.ac, value);
}

pub fn op_axs(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.regs.x & cpu.regs.ac;

    cpu.memory.write(addr, value);
}

pub fn op_las(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac &= value;
    cpu.regs.x = cpu.regs.ac;
    cpu.regs.sp = cpu.regs.ac;

    cpu.update_status(cpu.regs.ac);
}

pub fn op_lax(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac = value;
    cpu.regs.x = value;

    cpu.update_status(cpu.regs.ac);
}

pub fn op_sha(cpu: &mut CpuContext)
{
    // docs say this is an unstable instruction..
    // (follows logic)
    // You don't say...

    let addr = fetch_operand_address(cpu);
    let mut value = cpu.regs.ac & cpu.regs.x;
    value &= addr.hi().wrapping_add(1);

    cpu.memory.write(addr.wrapping_add(cpu.regs.y as Word), value);
}

pub fn op_sax(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.regs.x & cpu.regs.ac;

    cpu.memory.write(addr, value);
}

pub fn op_xaa(cpu: &mut CpuContext)
{
    // no telling if this is right.. docs don't agree and I'm not sure it matters

    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac &= cpu.regs.x & value;
    
    cpu.update_status(cpu.regs.ac);
}

pub fn op_shx(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let result = cpu.regs.x & (addr.hi().wrapping_add(1));

    cpu.memory.write(addr.wrapping_add(cpu.regs.y as Word), result);
}

pub fn op_rra(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    let result = cpu.alu_ror(value);
    cpu.regs.ac = cpu.alu_add(cpu.regs.ac, result);
    cpu.update_status(cpu.regs.ac);
}

pub fn op_tas(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);

    cpu.regs.sp = cpu.regs.ac & cpu.regs.x;
    let result = cpu.regs.sp & (addr.hi().wrapping_add(1));

    cpu.memory.write(addr.wrapping_add(cpu.regs.y as Word), result);
}

pub fn op_shy(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);

    let result = cpu.regs.y.wrapping_add(addr.hi().wrapping_add(1));

    cpu.memory.write(addr.wrapping_add(cpu.regs.x as Word), result);
}

pub fn op_arr(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac &= value;
    cpu.regs.ac = cpu.alu_ror(cpu.regs.ac);
    cpu.update_status(cpu.regs.ac);
}

pub fn op_sre(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac ^= cpu.alu_lsr(value);
    cpu.update_status(cpu.regs.ac);
}

pub fn op_alr(cpu: &mut CpuContext)
{
    //--TODO: Optimize this to utilize the immediate value
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac &= value;
    cpu.regs.ac = cpu.alu_lsr(cpu.regs.ac);

    cpu.update_status(cpu.regs.ac);
}

pub fn op_rla(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);
    
    cpu.regs.ac &= cpu.alu_rol(value);
    cpu.update_status(cpu.regs.ac);
}

pub fn op_anc(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac &= value;
    let hibit = cpu.regs.ac.bit(7);
    
    cpu.regs.p.carry = hibit;
    cpu.regs.p.negative = hibit;
}

pub fn op_slo(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    cpu.regs.ac |= cpu.alu_asl(value);
    cpu.update_status(cpu.regs.ac);
}