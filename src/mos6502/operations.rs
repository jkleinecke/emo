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
    KIL,ISB,DCP,AXS,LAS,LAX,SHA,SAX,XAA,
    SHX,RRA,TAS,SHY,ARR,SRE,ALR,RLA,ANC,
    SLO,LXA,AHX
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
type IllegalOp = Byte;

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
pub const OPCODE_TABLE: [(Operation, AddressingMode, CycleCount, CycleCount, fn(&mut CpuContext), IllegalOp);256] =
    // TODO Audit each record to see that it was input correctly
    // (Operation, addressing mode, clock cycles, oops clock cycles if page boundary crossed)
    [//  x0                    x1                     x2                     x3                     x4                     x5                     x6                     x7                     x8                     x9                     xA                     xB                     xC                     xD                     xE                     xF
/* 0x */(BRK,imp,7,0,op_brk,0),(ORA,izx,6,0,op_ora,0),(NOP,imp,0,0,op_kil,1),(SLO,izx,8,0,op_slo,1),(NOP,zp, 3,0,op_nop,1),(ORA,zp, 3,0,op_ora,0),(ASL,zp, 5,0,op_asl,0),(SLO,zp, 5,0,op_slo,1),(PHP,imp,3,0,op_php,0),(ORA,imm,2,0,op_ora,0),(ASL,acc,2,0,op_asl,0),(ANC,imm,2,0,op_anc,1),(NOP,abs,4,0,op_nop,1),(ORA,abs,4,0,op_ora,0),(ASL,abs,6,0,op_asl,0),(SLO,abs,6,0,op_slo,1),
/* 1x */(BPL,rel,2,1,op_bpl,0),(ORA,izy,5,1,op_ora,0),(NOP,imp,0,0,op_kil,1),(SLO,izy,8,0,op_slo,1),(NOP,zpx,4,0,op_nop,1),(ORA,zpx,4,0,op_ora,0),(ASL,zpx,6,0,op_asl,0),(SLO,zpx,6,0,op_slo,1),(CLC,imp,2,0,op_clc,0),(ORA,aby,4,1,op_ora,0),(NOP,imp,2,0,op_nop,1),(SLO,aby,7,0,op_slo,1),(NOP,abx,4,1,op_nop,1),(ORA,abx,4,1,op_ora,0),(ASL,abx,7,0,op_asl,0),(SLO,abx,7,0,op_slo,1),
/* 2x */(JSR,abs,6,0,op_jsr,0),(AND,izx,6,0,op_and,0),(NOP,imp,0,0,op_kil,1),(RLA,izx,8,0,op_rla,1),(BIT,zp, 3,0,op_bit,0),(AND,zp, 3,0,op_and,0),(ROL,zp, 5,0,op_rol,0),(RLA,zp, 5,0,op_rla,1),(PLP,imp,4,0,op_plp,0),(AND,imm,2,0,op_and,0),(ROL,acc,2,0,op_rol,0),(ANC,imm,2,0,op_anc,1),(BIT,abs,4,0,op_bit,0),(AND,abs,4,0,op_and,0),(ROL,abs,6,0,op_rol,0),(RLA,abs,6,0,op_rla,1),
/* 3x */(BMI,rel,2,1,op_bmi,0),(AND,izy,5,1,op_and,0),(NOP,imp,0,0,op_kil,1),(RLA,izy,8,0,op_rla,1),(NOP,zpx,4,0,op_nop,1),(AND,zpx,4,0,op_and,0),(ROL,zpx,6,0,op_rol,0),(RLA,zpx,6,0,op_rla,1),(SEC,imp,2,0,op_sec,0),(AND,aby,4,1,op_and,0),(NOP,imp,2,0,op_nop,1),(RLA,aby,7,0,op_rla,1),(NOP,abx,4,1,op_nop,1),(AND,abx,4,1,op_and,0),(ROL,abx,7,0,op_rol,0),(RLA,abx,7,0,op_rla,1),
/* 4x */(RTI,imp,6,0,op_rti,0),(EOR,izx,6,0,op_eor,0),(NOP,imp,0,0,op_kil,1),(SRE,izx,8,0,op_sre,1),(NOP,zp, 3,0,op_nop,1),(EOR,zp, 3,0,op_eor,0),(LSR,zp, 5,0,op_lsr,0),(SRE,zp, 5,0,op_sre,1),(PHA,imp,3,0,op_pha,0),(EOR,imm,2,0,op_eor,0),(LSR,imp,2,0,op_lsr,0),(ALR,imm,2,0,op_alr,1),(JMP,abs,3,0,op_jmp,0),(EOR,abs,4,0,op_eor,0),(LSR,abs,6,0,op_lsr,0),(SRE,abs,6,0,op_sre,1),
/* 5x */(BVC,rel,2,1,op_bvc,0),(EOR,izy,5,1,op_eor,0),(NOP,imp,0,0,op_kil,1),(SRE,izy,8,0,op_sre,1),(NOP,zpx,4,0,op_nop,1),(EOR,zpx,4,0,op_eor,0),(LSR,zpx,6,0,op_lsr,0),(SRE,zpx,6,0,op_sre,1),(CLI,imp,2,0,op_cli,0),(EOR,aby,4,1,op_eor,0),(NOP,imp,2,0,op_nop,1),(SRE,aby,7,0,op_sre,1),(NOP,abx,4,1,op_nop,1),(EOR,abx,4,1,op_eor,0),(LSR,abx,7,0,op_lsr,0),(SRE,abx,7,0,op_sre,1),
/* 6x */(RTS,imp,6,0,op_rts,0),(ADC,izx,6,0,op_adc,0),(NOP,imp,0,0,op_kil,1),(RRA,izx,8,0,op_rra,1),(NOP,zp, 3,0,op_nop,1),(ADC,zp, 3,0,op_adc,0),(ROR,zp, 5,0,op_ror,0),(RRA,zp, 5,0,op_rra,1),(PLA,imp,4,0,op_pla,0),(ADC,imm,2,0,op_adc,0),(ROR,acc,2,0,op_ror,0),(ARR,imm,2,0,op_arr,1),(JMP,ind,5,0,op_jmp,0),(ADC,abs,4,0,op_adc,0),(ROR,abs,6,0,op_ror,0),(RRA,abs,6,0,op_rra,1),
/* 7x */(BVS,rel,2,1,op_bvs,0),(ADC,izy,5,1,op_adc,0),(NOP,imp,0,0,op_kil,1),(RRA,izy,8,0,op_rra,1),(NOP,zpx,4,0,op_nop,1),(ADC,zpx,4,0,op_adc,0),(ROR,zpx,6,0,op_ror,0),(RRA,zpx,6,0,op_rra,1),(SEI,imp,2,0,op_sei,0),(ADC,aby,4,1,op_adc,0),(NOP,imp,2,0,op_nop,1),(RRA,aby,7,0,op_rra,1),(NOP,abx,4,1,op_nop,1),(ADC,abx,4,1,op_adc,0),(ROR,abx,7,0,op_ror,0),(RRA,abx,7,0,op_rra,1),
/* 8x */(NOP,imm,2,0,op_nop,1),(STA,izx,6,0,op_sta,0),(NOP,imm,2,0,op_nop,1),(SAX,izx,6,0,op_sax,1),(STY,zp, 3,0,op_sty,0),(STA,zp, 3,0,op_sta,0),(STX,zp, 3,0,op_stx,0),(SAX,zp, 3,0,op_sax,1),(DEY,imp,2,0,op_dey,0),(NOP,imm,2,0,op_nop,1),(TXA,imp,2,0,op_txa,0),(XAA,imm,2,1,op_xaa,1),(STY,abs,4,0,op_sty,0),(STA,abs,4,0,op_sta,0),(STX,abs,4,0,op_stx,0),(SAX,abs,4,0,op_sax,1),
/* 9x */(BCC,rel,2,1,op_bcc,0),(STA,izy,6,0,op_sta,0),(NOP,imp,0,0,op_kil,1),(AHX,izy,6,0,op_sha,1),(STY,zpx,4,0,op_sty,0),(STA,zpx,4,0,op_sta,0),(STX,zpy,4,0,op_stx,0),(SAX,zpy,4,0,op_sax,1),(TYA,imp,2,0,op_tya,0),(STA,aby,5,0,op_sta,0),(TXS,imp,2,0,op_txs,0),(TAS,aby,5,0,op_tas,1),(SHY,abx,5,0,op_shy,1),(STA,abx,5,0,op_sta,0),(SHX,aby,5,0,op_shx,1),(AHX,aby,5,0,op_sha,1),
/* Ax */(LDY,imm,2,0,op_ldy,0),(LDA,izx,6,0,op_lda,0),(LDX,imm,2,0,op_ldx,0),(LAX,izx,6,0,op_lax,1),(LDY,zp, 3,0,op_ldy,0),(LDA,zp, 3,0,op_lda,0),(LDX,zp, 3,0,op_ldx,0),(LAX,zp, 3,0,op_lax,1),(TAY,imp,2,0,op_tay,0),(LDA,imm,2,0,op_lda,0),(TAX,imp,2,0,op_tax,0),(LXA,imm,2,0,op_lax,1),(LDY,abs,4,0,op_ldy,0),(LDA,abs,4,0,op_lda,0),(LDX,abs,4,0,op_ldx,0),(LAX,abs,4,0,op_lax,1),
/* Bx */(BCS,rel,2,1,op_bcs,0),(LDA,izy,5,1,op_lda,0),(NOP,imp,0,0,op_kil,1),(LAX,izy,5,1,op_lax,1),(LDY,zpx,4,0,op_ldy,0),(LDA,zpx,4,0,op_lda,0),(LDX,zpy,4,0,op_ldx,0),(LAX,zpy,4,0,op_lax,1),(CLV,imp,2,0,op_clv,0),(LDA,aby,4,1,op_lda,0),(TSX,imp,2,0,op_tsx,0),(LAS,aby,4,1,op_las,1),(LDY,abx,4,1,op_ldy,0),(LDA,abx,4,1,op_lda,0),(LDX,aby,4,1,op_ldx,0),(LAX,aby,4,1,op_lax,1),
/* Cx */(CPY,imm,2,0,op_cpy,0),(CMP,izx,6,0,op_cmp,0),(NOP,imm,2,0,op_nop,1),(DCP,izx,8,0,op_dcp,1),(CPY,zp, 3,0,op_cpy,0),(CMP,zp, 3,0,op_cmp,0),(DEC,zp, 5,0,op_dec,0),(DCP,zp, 5,0,op_dcp,1),(INY,imp,2,0,op_iny,0),(CMP,imm,2,0,op_cmp,0),(DEX,imp,2,0,op_dex,0),(AXS,imm,2,0,op_axs,1),(CPY,abs,4,0,op_cpy,0),(CMP,abs,4,0,op_cmp,0),(DEC,abs,6,0,op_dec,0),(DCP,abs,6,0,op_dcp,1),
/* Dx */(BNE,rel,2,1,op_bne,0),(CMP,izy,5,1,op_cmp,0),(NOP,imp,0,0,op_kil,1),(DCP,izy,8,0,op_dcp,1),(NOP,zpx,4,0,op_nop,1),(CMP,zpx,4,0,op_cmp,0),(DEC,zpx,6,0,op_dec,0),(DCP,zpx,6,0,op_dcp,1),(CLD,imp,2,0,op_cld,0),(CMP,aby,4,1,op_cmp,0),(NOP,imp,2,0,op_nop,1),(DCP,aby,7,0,op_dcp,1),(NOP,abx,4,1,op_nop,1),(CMP,abx,4,1,op_cmp,0),(DEC,abx,7,0,op_dec,0),(DCP,abx,7,0,op_dcp,1),
/* Ex */(CPX,imm,2,0,op_cpx,0),(SBC,izx,6,0,op_sbc,0),(NOP,imm,2,0,op_nop,1),(ISB,izx,8,0,op_isb,1),(CPX,zp, 3,0,op_cpx,0),(SBC,zp, 3,0,op_sbc,0),(INC,zp, 5,0,op_inc,0),(ISB,zp, 5,0,op_isb,1),(INX,imp,2,0,op_inx,0),(SBC,imm,2,0,op_sbc,0),(NOP,imp,2,0,op_nop,0),(SBC,imm,2,0,op_sbc,1),(CPX,abs,4,0,op_cpx,0),(SBC,abs,4,0,op_sbc,0),(INC,abs,6,0,op_inc,0),(ISB,abs,6,0,op_isb,1),
/* Fx */(BEQ,rel,2,1,op_beq,0),(SBC,izy,5,1,op_sbc,0),(NOP,imp,0,0,op_kil,1),(ISB,izy,8,0,op_isb,1),(NOP,zpx,4,0,op_nop,1),(SBC,zpx,4,0,op_sbc,0),(INC,zpx,6,0,op_inc,0),(ISB,zpx,6,0,op_isb,1),(SED,imp,2,0,op_sed,0),(SBC,aby,4,1,op_sbc,0),(NOP,imp,2,0,op_nop,1),(ISB,aby,7,0,op_isb,1),(NOP,abx,4,1,op_nop,1),(SBC,abx,4,1,op_sbc,0),(INC,abx,7,0,op_inc,0),(ISB,abx,7,0,op_isb,1),
    ];

/**************************
 * Memory Addressing Modes
 **************************/

pub fn fetch_operand_address(cpu:&mut CpuContext) -> Word
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
            let lo = cpu.instruction.operand[0];
            lo as Word
        }
        AddressingMode::ZeroPageX => {
            let lo = cpu.instruction.operand[0];
            
            lo.wrapping_add(cpu.regs.x) as Word
        }
        AddressingMode::ZeroPageY => {
            let lo = cpu.instruction.operand[0];
            
            lo.wrapping_add(cpu.regs.y) as Word
        }
        AddressingMode::Absolute => {
            let lo = cpu.instruction.operand[0];
            let hi = cpu.instruction.operand[1];     // just making sure the registers are used properly here
            
            Word::make(hi, lo)
        }
        AddressingMode::AbsoluteX => {
            let ptr = cpu.instruction.operand[0];
            
            let base = ptr as Word + cpu.regs.x as Word;
            let carry = base.hi() > 0;
            
            let mut hi = cpu.instruction.operand[1];
            
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
            let ptr = cpu.instruction.operand[0];

            let base = ptr as Word + cpu.regs.y as Word;
            let carry = base.hi() > 0;
            
            let mut hi = cpu.instruction.operand[1];

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
            let base_lo = cpu.instruction.operand[0];
            let base_hi = cpu.instruction.operand[1];

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
            let base = cpu.instruction.operand[0];
            let addr = base.wrapping_add(cpu.regs.x);

            let lo = cpu.memory.read(addr as Word);
            let hi = cpu.memory.read(addr.wrapping_add(1) as Word);
            Word::make(hi, lo)
        }
        AddressingMode::IndirectY => {
            let ptr = cpu.instruction.operand[0];

            let base = cpu.memory.read(ptr as Word) as Word + cpu.regs.y as Word;
            let mut hi = cpu.memory.read(ptr.wrapping_add(1) as Word);

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
            let offset = cpu.instruction.operand[0];

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
    
    cpu.regs.ac = cpu.alu_sums(cpu.regs.ac, value);
    
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
    if cpu.instruction.addr_mode == acc {
        let result = cpu.alu_asl(cpu.regs.ac);
        cpu.update_status(result);
        cpu.regs.ac = result;
    }
    else {
        let addr = fetch_operand_address(cpu);
        let value = cpu.memory.read(addr);
        
        let result = cpu.alu_asl(value);
        cpu.update_status(result);
    
        // now write the value back out
        cpu.memory.write(addr, result);
    }
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
    cpu.regs.p.overflow = value.bit(6);
    cpu.regs.p.negative = value.bit(7);
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
    let mut status = cpu.regs.p.to_byte();
    
    // disable interrupts now
    cpu.regs.p.interrupt = true; 
    if !cpu.reset && !cpu.nmi && cpu.irq {
        // add the break bit
        status |= 0x10;
    }
        
    /*** At this point, the signal status determines which interrupt vector is used ***/
    cpu.stack_push(status);

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
    cpu.update_status(cpu.regs.ac);
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
    let value = cpu.memory.read(addr);//.wrapping_sub(ternary!(cpu.regs.p.carry, 0u8, 1u8));

    // further ambitious bullshit from me...
    // two's complement based subtraction is 
    // the same as a + !b, so just use the add routine

    cpu.regs.ac = cpu.alu_sums(cpu.regs.ac, !value);
    
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

pub fn op_isb(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr);

    let result = value.wrapping_add(1);
    
    cpu.regs.ac = cpu.alu_sums(cpu.regs.ac, !result);
    cpu.update_status(cpu.regs.ac);

    cpu.memory.write(addr, result);
}

pub fn op_dcp(cpu: &mut CpuContext)
{
    let addr = fetch_operand_address(cpu);
    let value = cpu.memory.read(addr).wrapping_sub(1);

    cpu.alu_compare(cpu.regs.ac, value);
    cpu.memory.write(addr, value);
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
    cpu.regs.ac = cpu.alu_sums(cpu.regs.ac, result);
    cpu.update_status(cpu.regs.ac);

    cpu.memory.write(addr, result);
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

    let result = cpu.alu_lsr(value);
    cpu.regs.ac ^= result;
    cpu.update_status(cpu.regs.ac);

    cpu.memory.write(addr, result);
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
    
    let result = cpu.alu_rol(value);
    cpu.regs.ac &= result;
    cpu.update_status(cpu.regs.ac);

    cpu.memory.write(addr, result);
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

    let result = cpu.alu_asl(value);
    cpu.regs.ac |= result;
    cpu.update_status(cpu.regs.ac);

    cpu.memory.write(addr, result);
}