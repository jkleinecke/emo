#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(unused_variables)]

use super::{MemoryMapped,Ram,Bit,BitTest,Clocked,WORD,Byte,Word,StatusRegister,Registers,CpuContext,ternary};
use super::operations::*;
use std::fmt;


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

pub const PC_START: Word = 0xFFFC;

#[derive(Copy,Clone)]
pub struct State {
    pub a: Byte,                      // Accumulator Register
    pub x: Byte,                      // X Register
    pub y: Byte,                      // Y Register
    pub sp: Byte,                     // Stack Pointer
    pub pc: Word,                        // Internal Program Counter Register
    
    pub status: StatusRegister,    // Status Register

    // internal registers
    pub ir_cycles: u8,
    pub instruction: DecodedInstruction,   // active instruction register
    
    halted: bool,                   // fake register to show that we should be done
}


impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x}", self.a, self.x, self.y, self.status.to_byte(), self.sp)
    }
}

#[derive(Copy,Clone)]
pub struct DecodedInstruction {
    pub ir: Byte,
    pub ir_address: Word,

    pub opcode: Operation,
    pub addr_mode: AddressingMode,
    pub opsize: u8,
    pub operand: [u8;2],

    pub cycles: u8,
    pub oops: bool,
}

impl Default for DecodedInstruction {
    fn default() -> Self { 
        Self {
            ir:0,ir_address:0,
            opcode: Operation::BRK,
            addr_mode: AddressingMode::Implicit,
            opsize:1,
            operand:[0;2],
            cycles:7,
            oops:false,
        }
     }
}


fn fmt_addressmode(addr_mode: AddressingMode, addr: Word) -> String {
    match addr_mode {
        abs => format!("${:04x}", addr),
        acc => format!("A"),
        imm => format!("#${:02x}", addr & 0xff),
        imp => format!(""),
        izx => format!("(${:02x},X)", addr & 0xff),
        izy => format!("(${:02x},Y)", addr & 0xff),
        zp =>  format!("${:02x}", addr & 0xff),
        zpx => format!("${:02x},X", addr & 0xff),
        zpy => format!("${:02x},Y", addr & 0xff),
        rel => format!("${:02x}", addr & 0xff),
        abx => format!("${:04x},X", addr),
        aby => format!("${:04x},Y", addr),
        ind => format!("(${:04x})", addr),
    }
}

fn fmt_operands(ir: Byte, op1: Byte, op2: Byte, size: Byte) -> String {
    match size {
        0 | 1 => format!("{:02x}      ", ir),
        2 => format!("{:02x} {:02x}   ", ir, op1),
        3 => format!("{:02x} {:02x} {:02x}", ir, op1, op2),
        _ => panic!("bad opsize while formatting operands"),
    }
}

impl fmt::Display for DecodedInstruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:04x}  {}  {} {}", self.ir_address, fmt_operands(self.ir, self.operand[0], self.operand[1], self.opsize), self.opcode, fmt_addressmode(self.addr_mode, Word::make(self.operand[1],self.operand[0])))
    }
}

pub struct Cpu {
    pub cycle_counter: u64,
    pub regs: Registers,

    pub reset: Bit,
    pub irq: Bit,
    pub nmi: Bit,

    // internal registers
    pub ir_cycles: u8,
    pub instr: DecodedInstruction,
    
    halted: bool,                   // fake register to show that we should be done
}

impl Clocked for Cpu
{
    fn clock(&mut self, memory: &mut impl MemoryMapped) 
    {
        self.cycle_counter = self.cycle_counter.wrapping_add(1);

        if self.reset {
            // a reset signal can interrupt a currently running instruction
            self.halted = false;
            self.ir_cycles = 0;
            self.regs.sp = 0x0;
        }

        if self.halted {
            return;
        }

        // todo - Handle any cpu interrupts

        if !self.reset && self.ir_cycles < self.instr.cycles
        {
            // wait the appropriate number of cycles
            // for the last instruction that was executed

            self.ir_cycles += 1;
            return;             
        }
        
        // make sure that interrupts aren't disabled for irq
        let interrupted = (self.irq && !self.regs.p.interrupt) || (self.nmi) || (self.reset);

        // Fetch the IR - BRK if interrupted    
        let ir = ternary!(interrupted, 0, memory.read(self.regs.pc));
        
        // Decode the IR
        let (operation, addr_mode, c1, c2, exec_op, illegal) = OPCODE_TABLE[ir as usize];

        self.instr = DecodedInstruction {
            ir: ir,
            ir_address: self.regs.pc,
            opcode: operation,
            addr_mode: addr_mode,
            opsize: ternary!(interrupted, 0, ADDR_OPSIZE_TABLE[addr_mode as usize]),
            operand: [0;2],
            cycles: c1,
            oops: ternary!(c2 == 0, false, true),    
        };

        self.halted = operation == Operation::KIL;

        if self.halted {
            return;
        }

        self.ir_cycles = 0;   

        self.instr.operand[0] = ternary!(self.instr.opsize > 1, memory.read(self.regs.pc + 1), 0);
        self.instr.operand[1] = ternary!(self.instr.opsize > 2, memory.read(self.regs.pc + 2), 0);

        let opsize = self.instr.opsize;
        let mut context = CpuContext::new(&mut self.regs, &mut self.instr, self.reset, self.nmi, self.irq, memory);

        // Execute the IR operation
        exec_op(&mut context);

        if context.increment_programcounter {
            context.regs.pc = context.regs.pc.wrapping_add(opsize as Word);
        }

        //self.instr.cycles += ternary!(context.oops, 1, 0); // account for the oops cycle if the oops condition was encountered 
        self.halted = context.halt; // halt if necessary...

        // reset the flags
        // reset precedes all other interrupts
        if self.reset {
            self.reset = false;
        }
        else if self.nmi {
            self.nmi = false;
        }
        //-- may need to also reset the IRQ flag here, unclear how to handle this...
    }
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            cycle_counter: 0,
            regs: Registers {
                ac: 0,
                x: 0,
                y: 0,
                sp: 0xFF,
                pc: PC_START,
                p: StatusRegister::from(0x34),
            },
            reset: false,
            irq: false,
            nmi: false,
            ir_cycles: 0, 
            instr: DecodedInstruction::default(),
            halted: false,
        }
    }

    pub fn did_halt(&self) -> bool {
        self.halted
    }

    pub fn copy_state(&self) -> State {
        State {
            a: self.regs.ac,
            x: self.regs.x,
            y: self.regs.y,
            pc: self.regs.pc,
            sp: self.regs.sp,
            status: self.regs.p,
            ir_cycles: self.ir_cycles,
            instruction: self.instr,
            halted: self.halted,
        }
    }
}
