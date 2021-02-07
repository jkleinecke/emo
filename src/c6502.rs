#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::common::{get_bit, Clocked, Memory};

pub struct EmoC6502 {
    pub a: u8,                      // Accumulator Register
    pub x: u8,                      // X Register
    pub y: u8,                      // Y Register
    pub sp: u8,                     // Stack Pointer
    pub pc: u16,                    // Program Counter
    
    pub carry:bool,             // carry
    pub zero:bool,             // zero
    pub interrupt:bool,             // interrupt disable
    pub decimal:bool,             // decimal mode
    pub brk:bool,             // break
    pub overflow:bool,             // overflow
    pub negative:bool,             // negative
}

impl Clocked for EmoC6502
{
    fn clock(&mut self, memory: &mut impl Memory)
    {
        let opscode = self.fetch(memory); // current ops code
        self.pc += 1;                   // move to the next instruction

        //self.S = StatusFlags::C | StatusFlags::Z;

        // loop {
        //     let opscode = program[self.PC as usize];
        //     self.PC += 1;

        match opscode {
            0xA9 => {                   // LDA
                let param = self.fetch(memory);   // mem address to load
                self.pc += 1;
                self.a = param;

                self.zero = self.a == 0;  // set the zero flag if we're at zero
            }
            0xAA => {                   // TAX
                self.x = self.a;

                self.zero      = self.x == 0;
                self.negative  = get_bit(self.x, 8) != 0;
            }       
            _ => todo!()
        }
    }
}

impl EmoC6502 {
    pub fn new() -> Self {
        EmoC6502 {
            a: 0,
            x: 0,
            y: 0,
            sp: 0,
            pc: 0,

            carry:false,           
            zero:false,            
            interrupt:false,
            decimal:false,          
            brk:false,        
            overflow:false,
            negative:false,         
        }
    }

    fn fetch(&mut self, memory: &mut impl Memory) -> u8 {
        memory.load(self.pc)
    }

}
