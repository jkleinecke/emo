#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::cpu6502::{Cpu6502,PC_START};
use crate::bus::{Bus};

const SYSMEMSIZE: usize = 0xFFFF;
const PROGRAM_START: u16 = 0x8000;

pub struct Nes {
    pub cpu:Cpu6502,
}

pub trait Clocked {
    fn clock(&mut self);  
}

impl Nes
{
    pub fn new() -> Self 
    {
        Nes {
            cpu: Cpu6502::new(Box::new(Bus::new())),
        }
    }

    pub fn reset(&mut self)
    {
        self.cpu.reset();
    }

    pub fn clock(&mut self)
    {
        // CPU clock also gets a data bus clock
        self.cpu.clock();
    }

    // TODO: this will eventually become load_cartridge
    pub fn load_program(&mut self, program: &Vec<u8>) 
    {
        // TODO: bounds check?
        self.cpu.bus.write_ram(PROGRAM_START as usize, program);

        // Now tell the cpu where the program starts
        let start_addr = PROGRAM_START.to_le_bytes();
        self.cpu.bus.write_ram(PC_START as usize, &start_addr.to_vec());

        self.reset();
    }
}

