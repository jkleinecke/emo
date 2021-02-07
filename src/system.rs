#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::c6502::{EmoC6502,PC_START};
use crate::bus::{Bus,BusInterface};
use crate::common::{MemoryBus,Clocked};

const SYSMEMSIZE: usize = 0xFFFF;
const PROGRAM_START: u16 = 0x8000;

pub struct Nes {
    pub cpu:EmoC6502,
    pub bus:Bus,
}

impl Nes
{
    pub fn new() -> Self 
    {
        Nes {
            cpu: EmoC6502::new(),
            bus: Bus::new(),
        }
    }

    pub fn reset(&mut self)
    {
        self.cpu.reset = true;
    }

    pub fn clock(&mut self)
    {
        self.cpu.clock();
    }

    // TODO: this will eventually become load_cartridge
    pub fn load_program(&mut self, program: &Vec<u8>) 
    {
        // TODO: bounds check?
        self.bus.write_ram(PROGRAM_START as usize, program);

        // Now tell the cpu where the program starts
        let start_addr = PROGRAM_START.to_le_bytes();
        self.bus.write_ram(PC_START as usize, &start_addr.to_vec());
    }
}
