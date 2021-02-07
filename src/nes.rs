#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::c6502::EmoC6502;
use crate::memory::SystemMemory;
use crate::common::Clocked;

const SYSMEMSIZE: usize = 0xFFFF;

pub struct NesSystem {
    pub cpu:EmoC6502,
    pub ram:SystemMemory,
}

impl NesSystem
{
    pub fn new() -> Self 
    {
        NesSystem {
            cpu: EmoC6502::new(),
            ram: SystemMemory::new(),
        }
    }

    pub fn clock(&mut self)
    {
        self.cpu.clock(&mut self.ram);
    }
}