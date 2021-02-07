#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::c6502::EmoC6502;
use crate::memory::SystemMemory;
use crate::common::Clocked;

const SYSMEMSIZE: usize = 0xFFFF;

pub struct NesSystem<'a> {
    pub cpu:EmoC6502<'a>,
    pub ram:SystemMemory,
}

impl<'a> Clocked for NesSystem<'a>
{
    fn clock(&mut self)
    {
        // Master clock speed is 21.477272 Mhz


        // CPU is 1:12
        self.cpu.clock();

        // PPU 1:4

        // APU 1:24 (sort of...)
    }
}

impl<'a> NesSystem<'a>
{
    pub fn new() -> Self 
    {
        let mut sys : NesSystem<'a>;

        sys.ram = SystemMemory::new();
        sys.cpu = EmoC6502::new(&sys.ram);

        return sys;
    }
}