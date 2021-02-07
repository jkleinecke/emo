
use crate::common::Memory;

const SYSMEMSIZE: usize = 0xFFFF;

pub struct SystemMemory {
    pub data: [u8;SYSMEMSIZE]
}

impl Memory for SystemMemory {
    fn load(&mut self, ptr: u16) -> u8
    {
        return self.data[ptr as usize];
    }

    fn store(&mut self, ptr: u16, value: u8)
    {
        self.data[ptr as usize] = value;
    }
}

impl SystemMemory {
    pub fn new() -> Self {
        SystemMemory {
            data: [0;SYSMEMSIZE]
        }
    }
}