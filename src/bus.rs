#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

pub struct Bus {
    pub cpu_ram: [u8;0xFFFF],
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            cpu_ram: [0;0xFFFF]
        }
    }

    pub fn load(&mut self, ptr: u16) -> u8
    {
        return self.cpu_ram[ptr as usize];
    }

    pub fn store(&mut self, ptr: u16, value: u8)
    {
        self.cpu_ram[ptr as usize] = value;
    }

    pub fn write_ram(&mut self, offset: usize, data: &Vec<u8>)
    {
        self.cpu_ram[offset .. (offset + data.len())].copy_from_slice(&data[..]);
    }

    pub fn read_ram(&mut self, offset: usize, size: usize) -> Vec<u8>
    {
        self.cpu_ram[offset .. (offset + size)].to_vec()
    }
}