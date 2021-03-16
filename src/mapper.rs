#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

pub trait Mapper {
    fn read(&mut self, addr:u16) -> u8;
    fn write(&mut self, addr:u16, value:u8);
}

pub struct Ram {
    pub data: [u8;0xFFFF],
}

impl Mapper for Ram {
    fn read(&mut self, addr:u16) -> u8 {
        self.data[addr as usize]
    }
    fn write(&mut self, addr:u16, value:u8) {
        self.data[addr as usize] = value
    }
}

impl Ram {
    pub fn new() -> Self {
        Ram {
            data: [0;0xFFFF],
        }
    }
}