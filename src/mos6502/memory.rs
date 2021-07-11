use super::{Byte,Word};

pub trait Memory {
    fn read(&self, addr:Word) -> Byte;
    fn write(&mut self, addr:Word, value:Byte);
}

pub struct Ram {
    pub data: [Byte;0xFFFF],
}

impl Memory for Ram {
    fn read(&self, addr:Word) -> Byte {
        self.data[addr as usize]
    }
    fn write(&mut self, addr:Word, value:Byte) {
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