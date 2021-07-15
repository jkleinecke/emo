use super::{Byte,Word,MemoryMapped};

pub struct Ram {
    pub data: [Byte;0xFFFF],
}

impl MemoryMapped for Ram {
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