use super::{Word,Byte};

pub trait MemoryMapped {
    fn read(&self, addr:Word) -> Byte;
    fn write(&mut self, addr:Word, value:Byte);
}