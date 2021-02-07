#![allow(unused_imports)]

use crate::system::Nes;
use crate::c6502::{ProcessorStatus,PC_START};

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_LDA_immediate() 
    {
        let mut nes = Nes::new();
        
        // setup some test instructions
        nes.write_ram(PC_START as usize, vec![0x00,0x80]);   // little endian address to where the instructions start
        nes.write_ram(0x8000,   vec![0xa9,0x05]);   // actual instructions
        
        nes.reset();
        nes.clock();

        assert_eq!(nes.cpu.a, 0x05);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Zero), false);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Negative), false);
    }

    #[test]
    fn test_LDA_zero() 
    {
        let mut nes = Nes::new();

        nes.write_ram(PC_START as usize, vec![0x00,0x80]);   // little endian address to where the instructions start
        nes.write_ram(0x8000,   vec![0xa9,0x00]);   // actual instructions
        
        nes.reset();
        nes.clock();

        assert_eq!(nes.cpu.a, 0x00);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Zero), true);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Negative), false);
    }
}