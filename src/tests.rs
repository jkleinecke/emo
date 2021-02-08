#![allow(unused_imports)]

use crate::system::Nes;
use crate::c6502::{ProcessorStatus,PC_START};

#[cfg(test)]
mod test {
    use super::*;

    fn run_clocks(nes: &mut Nes, num_clocks: u32) {
        for i in 0..num_clocks {
            nes.clock();
        }
    }

    #[test]
    fn test_lda_immediate() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        // setup some test instructions
        nes.load_program(&vec![0xa9,0x05]);   // 2 cycles for the LDA
        
        run_clocks(&mut nes, 9);

        assert_eq!(nes.cpu.a, 0x05);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Zero), false);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Negative), false);
    }

    #[test]
    fn test_lda_zero() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        // setup some test instructions
        nes.load_program(&vec![0xa9,0x00]);   // 2 cycles for the LDA
        
        run_clocks(&mut nes, 9);

        assert_eq!(nes.cpu.a, 0x00);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Zero), true);
        assert_eq!(nes.cpu.status.contains(ProcessorStatus::Negative), false);
    }
}