#![allow(unused_imports)]

use crate::system::Nes;
use crate::common::WORD;
use crate::cpu6502::{Status};
use crate::system::PROGRAM_START;

#[cfg(test)]
mod test {
    use super::*;

    fn run_instr(nes: &mut Nes, num_instr: u32) {
        if nes.cpu.ir_cycles != 0 {
            // first finish any current instructions (like the initial reset)
            nes.step_instruction();
        }

        for _i in 0..num_instr {
            nes.step_instruction();
        }
    }

    #[test]
    fn cpu_op_lda_imm() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        // setup some test instructions
        nes.load_program(&vec![0xa9,0x05]);   // 2 cycles for the LDA imm

        run_instr(&mut nes, 1);

        assert_eq!(nes.sys_clocks, 9);
        assert_eq!(nes.cpu.a, 0x05);
        assert_eq!(nes.cpu.status.zero(), false);
        assert_eq!(nes.cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_imm_zero() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        // setup some test instructions
        nes.load_program(&vec![0xa9,0x00]);   // 2 cycles for the LDA imm
        
        run_instr(&mut nes, 1);

        assert_eq!(nes.sys_clocks, 9);
        assert_eq!(nes.cpu.a, 0x00);
        assert_eq!(nes.cpu.status.zero(), true);
        assert_eq!(nes.cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_imm_negative() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        // setup some test instructions
        nes.load_program(&vec![0xa9,0xFF]);   // 2 cycles for the LDA imm
        
        run_instr(&mut nes, 1);

        assert_eq!(nes.sys_clocks, 9); 
        assert_eq!(nes.cpu.a, 0xFF);
        assert_eq!(nes.cpu.status.zero(), false);
        assert_eq!(nes.cpu.status.negative(), true);
    }

    #[test]
    fn cpu_op_lda_abs() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        let addr = 0x8003;
        // setup some test instructions
        nes.load_program(&vec![0xad, addr.lo(), addr.hi(), 0x05]);   // 4 cycles for the LDA abs
        
        run_instr(&mut nes, 1);

        assert_eq!(nes.sys_clocks, 11); 
        assert_eq!(nes.cpu.a, 0x05);
        assert_eq!(nes.cpu.status.zero(), false);
        assert_eq!(nes.cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_tax() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        // setup some test instructions
        nes.load_program(&vec![
            0xa9,0x42,  // LDA 2
            0xaa        // TAX 2
            ]);   
        
        run_instr(&mut nes, 2);

        assert_eq!(nes.sys_clocks, 11); 
        assert_eq!(nes.cpu.x, 0x42);
        assert_eq!(nes.cpu.status.zero(), false);
        assert_eq!(nes.cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_abx() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        let mut prog = [0u8;255];
        prog[0] = 0xa9; // LDA imm
        prog[1] = 0x42; // a = 0x42
        prog[2] = 0xaa; // x = a
        prog[3] = 0xbd; // LDA abx
        prog[4] = 0x10; // ptr lo = 10
        prog[5] = 0x80; // ptr hi = 80
        
        prog[82] = 0x66; // 0x10 + 0x42 value 0x66 

        // setup some test instructions
        nes.load_program(&prog.to_vec());   
        
        run_instr(&mut nes, 3);   

        assert_eq!(nes.sys_clocks, 15); // 15 clocks because no oops 
        assert_eq!(nes.cpu.a, 0x66);
        assert_eq!(nes.cpu.status.zero(), false);
        assert_eq!(nes.cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_abx_oops() 
    {
        let mut nes = Nes::new();   // 7 cycles for startup
        
        let mut prog = [0u8;512];
        prog[0] = 0xa9; // LDA imm
        prog[1] = 0xFF; // a = 0x42
        prog[2] = 0xaa; // TAX x = a
        prog[3] = 0xbd; // LDA abx
        prog[4] = 0x10; // ptr lo = 10
        prog[5] = 0x80; // ptr hi = 80
        
        prog[271] = 0x66; // 0x10 + 0xFF value 0x66 

        // setup some test instructions
        nes.load_program(&prog.to_vec());   
        
        run_instr(&mut nes, 3); // still only 3 instructions
        
        assert_eq!(nes.sys_clocks, 16); // 16 clocks because oops 
        assert_eq!(nes.cpu.a, 0x66);
        assert_eq!(nes.cpu.status.carry(), false);
        assert_eq!(nes.cpu.status.zero(), false);
        assert_eq!(nes.cpu.status.negative(), false);
    }

}