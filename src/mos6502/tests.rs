#![allow(unused_imports)]

use super::cpu6502::{Cpu6502,Ram,Status};
use crate::common::{Clocked,WORD};
use std::cell::RefCell;
use std::rc::Rc;

#[cfg(test)]
mod test {
    use super::*;

    fn new_cpu(program: &Vec<u8>) -> Cpu6502
    {
        let mut ram = Ram::new();
        ram.data[0x600 .. (0x600 + program.len())].copy_from_slice(&program[..]);

        let bus = Rc::new(RefCell::new(ram));
        let mut cpu = Cpu6502::new(bus.clone());
        cpu.pc = 0x600;
        cpu.status = Status::from_str("nv-Bdizc");

        cpu 
    }

    fn run_instr(cpu: &mut Cpu6502, num_instr: u32) {
        for _i in 0..num_instr {
            cpu.ir_cycles = 0;  // don't care about cycle accuracy here
            cpu.clock();
        }
    }

    #[test]
    fn cpu_op_lda_imm() 
    {
        let mut cpu = new_cpu(&vec![0xa9,0x05]);

        run_instr(&mut cpu, 1);

        assert_eq!(cpu.a, 0x05);
        assert_eq!(cpu.status.zero(), false);
        assert_eq!(cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_imm_zero() 
    {
        let mut cpu = new_cpu(&vec![0xa9,0x00]);
        
        run_instr(&mut cpu, 1);

        assert_eq!(cpu.a, 0x00);
        assert_eq!(cpu.status.zero(), true);
        assert_eq!(cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_imm_negative() 
    {
        let mut cpu = new_cpu(&vec![0xa9,0xFF]);
        
        run_instr(&mut cpu, 1);

        assert_eq!(cpu.a, 0xFF);
        assert_eq!(cpu.status.zero(), false);
        assert_eq!(cpu.status.negative(), true);
    }

    #[test]
    fn cpu_op_lda_abs() 
    {
        let addr = 0x8003;
        let mut cpu = new_cpu(&vec![0xad, addr.lo(), addr.hi(), 0x05]);
        
        run_instr(&mut cpu, 1);

        assert_eq!(cpu.a, 0x05);
        assert_eq!(cpu.status.zero(), false);
        assert_eq!(cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_tax() 
    {
        let mut cpu = new_cpu(&vec![ 
            0xa9,0x42,  // LDA 2
            0xaa        // TAX 2
            ]);
        
        run_instr(&mut cpu, 2);

        assert_eq!(cpu.x, 0x42);
        assert_eq!(cpu.status.zero(), false);
        assert_eq!(cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_abx() 
    {
        let mut prog = [0u8;255];
        prog[0] = 0xa9; // LDA imm
        prog[1] = 0x42; // a = 0x42
        prog[2] = 0xaa; // x = a
        prog[3] = 0xbd; // LDA abx
        prog[4] = 0x10; // ptr lo = 10
        prog[5] = 0x06; // ptr hi = 06
        
        prog[82] = 0x66; // 0x10 + 0x42 value 0x66 

        let mut cpu = new_cpu(&prog.to_vec());
        
        run_instr(&mut cpu, 3);   

        assert_eq!(cpu.a, 0x66);
        assert_eq!(cpu.status.zero(), false);
        assert_eq!(cpu.status.negative(), false);
    }

    #[test]
    fn cpu_op_lda_abx_oops() 
    {
        let mut prog = [0u8;512];
        prog[0] = 0xa9; // LDA imm
        prog[1] = 0xFF; // a = 0x42
        prog[2] = 0xaa; // TAX x = a
        prog[3] = 0xbd; // LDA abx
        prog[4] = 0x10; // ptr lo = 10
        prog[5] = 0x06; // ptr hi = 06
        
        prog[271] = 0x66; // 0x10 + 0xFF value 0x66 

        let mut cpu = new_cpu(&prog.to_vec());
        
        run_instr(&mut cpu, 3); // still only 3 instructions
        
        assert_eq!(cpu.a, 0x66);
        assert_eq!(cpu.status.carry(), false);
        assert_eq!(cpu.status.zero(), false);
        assert_eq!(cpu.status.negative(), false);
    }

    
    fn run_until_break(cpu: &mut Cpu6502) {
        let mem = cpu.memory_bus.clone();

        while mem.borrow_mut().read(cpu.pc) != 0x00
        {
            cpu.ir_cycles = 0;  // don't care about cycle accuracy here
            cpu.clock();
        }
    }

    // tests taken from Easy 6502 website
    // https://skilldrick.github.io/easy6502/#intro
    #[test]
    fn ez_first()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 01     LDA #$01
        // $0602    8d 00 02  STA $0200
        // $0605    a9 05     LDA #$05
        // $0607    8d 01 02  STA $0201
        // $060a    a9 08     LDA #$08
        // $060c    8d 02 02  STA $0202
        let mut cpu = new_cpu(&vec![ 0xa9, 0x01, 0x8d, 0x00, 0x02, 0xa9, 0x05, 0x8d, 0x01, 0x02, 0xa9, 0x08, 0x8d, 0x02, 0x02 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x08);
        assert_eq!(cpu.x, 0x00);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x60F);
        assert_eq!(cpu.status, Status::from_str("nv-Bdizc"));

        let mut mem = cpu.memory_bus.borrow_mut();
        assert_eq!(mem.read(0x200), 0x01);
        assert_eq!(mem.read(0x201), 0x05);
        assert_eq!(mem.read(0x202), 0x08);
    }

    #[test]
    fn ez_instructions1()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 c0     LDA #$c0
        // $0602    aa        TAX 
        // $0603    e8        INX 
        // $0604    69 c4     ADC #$c4
        // $0606    00        BRK
        let mut cpu = new_cpu(&vec![ 0xa9,0xc0,0xaa,0xe8,0x69,0xc4 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x84);
        assert_eq!(cpu.x, 0xc1);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x606);
        assert_eq!(cpu.status, Status::from_str("Nv-BdizC"));
    }
    
    #[test]
    fn ez_instructions2()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 80     LDA #$80
        // $0602    85 01     STA $01
        // $0604    65 01     ADC $01
        let mut cpu = new_cpu(&vec![ 0xa9,0x80,0x85,0x01,0x65,0x01 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x00);
        assert_eq!(cpu.x, 0x00);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x606);
        assert_eq!(cpu.status, Status::from_str("nV-BdiZC"));
        
        let mut mem = cpu.memory_bus.borrow_mut();
        assert_eq!(mem.read(0x01), 0x80);
    }
    
    #[test]
    fn ez_branching()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a2 08     LDX #$08
        // $0602    ca        DEX 
        // $0603    8e 00 02  STX $0200
        // $0606    e0 03     CPX #$03
        // $0608    d0 f8     BNE $0602
        // $060a    8e 01 02  STX $0201
        // $060d    00        BRK 
        let mut cpu = new_cpu(&vec![ 0xa2,0x08,0xca,0x8e,0x00,0x02,0xe0,0x03,0xd0,0xf8,0x8e,0x01,0x02 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x00);
        assert_eq!(cpu.x, 0x03);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x60d);
        assert_eq!(cpu.status, Status::from_str("nv-BdiZC"));
        
        let mut mem = cpu.memory_bus.borrow_mut();
        assert_eq!(mem.read(0x200), 0x03);
        assert_eq!(mem.read(0x201), 0x03);
    }

    #[test]
    fn ez_indirect()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 01     LDA #$01
        // $0602    85 f0     STA $f0
        // $0604    a9 cc     LDA #$cc
        // $0606    85 f1     STA $f1
        // $0608    6c f0 00  JMP ($00f0)
        let mut cpu = new_cpu(&vec![ 0xa9,0x01,0x85,0xf0,0xa9,0xcc,0x85,0xf1,0x6c,0xf0,0x00 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0xcc);
        assert_eq!(cpu.x, 0x00);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0xcc01);
        assert_eq!(cpu.status, Status::from_str("Nv-Bdizc"));
        
        let mut mem = cpu.memory_bus.borrow_mut();
        assert_eq!(mem.read(0x00f0), 0x01);
        assert_eq!(mem.read(0x00f1), 0xcc);
    }

    
    #[test]
    fn ez_indirect_indexed()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a2 01     LDX #$01
        // $0602    a9 05     LDA #$05
        // $0604    85 01     STA $01
        // $0606    a9 07     LDA #$07
        // $0608    85 02     STA $02
        // $060a    a0 0a     LDY #$0a
        // $060c    8c 05 07  STY $0705
        // $060f    a1 00     LDA ($00,X)
        let mut cpu = new_cpu(&vec![ 0xa2,0x01,0xa9,0x05,0x85,0x01,0xa9,0x07,0x85,0x02,0xa0,0x0a,0x8c,0x05,0x07,0xa1 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x0a);
        assert_eq!(cpu.x, 0x01);
        assert_eq!(cpu.y, 0x0a);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x0611);
        assert_eq!(cpu.status, Status::from_str("nv-Bdizc"));
        
        let mut mem = cpu.memory_bus.borrow_mut();
        assert_eq!(mem.read(0x0001), 0x05);
        assert_eq!(mem.read(0x0002), 0x07);
        assert_eq!(mem.read(0x0705), 0x0a);
    }

    
    #[test]
    fn ez_indirect_y_indexed()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a0 01     LDY #$01
        // $0602    a9 03     LDA #$03
        // $0604    85 01     STA $01
        // $0606    a9 07     LDA #$07
        // $0608    85 02     STA $02
        // $060a    a2 0a     LDX #$0a
        // $060c    8e 04 07  STX $0704
        // $060f    b1 01     LDA ($01),Y
        let mut cpu = new_cpu(&vec![ 0xa0,0x01,0xa9,0x03,0x85,0x01,0xa9,0x07,0x85,0x02,0xa2,0x0a,0x8e,0x04,0x07,0xb1 
            ,0x01 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x0a);
        assert_eq!(cpu.x, 0x0a);
        assert_eq!(cpu.y, 0x01);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x0611);
        assert_eq!(cpu.status, Status::from_str("nv-Bdizc"));
        
        let mut mem = cpu.memory_bus.borrow_mut();
        assert_eq!(mem.read(0x0001), 0x03);
        assert_eq!(mem.read(0x0002), 0x07);
        assert_eq!(mem.read(0x0704), 0x0a);
    }

    
    #[test]
    fn ez_stack()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a2 00     LDX #$00
        // $0602    a0 00     LDY #$00
        // $0604    8a        TXA 
        // $0605    99 00 02  STA $0200,Y
        // $0608    48        PHA 
        // $0609    e8        INX 
        // $060a    c8        INY 
        // $060b    c0 10     CPY #$10
        // $060d    d0 f5     BNE $0604
        // $060f    68        PLA 
        // $0610    99 00 02  STA $0200,Y
        // $0613    c8        INY 
        // $0614    c0 20     CPY #$20
        // $0616    d0 f7     BNE $060f
        let mut cpu = new_cpu(&vec![ 0xa2,0x00,0xa0,0x00,0x8a,0x99,0x00,0x02,0x48,0xe8,0xc8,0xc0,0x10,0xd0,0xf5,0x68 
            ,0x99,0x00,0x02,0xc8,0xc0,0x20,0xd0,0xf7 ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x00);
        assert_eq!(cpu.x, 0x10);
        assert_eq!(cpu.y, 0x20);
        assert_eq!(cpu.sp, 0xff);
        assert_eq!(cpu.pc, 0x0618);
        assert_eq!(cpu.status, Status::from_str("nv-BdiZC"));
        
        let mut mem = cpu.memory_bus.borrow_mut();

        for i in 0..16
        {
            assert_eq!(mem.read(0x0200 + i), i as u8);
        }

        for i in 0..16
        {
            assert_eq!(mem.read(0x0210 + i), 15 - i as u8)
        }
    }

    
    #[test]
    fn ez_subroutine()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    20 09 06  JSR $0609
        // $0603    20 0c 06  JSR $060c
        // $0606    20 12 06  JSR $0612
        // $0609    a2 00     LDX #$00
        // $060b    60        RTS 
        // $060c    e8        INX 
        // $060d    e0 05     CPX #$05
        // $060f    d0 fb     BNE $060c
        // $0611    60        RTS 
        // $0612    00        BRK 
        let mut cpu = new_cpu(&vec![ 0x20,0x09,0x06,0x20,0x0c,0x06,0x20,0x12,0x06,0xa2,0x00,0x60,0xe8,0xe0,0x05,0xd0 
            ,0xfb,0x60,0x00  ]);
        
        run_until_break(&mut cpu);

        assert_eq!(cpu.a, 0x00);
        assert_eq!(cpu.x, 0x05);
        assert_eq!(cpu.y, 0x00);
        assert_eq!(cpu.sp, 0xfd);
        assert_eq!(cpu.pc, 0x0612);
        assert_eq!(cpu.status, Status::from_str("nv-BdiZC"));
    }
}