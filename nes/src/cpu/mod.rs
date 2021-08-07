
// use cpu6502::;
// use operations;
// use memory;

#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

pub use super::{BitTest,Clocked,WORD,Word,Byte,Bit,MemoryMapped,ternary};

mod cpu;
mod context;
mod operations;
mod registers;
mod memory;
mod trace;

pub use self::trace::trace;
pub use self::memory::{Ram};
pub use self::registers::*;

use self::context::CpuContext;
pub use self::cpu::*;
use self::operations::*;


#[cfg(test)]
mod test {
    use super::*;

    fn new_cpu(program: &Vec<u8>) -> (Cpu,Ram)
    {
        let mut ram = Ram::new();
        ram.data[0x600 .. (0x600 + program.len())].copy_from_slice(&program[..]);
        let mut cpu = Cpu::new();
        cpu.regs.pc = 0x600;
        cpu.regs.p = StatusRegister::from("nv-Bdizc");

        (cpu,ram)
    }

    fn run_instr(cpu: &mut Cpu, ram: &mut Ram, num_instr: u32) {
        for _i in 0..num_instr {
            cpu.ir_cycles = 0;  // don't care about cycle accuracy here
            cpu.clock(ram);
        }
    }

    #[test]
    fn cpu_op_lda_imm() 
    {
        let (mut cpu,mut ram) = new_cpu(&vec![0xa9,0x05]);

        run_instr(&mut cpu,&mut ram, 1);

        assert_eq!(cpu.regs.ac, 0x05);
        assert_eq!(cpu.regs.p.zero, false);
        assert_eq!(cpu.regs.p.negative, false);
    }

    #[test]
    fn cpu_op_lda_imm_zero() 
    {
        let (mut cpu,mut ram) = new_cpu(&vec![0xa9,0x00]);
        
        run_instr(&mut cpu,&mut ram, 1);

        assert_eq!(cpu.regs.ac, 0x00);
        assert_eq!(cpu.regs.p.zero, true);
        assert_eq!(cpu.regs.p.negative, false);
    }

    #[test]
    fn cpu_op_lda_imm_negative() 
    {
        let (mut cpu,mut ram) = new_cpu(&vec![0xa9,0xFF]);
        
        run_instr(&mut cpu,&mut ram, 1);

        assert_eq!(cpu.regs.ac, 0xFF);
        assert_eq!(cpu.regs.p.zero, false);
        assert_eq!(cpu.regs.p.negative, true);
    }

    #[test]
    fn cpu_op_lda_abs() 
    {
        let addr = 0x0603;
        let (mut cpu,mut ram) = new_cpu(&vec![0xad, addr.lo(), addr.hi(), 0x05]);
        
        run_instr(&mut cpu,&mut ram, 1);

        assert_eq!(cpu.regs.ac, 0x05);
        assert_eq!(cpu.regs.p.zero, false);
        assert_eq!(cpu.regs.p.negative, false);
    }

    #[test]
    fn cpu_op_tax() 
    {
        let (mut cpu,mut ram) = new_cpu(&vec![ 
            0xa9,0x42,  // LDA 2
            0xaa        // TAX 2
            ]);
        
        run_instr(&mut cpu,&mut ram, 2);

        assert_eq!(cpu.regs.x, 0x42);
        assert_eq!(cpu.regs.p.zero, false);
        assert_eq!(cpu.regs.p.negative, false);
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

        let (mut cpu,mut ram) = new_cpu(&prog.to_vec());
        
        run_instr(&mut cpu,&mut ram, 3);   

        assert_eq!(cpu.regs.ac, 0x66);
        assert_eq!(cpu.regs.p.zero, false);
        assert_eq!(cpu.regs.p.negative, false);
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

        let (mut cpu,mut ram) = new_cpu(&prog.to_vec());
        
        run_instr(&mut cpu,&mut ram, 3); // still only 3 instructions
        
        assert_eq!(cpu.regs.ac, 0x66);
        assert_eq!(cpu.regs.p.carry, false);
        assert_eq!(cpu.regs.p.zero, false);
        assert_eq!(cpu.regs.p.negative, false);
    }

    
    fn run_until_break(cpu: &mut Cpu, ram: &mut Ram) {
        while ram.read(cpu.regs.pc) != 0x00
        {
            cpu.ir_cycles = 0;  // don't care about cycle accuracy here
            cpu.clock(ram);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa9, 0x01, 0x8d, 0x00, 0x02, 0xa9, 0x05, 0x8d, 0x01, 0x02, 0xa9, 0x08, 0x8d, 0x02, 0x02 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x08);
        assert_eq!(cpu.regs.x, 0x00);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x60F);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-Bdizc"));

        assert_eq!(ram.read(0x200), 0x01);
        assert_eq!(ram.read(0x201), 0x05);
        assert_eq!(ram.read(0x202), 0x08);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa9,0xc0,0xaa,0xe8,0x69,0xc4 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x84);
        assert_eq!(cpu.regs.x, 0xc1);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x606);
        assert_eq!(cpu.regs.p, StatusRegister::from("Nv-BdizC"));
    }
    
    #[test]
    fn ez_instructions2()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 80     LDA #$80
        // $0602    85 01     STA $01
        // $0604    65 01     ADC $01
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa9,0x80,0x85,0x01,0x65,0x01 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x00);
        assert_eq!(cpu.regs.x, 0x00);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x606);
        assert_eq!(cpu.regs.p, StatusRegister::from("nV-BdiZC"));
        
        assert_eq!(ram.read(0x01), 0x80);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa2,0x08,0xca,0x8e,0x00,0x02,0xe0,0x03,0xd0,0xf8,0x8e,0x01,0x02 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x00);
        assert_eq!(cpu.regs.x, 0x03);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x60d);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-BdiZC"));
        
        assert_eq!(ram.read(0x200), 0x03);
        assert_eq!(ram.read(0x201), 0x03);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa9,0x01,0x85,0xf0,0xa9,0xcc,0x85,0xf1,0x6c,0xf0,0x00 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0xcc);
        assert_eq!(cpu.regs.x, 0x00);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0xcc01);
        assert_eq!(cpu.regs.p, StatusRegister::from("Nv-Bdizc"));
        
        assert_eq!(ram.read(0x00f0), 0x01);
        assert_eq!(ram.read(0x00f1), 0xcc);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa2,0x01,0xa9,0x05,0x85,0x01,0xa9,0x07,0x85,0x02,0xa0,0x0a,0x8c,0x05,0x07,0xa1 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x0a);
        assert_eq!(cpu.regs.x, 0x01);
        assert_eq!(cpu.regs.y, 0x0a);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x0611);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-Bdizc"));
        
        assert_eq!(ram.read(0x0001), 0x05);
        assert_eq!(ram.read(0x0002), 0x07);
        assert_eq!(ram.read(0x0705), 0x0a);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa0,0x01,0xa9,0x03,0x85,0x01,0xa9,0x07,0x85,0x02,0xa2,0x0a,0x8e,0x04,0x07,0xb1 
            ,0x01 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x0a);
        assert_eq!(cpu.regs.x, 0x0a);
        assert_eq!(cpu.regs.y, 0x01);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x0611);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-Bdizc"));
        
        assert_eq!(ram.read(0x0001), 0x03);
        assert_eq!(ram.read(0x0002), 0x07);
        assert_eq!(ram.read(0x0704), 0x0a);
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa2,0x00,0xa0,0x00,0x8a,0x99,0x00,0x02,0x48,0xe8,0xc8,0xc0,0x10,0xd0,0xf5,0x68 
            ,0x99,0x00,0x02,0xc8,0xc0,0x20,0xd0,0xf7 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x00);
        assert_eq!(cpu.regs.x, 0x10);
        assert_eq!(cpu.regs.y, 0x20);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x0618);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-BdiZC"));
        
        for i in 0..16
        {
            assert_eq!(ram.read(0x0200 + i), i as u8);
        }

        for i in 0..16
        {
            assert_eq!(ram.read(0x0210 + i), 15 - i as u8)
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
        let (mut cpu,mut ram) = new_cpu(&vec![ 0x20,0x09,0x06,0x20,0x0c,0x06,0x20,0x12,0x06,0xa2,0x00,0x60,0xe8,0xe0,0x05,0xd0 
            ,0xfb,0x60,0x00  ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x00);
        assert_eq!(cpu.regs.x, 0x05);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xfd);
        assert_eq!(cpu.regs.pc, 0x0612);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-BdiZC"));
    }

    #[test]
    fn ez_cmp()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 01     LDA #$01
        // $0602    85 02     STA $02
        // $0604    a9 77     LDA #$77
        // $0606    c9 77     CMP #$77
        // $0608    a9 04     LDA #$04
        // $060a    24 02     BIT $02
        // $060c    d0 06     BNE $0614
        // $060e    a9 01     LDA #$01
        // $0610    85 02     STA $02
        // $0612    00        BRK 
        // $0613    00        BRK 
        // $0614    00        BRK 
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa9,0x01,0x85,0x02,0xa9,0x77,0xc9,0x77,0xa9,0x04,0x24,0x02,
                                     0xd0,0x06,0xa9,0x01,0x85,0x02,0x00,0x00,0x00 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x01);
        assert_eq!(cpu.regs.x, 0x00);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x0612);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-BdizC"));

        assert_eq!(ram.read(0x0002), 0x01);
        
    }

    
    #[test]
    fn ez_beq_relative_addressing()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a2 01     LDX #$01
        // $0602    e8        INX 
        // $0603    e0 02     CPX #$02
        // $0605    f0 fb     BEQ $0602
        // $0607    e0 03     CPX #$03
        // $0609    f0 05     BEQ $0610
        // $060b    00        BRK 
        // $060c    00        BRK 
        // $060d    00        BRK 
        // $060e    00        BRK 
        // $060f    00        BRK 
        // $0610    a9 42     LDA #$42 
        let (mut cpu,mut ram) = new_cpu(&vec![ 0xa2,0x01,0xe8,0xe0,0x02,0xf0,0xfb,0xe0,0x03,0xf0,0x05,0x00,
                                     0x00,0x00,0x00,0x00,0xa9,0x42 ]);
        
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x42);
        assert_eq!(cpu.regs.x, 0x03);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x0612);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-BdizC"));
    }

    #[test]
    fn ez_snake_dir()
    {
        // Address  Hexdump   Dissassembly
        // -------------------------------
        // $0600    a9 02     LDA #$02
        // $0602    85 02     STA $02
        // $0604    a9 77     LDA #$77
        // $0606    85 ff     STA $ff
        // $0608    20 0c 06  JSR $060c
        // $060b    00        BRK 
        // $060c    a5 ff     LDA $ff
        // $060e    c9 77     CMP #$77
        // $0610    f0 0d     BEQ $061f
        // $0612    c9 64     CMP #$64
        // $0614    f0 14     BEQ $062a
        // $0616    c9 73     CMP #$73
        // $0618    f0 1b     BEQ $0635
        // $061a    c9 61     CMP #$61
        // $061c    f0 22     BEQ $0640
        // $061e    60        RTS 
        // $061f    a9 04     LDA #$04
        // $0621    24 02     BIT $02
        // $0623    d0 26     BNE $064b
        // $0625    a9 01     LDA #$01
        // $0627    85 02     STA $02
        // $0629    60        RTS 
        // $062a    a9 08     LDA #$08
        // $062c    24 02     BIT $02
        // $062e    d0 1b     BNE $064b
        // $0630    a9 02     LDA #$02
        // $0632    85 02     STA $02
        // $0634    60        RTS 
        // $0635    a9 01     LDA #$01
        // $0637    24 02     BIT $02
        // $0639    d0 10     BNE $064b
        // $063b    a9 04     LDA #$04
        // $063d    85 02     STA $02
        // $063f    60        RTS 
        // $0640    a9 02     LDA #$02
        // $0642    24 02     BIT $02
        // $0644    d0 05     BNE $064b
        // $0646    a9 08     LDA #$08
        // $0648    85 02     STA $02
        // $064a    60        RTS 
        // $064b    60        RTS 
        let (mut cpu,mut ram) = new_cpu(&vec![ 
            0xa9,0x02,0x85,0x02,0xa9,0x77,0x85,0xff,0x20,0x0c,0x06,0x00,0xa5,0xff,0xc9,0x77, 
            0xf0,0x0d,0xc9,0x64,0xf0,0x14,0xc9,0x73,0xf0,0x1b,0xc9,0x61,0xf0,0x22,0x60,0xa9, 
            0x04,0x24,0x02,0xd0,0x26,0xa9,0x01,0x85,0x02,0x60,0xa9,0x08,0x24,0x02,0xd0,0x1b, 
            0xa9,0x02,0x85,0x02,0x60,0xa9,0x01,0x24,0x02,0xd0,0x10,0xa9,0x04,0x85,0x02,0x60, 
            0xa9,0x02,0x24,0x02,0xd0,0x05,0xa9,0x08,0x85,0x02,0x60,0x60,0x0
            ]);

    
        run_until_break(&mut cpu,&mut ram);

        assert_eq!(cpu.regs.ac, 0x01);
        assert_eq!(cpu.regs.x, 0x00);
        assert_eq!(cpu.regs.y, 0x00);
        assert_eq!(cpu.regs.sp, 0xff);
        assert_eq!(cpu.regs.pc, 0x060b);
        assert_eq!(cpu.regs.p, StatusRegister::from("nv-BdizC"));

        assert_eq!(ram.read(0x0002), 0x01);
        assert_eq!(ram.read(0x00FF), 0x77);
    }
}