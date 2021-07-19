use super::{
    WORD,Word,
    Byte,BitTest,
    Cpu,CpuContext,
    StatusRegister,
    MemoryMapped,
    DecodedInstruction,
    AddressingMode,
    fetch_operand_address,
    OPCODE_TABLE,
    ADDR_OPSIZE_TABLE,
};


pub fn trace(cpu:&mut Cpu, memory:&mut dyn MemoryMapped) -> String {

    let ir = memory.read(cpu.regs.pc);
    let op1 = memory.read(cpu.regs.pc + 1);
    let op2 = memory.read(cpu.regs.pc + 2);

    let (operation, addr_mode, c1, c2, exec_op, illegal) = OPCODE_TABLE[ir as usize];

    let instr = DecodedInstruction {
        ir: ir,
        ir_address: cpu.regs.pc,
        opcode: operation,
        addr_mode: addr_mode,
        opsize: ADDR_OPSIZE_TABLE[addr_mode as usize],
        operand: [op1, op2],
        cycles: c1,
        oops: ternary!(c2 == 0, false, true),    
    };

    let tmp = match instr.opsize {
        1 => match ir {
            0x0A | 0x4A | 0x2A | 0x6A => format!("A "),
            _ => String::from(""),
        }
        2 => {
            let mut context = CpuContext::new(cpu, memory);
            context.instruction = &instr;
            let address = fetch_operand_address(&mut context);
            let value = context.memory.read(address);

            match instr.addr_mode {
                AddressingMode::Immediate => format!("#${:02x}", op1),
                AddressingMode::ZeroPage => format!("${:02x} = {:02x}", address, value),
                AddressingMode::ZeroPageX => format!("${:02x},X @ {:02x} = {:02x}", op1, address, value),
                AddressingMode::ZeroPageY => format!("${:02x},Y @ {:02x} = {:02x}", op1, address, value),
                AddressingMode::IndirectX => format!("(${:02x},X) @ {:02x} = {:04x} = {:02x}", op1, op1.wrapping_add(cpu.regs.x), address, value),
                AddressingMode::IndirectY => format!("(${:02x}),Y = {:04x} @ {:04x} = {:02x}", op1, address.wrapping_sub(cpu.regs.y as u16), address, value),
                AddressingMode::Relative => format!("${:04x}", address),
                _ => panic!("unexpected addressing mode {:?} has ops-len 2. code {:02x}", instr.addr_mode, ir),
            }
        }
        3 => {
            let mut context = CpuContext::new(cpu, memory);
            context.instruction = &instr;
            let op16 = Word::make(op2,op1);
            let address = fetch_operand_address(&mut context);
            let value = context.memory.read(address);
            
            match instr.addr_mode {
                AddressingMode::Indirect => format!("(${:04x}) = {:04x}", op16, address),                    
                AddressingMode::Absolute => {
                    match ir {
                        0x4C | 0x20 | 0x40 => format!("${:04x}", address),
                        _ => format!("${:04x} = {:02x}", address, value),
                    }
                },
                AddressingMode::AbsoluteX => format!("${:04x},X @ {:04x} = {:02x}", op16, address, value),
                AddressingMode::AbsoluteY => format!("${:04x},Y @ {:04x} = {:02x}", op16, address, value),
                _ => panic!("unexpected addressing mode {:?} has ops-len 3. code {:02x}", instr.addr_mode, ir),
            }
        },
        _ => String::from(""),
    };

    let hex = match instr.opsize {
        0 | 1 => format!("{:02x}      ", ir),
        2 => format!("{:02x} {:02x}   ", ir, op1),
        3 => format!("{:02x} {:02x} {:02x}", ir, op1, op2),
        _ => panic!("bad instruction opsize {:02x}", ir),
    };

    //--TODO: Fix formatting issue with the opcode...
    let asm_str = format!("{:04x}  {:8} {}{: >4} {}", instr.ir_address, hex, ternary!(illegal>0,"*"," "),instr.opcode, tmp).trim().to_string();

    format!("{:47} A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x}", asm_str, cpu.regs.ac, cpu.regs.x, cpu.regs.y, cpu.regs.p.to_byte(), cpu.regs.sp)
        .to_ascii_uppercase()
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::mos6502::{Ram,Clocked};

    #[test]
    fn test_format_trace() {
        let mut ram = Ram::new();
        ram.write(100, 0xa2);
        ram.write(101, 0x01);
        ram.write(102, 0xca);
        ram.write(103, 0x88);
        ram.write(104, 0x00);

        let mut cpu = Cpu::new();
        cpu.reset = true;
        cpu.clock(&mut ram);
        cpu.ir_cycles = 0;
        
        cpu.regs.pc = 0x64;
        cpu.regs.ac = 1;
        cpu.regs.x = 2;
        cpu.regs.y = 3;
        
        let t1 = trace(&mut cpu, &mut ram);
        assert_eq!(
            "0064  A2 01     LDX #$01                        A:01 X:02 Y:03 P:24 SP:FD",
            t1
        );

        cpu.ir_cycles = 0;
        cpu.clock(&mut ram);
        let t2 = trace(&mut cpu, &mut ram);
        assert_eq!(
            "0066  CA        DEX                             A:01 X:01 Y:03 P:24 SP:FD",
            t2
        );
        
        cpu.ir_cycles = 0;
        cpu.clock(&mut ram);
        let t3 = trace(&mut cpu, &mut ram);
        assert_eq!(
            "0067  88        DEY                             A:01 X:00 Y:03 P:26 SP:FD",
            t3
        );
    }

    #[test]
    fn test_format_mem_access() {
        let mut ram = Ram::new();
        // ORA ($33), Y
        ram.write(100, 0x11);
        ram.write(101, 0x33);

        //data
        ram.write(0x33, 00);
        ram.write(0x34, 04);

        //target cell
        ram.write(0x400, 0xAA);

        let mut cpu = Cpu::new();
        cpu.reset = true;
        cpu.clock(&mut ram);
        cpu.ir_cycles = 0;
        cpu.regs.pc = 0x64;
        cpu.regs.y = 0;
       
        let t1 = trace(&mut cpu, &mut ram);
        assert_eq!(
            "0064  11 33     ORA ($33),Y = 0400 @ 0400 = AA  A:00 X:00 Y:00 P:24 SP:FD",
            t1
        );
    }
}