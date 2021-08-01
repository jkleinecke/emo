use super::{
    Word,WORD,Byte,Bit,BitTest,ternary,
    MemoryMapped,
    DecodedInstruction,
    Registers,StatusRegister,
    Cpu
};

const STACK_BASE: Word = 0x100;

pub struct CpuContext<'a> {
    pub regs: &'a mut Registers,
    pub memory: &'a mut dyn MemoryMapped,
    pub instruction: &'a mut DecodedInstruction,
    pub reset: Bit,
    pub nmi: Bit,
    pub irq: Bit,
    pub halt: bool,
    pub increment_programcounter: bool,
}

impl<'a> CpuContext<'a> {
    pub fn new(regs: &'a mut Registers, instruction: &'a mut DecodedInstruction, reset: bool, nmi: bool, irq: bool, memory: &'a mut dyn MemoryMapped) -> Self {
        Self {
            regs,
            memory,
            instruction,
            reset,
            nmi,
            irq,
            halt: false,
            increment_programcounter: true,
        }
    }

    //====================
    // I/O Memory Bus
    //====================
    
    pub fn jump_pc(&mut self, addr: Word) {
        self.increment_programcounter = false;
        self.regs.pc = addr;
    }

    pub fn stack_push(&mut self, v:Byte) 
    {
        let addr = Word::make(0x01,self.regs.sp);    // calc the stack pointer address
        self.regs.sp = self.regs.sp.wrapping_sub(1);     // decrement the stack pointer
        
        if self.reset == false {
            // writes are disabled during a reset
            self.memory.write(addr, v) ;               // queue up the bus write
        }
    }

    pub fn stack_pop(&mut self) -> Byte
    {
        self.regs.sp = self.regs.sp.wrapping_add(1);     // increment the stack pointer
        let addr = Word::make(0x01,self.regs.sp);    // calc the stack pointer address
        self.memory.read(addr)                   // bus read        
    }

    // Status Flags

    pub fn update_status(&mut self, result:Byte)
    {
        self.regs.p.zero = result == 0;
        self.regs.p.negative = result.bit(7);
    }

    
    /*************************
     * ALU operations
     *************************/

     pub fn alu_sums(&mut self, a:Byte, b:Byte) -> Byte
     {
         // a rather ambitious attempt to generalize the alu operations with
         // the status flag updates
         //
         // Important to note that the result does not actually wrap around.
         //  A second add operation is required to account for the carry bit
         //
         // CPU status flags are then set to indicate
         //  zero, 
         //  negative, 
         //  another carry was generated
         //  signed overflow occurred if the values are interpreted as signed
 
         let result = a as Word + b as Word + self.regs.p.carry as Word;
         let signed_overflow = !(a ^ b) & (a ^ result.lo());
 
         self.regs.p.carry = result.hi() > 0;
         self.regs.p.overflow = signed_overflow.bit(7);
 
         result.lo()
     }
 
     pub fn alu_lsr(&mut self, a:Byte) -> Byte
     {
         self.regs.p.carry = a.bit(0);
 
         a >> 1
     }
 
     pub fn alu_asl(&mut self, a:Byte) -> Byte
     {
         self.regs.p.carry = a.bit(7);
 
         a << 1
     }
 
     pub fn alu_rol(&mut self, a:Byte) -> Byte
     {
         let carry = self.regs.p.carry ;
         self.regs.p.carry = a.bit(7);
 
         (a << 1).wrapping_add(ternary!(carry,1,0))
     }
 
     pub fn alu_ror(&mut self, a:Byte) -> Byte
     {
         let carry = self.regs.p.carry ;
 
         self.regs.p.carry = a.bit(0);
 
         (a >> 1).wrapping_add(ternary!(carry,0x80,0))
     }
 
     pub fn alu_compare(&mut self, v1: Byte, v2: Byte)
     {
         self.regs.p.negative = v1.wrapping_sub(v2).bit(7);
         self.regs.p.zero = v1 == v2;
         self.regs.p.carry = v1 >= v2;
     }
}


#[cfg(test)]
mod test {
    use super::*;
    use crate::memory::Ram;
    

    #[test]
    fn cpu_alu_add() 
    {
        let mut ram = Ram::new();
        let mut cpu = Cpu::new();
        let mut cpuctx = CpuContext::new(&mut cpu.regs, &mut cpu.instr, false, false, false, &mut ram);   
        
        let result = cpuctx.alu_sums(13, 211);
        
        assert_eq!(result, 224);
        assert_eq!(cpuctx.regs.p.carry, false);
    }
    
    #[test]
    fn cpu_alu_sub() 
    {
        let mut ram = Ram::new();
        let mut cpu = Cpu::new();
        let mut cpuctx = CpuContext::new(&mut cpu.regs, &mut cpu.instr, false, false, false, &mut ram);   
        cpuctx.regs.p.carry = true;

        let result = cpuctx.alu_sums(211, !10);
        
        assert_eq!(result, 201);
        assert_eq!(cpuctx.regs.p, StatusRegister::from("nv-bdIzC"));
    }

    #[test]
    fn cpu_alu_add_carry() 
    {
        let mut ram = Ram::new();
        let mut cpu = Cpu::new();
        let mut cpuctx = CpuContext::new(&mut cpu.regs, &mut cpu.instr, false, false, false, &mut ram);   
        
        let result = cpuctx.alu_sums(254, 6);
        
        assert_eq!(result, 4);
        assert_eq!(cpuctx.regs.p.carry, true);
    }

    #[test]
    fn cpu_alu_sub_carry() 
    {
        let mut ram = Ram::new();
        let mut cpu = Cpu::new();
        let mut cpuctx = CpuContext::new(&mut cpu.regs, &mut cpu.instr, false, false, false, &mut ram);   
        cpuctx.regs.p.carry = true;

        let result = cpuctx.alu_sums(10, !20);
        
        assert_eq!(result as i8, -10);
        assert_eq!(cpuctx.regs.p, StatusRegister::from("nv-BdIzc"));
    }

    #[test]
    fn cpu_alu_add_carry_clear() 
    {
        let mut ram = Ram::new();
        let mut cpu = Cpu::new();
        let mut cpuctx = CpuContext::new(&mut cpu.regs, &mut cpu.instr, false, false, false, &mut ram);   
        
        let temp = cpuctx.alu_sums(254,6); // 4 + carry
        let result = cpuctx.alu_sums(temp, 6); 
        
        assert_eq!(result, 11);
        assert_eq!(cpuctx.regs.p.carry, false);
    }
}