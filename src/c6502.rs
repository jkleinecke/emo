#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

#[derive(Debug,Copy,Clone)]
pub struct StatusFlags {
    pub c:bool,
    pub z:bool,
    pub i:bool,
    pub d:bool,
    pub b:bool,
    pub v:bool,
    pub n:bool,
}

pub union StatusRegister {
    pub flags: StatusFlags,
    pub register: u8,
}

pub struct EmoC6502 {
    pub a: u8,                      // Accumulator Register
    pub x: u8,                      // X Register
    pub y: u8,                      // Y Register
    pub sp: u8,                     // Stack Pointer
    pub pc: u16,                    // Program Counter
    pub status: StatusRegister,     // Status Register

}

impl EmoC6502 {
    pub fn new() -> Self {
        EmoC6502 {
            a: 0,
            x: 0,
            y: 0,
            sp: 0,
            pc: 0,
            status: StatusRegister{register:0},
        }
    }

    pub fn interpret(&mut self, program: Vec<u8>) {
        self.pc = 0;

        //self.S = StatusFlags::C | StatusFlags::Z;

        // loop {
        //     let opscode = program[self.PC as usize];
        //     self.PC += 1;

        //     match opscode {
        //         0xA9 => {
        //             let param = program[self.PC as usize];
        //             self.PC += 1;
        //             self.A = param;

        //             if self.A == 0 {
        //                 self.status = self.status | 0b0000_0010;
        //             }
        //             else {
        //                 self.status = self.status & 0b1111_1101;
        //             }
        //         }
        //         _ => todo!()
        //     }
        // }
    }
}