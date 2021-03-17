#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::mos6502::{Cpu6502,PC_START,Memory};
use crate::common::Clocked;
use crate::bus::{Bus};
use std::cell::RefCell;
use std::rc::Rc;

const SYSMEMSIZE: usize = 0xFFFF;
pub const PROGRAM_START: u16 = 0x8000;

pub struct Nes {
    pub sys_clocks:u64,
    pub cpu:Cpu6502,
    pub bus:Rc<RefCell<Bus>>,
}


impl Memory for Bus {
    fn read(&mut self, addr:u16) -> u8 {
        self.load(addr)
    }

    fn write(&mut self, addr:u16, value:u8) {
        self.store(addr, value)
    }
}

impl Nes
{
    pub fn new() -> Self 
    {
        let bus = Rc::new(RefCell::new(Bus::new()));
        let cpu = Cpu6502::new(bus.clone());

        Nes {
            sys_clocks: 0,
            cpu: cpu,
            bus: bus.clone(),
        }
    }

    pub fn reset(&mut self)
    {
        self.cpu.reset();
    }

    pub fn run(&mut self)
    {
        loop
        {
            self.step_instruction();

            if self.cpu.did_halt() 
            {
                break;
            }
        }
    }

    pub fn step_instruction(&mut self)
    {
        loop 
        {
            self.clock();

            if self.cpu.ir_cycles == 0
            {
                break;
            }
        }
    }

    pub fn clock(&mut self)
    {
        // CPU clock also gets a data bus clock
        if self.cpu.did_halt() == false 
        {
            self.cpu.clock();
        }

        self.sys_clocks += 1;
    }

    // TODO: this will eventually become load_cartridge
    pub fn load_program(&mut self, program: &Vec<u8>) 
    {
        // TODO: bounds check?
        self.bus.borrow_mut().write_ram(PROGRAM_START as usize, program);

        // Now tell the cpu where the program starts
        let start_addr = PROGRAM_START.to_le_bytes();
        self.bus.borrow_mut().write_ram(PC_START as usize, &start_addr.to_vec());

        self.reset();
    }
}

