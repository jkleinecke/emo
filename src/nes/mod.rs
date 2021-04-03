#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::mos6502::{Cpu6502,PC_START,Memory};
use crate::common::Clocked;
use std::cell::RefCell;
use std::rc::Rc;

const SYSMEMSIZE: usize = 0xFFFF;
pub const PROGRAM_START: u16 = 0x8000;

pub struct Nes {
    pub sys_clocks:u64,
    pub cpu:Cpu6502,
    pub bus:Rc<RefCell<Bus>>,
}

pub struct Bus {
    pub cpu_ram: [u8;0x800],
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


impl Bus {
    pub fn new() -> Self {
        Bus {
            cpu_ram: [0;0x800],
        }
    }

    pub fn load(&mut self, ptr: u16) -> u8
    {
        return self.cpu_ram[ptr as usize];
    }

    pub fn store(&mut self, ptr: u16, value: u8)
    {
        self.cpu_ram[ptr as usize] = value;
    }

    pub fn write_ram(&mut self, offset: usize, data: &Vec<u8>)
    {
        self.cpu_ram[offset .. (offset + data.len())].copy_from_slice(&data[..]);
    }

    pub fn read_ram(&mut self, offset: usize, size: usize) -> Vec<u8>
    {
        self.cpu_ram[offset .. (offset + size)].to_vec()
    }
}

const RAM_ADDR: u16 = 0x0000;
const RAM_ADDR_END: u16 = 0x1FFF;

impl Memory for Bus {

    fn read(&mut self, addr:u16) -> u8 {
        match addr {
            RAM_ADDR ..= RAM_ADDR_END => self.cpu_ram[(addr & 0x7FF) as usize],
            _ => {
                println!("BAD READ ACCESS: Ignoring bad bus access at {:#06X}", addr);
                0
            }
        }
    }

    fn write(&mut self, addr:u16, value:u8) {
        match addr {
            RAM_ADDR ..= RAM_ADDR_END => self.cpu_ram[(addr & 0x7FF) as usize] = value,
            _ => {
                println!("BAD WRITE ACCESS: Ignoring bad bus access at {:#06X}", addr);
            }
        }
        self.store(addr, value)
    }
}
