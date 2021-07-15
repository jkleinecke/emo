#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use crate::mos6502::{Cpu,PC_START,MemoryMapped,State};
use crate::common::Clocked;
use crate::cartridge::Rom;

pub struct Nes {
    pub sys_clocks:u64,
    pub cpu:Cpu,
    pub bus:Bus,
}

pub struct Bus {
    pub cpu_ram: [u8;0x800],
    pub cartridge: &'static Rom,
}

impl Nes
{
    pub fn from_cartridge(cartridge: &'static Rom) -> Self 
    {
        let bus = Bus::new(cartridge);
        let cpu = Cpu::new();

        let mut nes = Nes {
            sys_clocks: 0,
            cpu: cpu,
            bus: bus,
        };

        nes.reset();

        nes
    }

    pub fn reset(&mut self)
    {
        //self.cpu.reset();
    }

    pub fn run(&mut self)
    {
        self.run_with_callback( |nes| nes.cpu.did_halt() );                
    }

    pub fn run_with_callback<CB>(&mut self, callback: CB)
        where CB: Fn(&Nes) -> bool
    {
        loop
        {
            self.clock();

            if self.cpu.ir_cycles == 0 
            {
                if (callback)(&self)
                {
                    break;
                }
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
        self.cpu.clock(&mut self.bus);
        self.sys_clocks = self.sys_clocks.wrapping_add(1);
    }
}


impl Bus {
    pub fn new(cartridge: &'static Rom) -> Self {
        Bus {
            cpu_ram: [0;0x800],
            cartridge,
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
const PPU_ADDR: u16 = 0x2000;
const PPU_ADDR_END: u16 = 0x3FFF;
const CARTRIDGE_ADDR: u16 = 0x8000;
const CARTRIDGE_ADDR_END: u16 = 0xFFFF;

impl MemoryMapped for Bus {

    fn read(&self, addr:u16) -> u8 {
        match addr {
            RAM_ADDR ..= RAM_ADDR_END => {
                // cpu ram is 13 bits in total address size, but actual hardware only
                // connects 11 bits, so we mimic that by only using 11 bits ()
                let mirror_addr = addr & 0x7FF;
                self.cpu_ram[mirror_addr as usize]
            },
            CARTRIDGE_ADDR ..= CARTRIDGE_ADDR_END => {
                // translate to the cart address by masking off
                // the 16th bit
                let cart_addr = addr & 0x7FFF;
                self.cartridge.read(cart_addr)
            },
            // PPU registers
            PPU_ADDR ..= PPU_ADDR_END => {
                let mirror_addr = addr & 0x2007;
                todo!("PPU not supported yet")
            }
            _ => {
                println!("BAD READ ACCESS: Ignoring bad bus access at {:#06X}", addr);
                0
            }
        }
    }

    fn write(&mut self, addr:u16, value:u8) {
        match addr {
            RAM_ADDR ..= RAM_ADDR_END => self.cpu_ram[(addr & 0x7FF) as usize] = value,
            // PPU registers
            PPU_ADDR ..= PPU_ADDR_END => {
                let mirror_addr = addr & 0x2007;
                todo!("PPU not supported yet")
            },
            CARTRIDGE_ADDR ..= CARTRIDGE_ADDR_END => {
                panic!("BAD WRITE ACCESS: Cannot write to ROM cartridge");                
            }
            _ => {
                println!("BAD WRITE ACCESS: Ignoring bad bus access at {:#06X}", addr);
            }
        }
        self.store(addr, value)
    }
}
