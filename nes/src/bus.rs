use super::{
    Byte,
    Word,
    WORD,
    Rom,
    MemoryMapped
};


pub struct Bus<'a> {
    pub cpu_ram: [u8;0x800],
    pub cartridge: &'a Rom,
}

impl<'a> Bus<'a> {
    pub fn from_cartridge(cartridge:&'a Rom) -> Self {
        Self {
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

impl<'a> MemoryMapped for Bus<'a> {

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