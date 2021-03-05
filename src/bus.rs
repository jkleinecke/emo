#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

use sdl2::pixels::Color;

fn color(byte: u8) -> Color {
    match byte {
        0 => sdl2::pixels::Color::BLACK,
        1 => sdl2::pixels::Color::WHITE,
        2 | 9 => sdl2::pixels::Color::GREY,
        3 | 10 => sdl2::pixels::Color::RED,
        4 | 11 => sdl2::pixels::Color::GREEN,
        5 | 12 => sdl2::pixels::Color::BLUE,
        6 | 13 => sdl2::pixels::Color::MAGENTA,
        7 | 14 => sdl2::pixels::Color::YELLOW,
        _ => sdl2::pixels::Color::CYAN,
    }
}

pub struct Bus {
    pub vram_dirty: bool,
    pub vram: [u8; 32 * 32 * 3],    // 24-bit
    pub cpu_ram: [u8;0xFFFF],
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            vram_dirty: false,
            vram: [0;32 * 32 * 3],
            cpu_ram: [0;0xFFFF],
        }
    }

    pub fn init_vram(&mut self)
    {
        self.vram_dirty = true;

        let mut frame_idx = 0;
        for i in 0x0200..0x600 {
            let color_idx = self.cpu_ram[i as usize];
            let (b1, b2, b3) = color(color_idx).rgb();
           
            self.vram[frame_idx] = b1;
            self.vram[frame_idx + 1] = b2;
            self.vram[frame_idx + 2] = b3;
        
            frame_idx += 3;
        }
    }

    pub fn load(&mut self, ptr: u16) -> u8
    {
        return self.cpu_ram[ptr as usize];
    }

    pub fn store(&mut self, ptr: u16, value: u8)
    {
        if !self.vram_dirty && ptr <= 0x200 && ptr >= 0x600
        {
            // only dirty if the value is different than what is already there
            self.vram_dirty = self.cpu_ram[ptr as usize] != value;

            let (b1,b2,b3) = color(value).rgb();
            let pixel_idx = ((ptr - 0x200) * 3) as usize;
            self.vram[pixel_idx + 0] = b1;
            self.vram[pixel_idx + 1] = b2;
            self.vram[pixel_idx + 2] = b3;
        }

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