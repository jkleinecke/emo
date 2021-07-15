
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::{Texture};
use sdl2::pixels::{Color};

use rand::Rng;
use rand::prelude::ThreadRng;

use crate::common::{Clocked,Word,Byte};
use crate::mos6502::*;
use crate::cartridge::Rom;


fn color(byte: Byte) -> Color {
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

#[derive(Debug,PartialEq,Copy,Clone)]
pub enum RunMode {
    Run,
    StepInstruction,
}

pub struct SnakeGame<'a> {
    rng: ThreadRng,
    cpu: Cpu,
    memory: SnakeMemory,
    event_pump: &'a mut sdl2::EventPump,
    canvas: &'a mut sdl2::render::Canvas<sdl2::video::Window>,
    texture: Texture<'a>,
    pub run_mode: RunMode,
    vram: [Byte;32*32*3],
}


struct SnakeMemory {
    ram: [Byte;0x800],
    rom: Rom,
}

impl MemoryMapped for SnakeMemory {
    fn read(&self, addr:Word) -> Byte {
        match addr
        {
            0x8000 ..= 0xFFFF => self.rom.read(addr & 0x7FFF),
            0x0000 ..= 0x1FFF => self.ram[(addr & 0x7FF) as usize],
            _ => 0
        }
    }
    fn write(&mut self, addr:Word, value:Byte) {
        match addr
        {
            0x0000..= 0x1FFF => { 
                self.ram[(addr & 0x7FF) as usize] = value
            },
            _ => { panic!("Invalid write") },
        };
    }
}

impl SnakeMemory {
    pub fn new(rom_bytes: &Vec<u8>) -> Self {
        SnakeMemory {
            ram: [0;0x800],  
            rom: Rom::new(rom_bytes).unwrap(),   
        }
    }
}

impl<'a> SnakeGame<'a> {
    pub fn new(rom_bytes: &Vec<u8>, event_pump: &'a mut sdl2::EventPump, canvas: &'a mut sdl2::render::Canvas<sdl2::video::Window>, texture: sdl2::render::Texture<'a>) -> Self {
        let memory = SnakeMemory::new(rom_bytes);
        let cpu = Cpu::new();

        SnakeGame {
            memory,
            cpu,
            rng: rand::thread_rng(),
            event_pump,
            canvas,
            texture,
            run_mode: RunMode::Run,
            vram: [0;32*32*3],
        }
    }
    

    pub fn init(&mut self) 
    {
        self.cpu.reset = true;

        self.vram = [0;32*32*3];
        self.texture.update(None, &self.vram, 32 * 3).unwrap();
        self.canvas.copy(&self.texture, None, None).unwrap();

        let _ = self.canvas.window_mut().set_title("Snake Game - Space to Step CPU");

        println!("Address  Hex Dump  Disassembly");
        println!("------------------------------");
    }

    pub fn update_once(&mut self) 
    {
        let mut should_step_cpu = match self.run_mode {
            RunMode::Run => true,
            RunMode::StepInstruction => false,
        };

        {            
            // update the random memory generator location
            self.memory.write(0x00FE, self.rng.gen_range(1,16) as u8);

            // handle user input
            for event in self.event_pump.poll_iter()
            {
                match event
                {
                    Event::Quit {..} | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                        std::process::exit(0);
                    },
                    Event::KeyDown { keycode: Some(Keycode::W), .. } => {
                        self.memory.write(0x00FF, 0x77);
                    }
                    Event::KeyDown { keycode: Some(Keycode::S), .. } => {
                        self.memory.write(0x00FF, 0x73);
                        
                    }
                    Event::KeyDown { keycode: Some(Keycode::A), .. } => {
                        self.memory.write(0x00FF, 0x61);
                    }
                    Event::KeyDown { keycode: Some(Keycode::D), .. } => {
                        self.memory.write(0x00FF, 0x64);
                    }
                    Event::KeyDown { keycode: Some(Keycode::Space), .. } => {
                        self.run_mode = RunMode::StepInstruction;
                        let _ = self.canvas.window_mut().set_title("Snake Game - Space to Step CPU");
                        should_step_cpu = true;
                    }
                    Event::KeyDown { keycode: Some(Keycode::F5), .. } => {
                        let _ = self.canvas.window_mut().set_title("Snake Game");
                        self.run_mode = RunMode::Run;
                    }
                    _ => {/* do nothing */}
                }
            }
        }

        if should_step_cpu
        {
            // clock the cpu
            self.cpu.ir_cycles = 0;  // snake doesn't rely on cycle counting, so just go for it here
            self.cpu.clock(&mut self.memory);

            let state = self.cpu.copy_state();
            
            let instr = state.instruction;
            
            let instr_str = format!("{}", instr);

            println!("{} => {}", instr_str, state);

            let next_instr = instr.ir_address + instr.opsize as u16;
            if next_instr != state.pc
            {
                // we must have made a jump
                println!("========== <<<<<<<<<< JUMP {:#06x} >>>>>>>>>> ==========", state.pc)
            }

            // see if anything changed on the screen
            
            let mut did_screen_change = false;
            for addr in 0x200..0x600
            {
                let value = self.memory.ram[addr];
                let (r,g,b) = color(value).rgb();
                let pixel_idx = ((addr - 0x200) * 3) as usize; // or & 0x1FF?
                if self.vram[pixel_idx] != r || self.vram[pixel_idx+1] != g || self.vram[pixel_idx+2] != b
                {
                    self.vram[pixel_idx+0] = r;
                    self.vram[pixel_idx+1] = g;
                    self.vram[pixel_idx+2] = b;
                    did_screen_change = true;
                }
            }
                
            // update the screen if anything changed
            if did_screen_change
            {
                println!("VRAM DIRTY!");
                
                self.texture.update(None, &self.vram, 32 * 3).unwrap();
                        
                self.canvas.copy(&self.texture, None, None).unwrap();
                self.canvas.present();
            }
        
        }
        else
        {
            self.canvas.present();
        }

    }
}

