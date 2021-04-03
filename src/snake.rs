
use std::collections::BTreeMap;
use std::cell::RefCell;
use std::rc::Rc;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::{Texture};
use sdl2::pixels::{Color};

use rand::Rng;
use rand::prelude::ThreadRng;

use crate::common::{Clocked,Word,Byte};
use crate::mos6502::*;


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
    cpu: Cpu6502,
    memory: Rc<RefCell<SnakeMemory>>,
    event_pump: &'a mut sdl2::EventPump,
    canvas: &'a mut sdl2::render::Canvas<sdl2::video::Window>,
    texture: Texture<'a>,
    pub run_mode: RunMode,
    disassembly: BTreeMap<u16, DecodedInstruction>,
}


struct SnakeMemory {
    ram: [Byte;0xffff],
    vram: [Byte;32*32*3],
    vram_dirty: bool,
}

impl Memory for SnakeMemory {
    fn read(&mut self, addr:Word) -> Byte {
        self.ram[addr as usize]
    }
    fn write(&mut self, addr:Word, value:Byte) {
        
        if addr >= 0x200 && addr < 0x600 {
            self.vram_dirty = true;
            let (b1,b2,b3) = color(value).rgb();
            let pixel_idx = ((addr - 0x200) * 3) as usize;
            self.vram[pixel_idx + 0] = b1;
            self.vram[pixel_idx + 1] = b2;
            self.vram[pixel_idx + 2] = b3;
        }

        self.ram[addr as usize] = value;
    }
}

impl SnakeMemory {
    pub fn new() -> Self {
        SnakeMemory {
            ram: [0;0xffff],
            vram: [0;32*32*3],
            vram_dirty: false,           
        }
    }

    pub fn load(&mut self, offset: usize, data: &Vec<Byte>)
    {
        self.ram[offset .. (offset + data.len())].copy_from_slice(&data[..]);
    }


}

impl<'a> SnakeGame<'a> {
    pub fn new(event_pump: &'a mut sdl2::EventPump, canvas: &'a mut sdl2::render::Canvas<sdl2::video::Window>, texture: sdl2::render::Texture<'a>) -> Self {
        let memory = Rc::new(RefCell::new(SnakeMemory::new()));
        let cpu = Cpu6502::new(memory.clone());

        SnakeGame {
            memory,
            cpu,
            rng: rand::thread_rng(),
            event_pump,
            canvas,
            texture,
            run_mode: RunMode::Run,
            disassembly: BTreeMap::new(),
        }
    }
    

    pub fn init(&mut self) 
    {
        let game_code = vec![
            0x20, 0x06, 0x06, 0x20, 0x38, 0x06, 0x20, 0x0d, 0x06, 0x20, 0x2a, 0x06, 0x60, 0xa9, 0x02,
            0x85, 0x02, 0xa9, 0x04, 0x85, 0x03, 0xa9, 0x11, 0x85, 0x10, 0xa9, 0x10, 0x85, 0x12, 0xa9,
            0x0f, 0x85, 0x14, 0xa9, 0x04, 0x85, 0x11, 0x85, 0x13, 0x85, 0x15, 0x60, 0xa5, 0xfe, 0x85,
            0x00, 0xa5, 0xfe, 0x29, 0x03, 0x18, 0x69, 0x02, 0x85, 0x01, 0x60, 0x20, 0x4d, 0x06, 0x20,
            0x8d, 0x06, 0x20, 0xc3, 0x06, 0x20, 0x19, 0x07, 0x20, 0x20, 0x07, 0x20, 0x2d, 0x07, 0x4c,
            0x38, 0x06, 0xa5, 0xff, 0xc9, 0x77, 0xf0, 0x0d, 0xc9, 0x64, 0xf0, 0x14, 0xc9, 0x73, 0xf0,
            0x1b, 0xc9, 0x61, 0xf0, 0x22, 0x60, 0xa9, 0x04, 0x24, 0x02, 0xd0, 0x26, 0xa9, 0x01, 0x85,
            0x02, 0x60, 0xa9, 0x08, 0x24, 0x02, 0xd0, 0x1b, 0xa9, 0x02, 0x85, 0x02, 0x60, 0xa9, 0x01,
            0x24, 0x02, 0xd0, 0x10, 0xa9, 0x04, 0x85, 0x02, 0x60, 0xa9, 0x02, 0x24, 0x02, 0xd0, 0x05,
            0xa9, 0x08, 0x85, 0x02, 0x60, 0x60, 0x20, 0x94, 0x06, 0x20, 0xa8, 0x06, 0x60, 0xa5, 0x00,
            0xc5, 0x10, 0xd0, 0x0d, 0xa5, 0x01, 0xc5, 0x11, 0xd0, 0x07, 0xe6, 0x03, 0xe6, 0x03, 0x20,
            0x2a, 0x06, 0x60, 0xa2, 0x02, 0xb5, 0x10, 0xc5, 0x10, 0xd0, 0x06, 0xb5, 0x11, 0xc5, 0x11,
            0xf0, 0x09, 0xe8, 0xe8, 0xe4, 0x03, 0xf0, 0x06, 0x4c, 0xaa, 0x06, 0x4c, 0x35, 0x07, 0x60,
            0xa6, 0x03, 0xca, 0x8a, 0xb5, 0x10, 0x95, 0x12, 0xca, 0x10, 0xf9, 0xa5, 0x02, 0x4a, 0xb0,
            0x09, 0x4a, 0xb0, 0x19, 0x4a, 0xb0, 0x1f, 0x4a, 0xb0, 0x2f, 0xa5, 0x10, 0x38, 0xe9, 0x20,
            0x85, 0x10, 0x90, 0x01, 0x60, 0xc6, 0x11, 0xa9, 0x01, 0xc5, 0x11, 0xf0, 0x28, 0x60, 0xe6,
            0x10, 0xa9, 0x1f, 0x24, 0x10, 0xf0, 0x1f, 0x60, 0xa5, 0x10, 0x18, 0x69, 0x20, 0x85, 0x10,
            0xb0, 0x01, 0x60, 0xe6, 0x11, 0xa9, 0x06, 0xc5, 0x11, 0xf0, 0x0c, 0x60, 0xc6, 0x10, 0xa5,
            0x10, 0x29, 0x1f, 0xc9, 0x1f, 0xf0, 0x01, 0x60, 0x4c, 0x35, 0x07, 0xa0, 0x00, 0xa5, 0xfe,
            0x91, 0x00, 0x60, 0xa6, 0x03, 0xa9, 0x00, 0x81, 0x10, 0xa2, 0x00, 0xa9, 0x01, 0x81, 0x10,
            0x60, 0xa6, 0xff, 0xea, 0xea, 0xca, 0xd0, 0xfb, 0x60,
        ];

        {
            let mut mem = self.memory.borrow_mut();
            mem.load(0x600, &game_code);
            self.cpu.pc = 0x600;
            self.cpu.status.set_interrupt(false);

            self.texture.update(None, &mem.vram, 32 * 3).unwrap();

            self.canvas.copy(&self.texture, None, None).unwrap();
        }

        self.canvas.window_mut().set_title("Snake Game - Space to Step CPU");

        self.disassembly = self.dissassemble().unwrap();

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
            let mut mem = self.memory.borrow_mut();
            
            // update the random memory generator location
            mem.write(0x00FE, self.rng.gen_range(1,16) as u8);

            // handle user input
            for event in self.event_pump.poll_iter()
            {
                match event
                {
                    Event::Quit {..} | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                        std::process::exit(0);
                    },
                    Event::KeyDown { keycode: Some(Keycode::W), .. } => {
                        mem.write(0x00FF, 0x77);
                    }
                    Event::KeyDown { keycode: Some(Keycode::S), .. } => {
                        mem.write(0x00FF, 0x73);
                    }
                    Event::KeyDown { keycode: Some(Keycode::A), .. } => {
                        mem.write(0x00FF, 0x61);
                    }
                    Event::KeyDown { keycode: Some(Keycode::D), .. } => {
                        mem.write(0x00FF, 0x64);
                    }
                    Event::KeyDown { keycode: Some(Keycode::Space), .. } => {
                        self.run_mode = RunMode::StepInstruction;
                        self.canvas.window_mut().set_title("Snake Game - Space to Step CPU");
                        should_step_cpu = true;
                    }
                    Event::KeyDown { keycode: Some(Keycode::F5), .. } => {
                        self.canvas.window_mut().set_title("Snake Game");
                        self.run_mode = RunMode::Run;
                    }
                    _ => {/* do nothing */}
                }
            }
        }

        if should_step_cpu
        {
            let last_pc = self.cpu.pc;
            // clock the cpu
            self.cpu.ir_cycles = 0;  // snake doesn't rely on cycle counting, so just go for it here
            self.cpu.clock();

            let mut mem = self.memory.borrow_mut();

            let state = self.cpu.copy_state();
            
            let instr = self.disassembly.get(&last_pc).unwrap();
            
            let instr_str = format!("{:#06x}:  {:02x} {:02x} {:02x}   {} {}", instr.address, instr.dump[0], instr.dump[1], instr.dump[2], instr.code.opcode, instr.operand);

            println!("{} => {}", instr_str, state);

            let next_instr = last_pc + instr.code.opsize as u16;
            if next_instr != state.pc
            {
                // we must have made a jump
                println!("========== <<<<<<<<<< JUMP {:#06x} >>>>>>>>>> ==========", state.pc)
            }
                
            // update the screen if anything changed
            if mem.vram_dirty
            {
                println!("VRAM DIRTY!");
                mem.vram_dirty = false;
                
                self.texture.update(None, &mem.vram, 32 * 3).unwrap();

                self.canvas.copy(&self.texture, None, None).unwrap();
                self.canvas.present();
            }
        }
        else
        {
            self.canvas.present();
        }

    }

    pub fn cpu_state(&self) -> State {
        self.cpu.copy_state()
    }

    pub fn dissassemble(&self) -> Result<BTreeMap<u16,DecodedInstruction>,DissassemblyError>
    {
        crate::mos6502::dissassemble(&self.memory.borrow().ram[0x600..0x73b], 0x600)
    }

    pub fn copy_ram(&self, dst:&mut [Byte], offset:Word,size:Word) 
    {
        dst.copy_from_slice(&self.memory.borrow().ram[offset as usize..size as usize]);        
    }
}

