
#[macro_use]
extern crate libretro_backend;

//mod game;

use rand::Rng;
use rand::prelude::ThreadRng;

fn color(byte: Byte) -> (u8,u8,u8) {
    match byte {
        0 => (0,0,0),
        1 => (255,255,255),
        2 | 9 => (125,125,125),
        3 | 10 => (255,0,0),
        4 | 11 => (0,255,0),
        5 | 12 => (0,0,255),
        6 | 13 => (255,0,255),
        7 | 14 => (255,255,0),
        _ => (0,255,255),
    }
}

use mos6502::{
    Cpu,
    MemoryMapped,
    Clocked,
    Byte, Word, WORD
};


use utilities::ternary;

use cartridge::Rom;

use libretro_backend::{
    CoreInfo,
    AudioVideoInfo,
    PixelFormat,
    GameData,
    LoadGameResult,
    Region,
    RuntimeHandle,
    Key,
};

struct SnakeMemory {
    ram: [Byte;0x800],
    rom: Option<Rom>,
    vram: [Byte;32*32*4],
    vram_updated: bool,
}

impl MemoryMapped for SnakeMemory {
    fn read(&self, addr:Word) -> Byte {
        match addr
        {
            0x8000 ..= 0xFFFF => if let Some(rom) = &self.rom {rom.read(addr & 0x7FFF)}else{0},
            0x0000 ..= 0x1FFF => self.ram[(addr & 0x7FF) as usize],
            _ => 0
        }
    }
    fn write(&mut self, addr:Word, value:Byte) {
        match addr
        {
            0x0000 ..= 0x01FF => self.ram[addr as usize] = value,
            0x0200 ..= 0x05FF => {
                self.ram[addr as usize] = value;
                self.vram_updated = true;
                let idx = (addr - 0x0200) * 4;
                let (r,g,b) = color(value);

                self.vram[idx as usize + 0] = r;
                self.vram[idx as usize + 1] = g;
                self.vram[idx as usize + 2] = b;
            }
            0x0000..= 0x1FFF => { 
                self.ram[(addr & 0x7FF) as usize] = value
            },
            _ => { panic!("Invalid write") },
        };
    }
}

impl SnakeMemory {
    pub fn new() -> Self {
        SnakeMemory {
            ram: [0;0x800],  
            rom: None,  
            vram: [0;32*32*4],
            vram_updated: false,
        }
    }

    pub fn load_rom(&mut self, rom_bytes: &[u8]) -> Result<&Self, String> {
        self.rom = Rom::new(rom_bytes).ok();

        Result::Ok(self)
    }
}

struct SnakeCore {
    cpu: Cpu,
    game: Option<GameData>,
    memory: SnakeMemory,
    rng: ThreadRng,
}

impl SnakeCore {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            game: None,
            memory: SnakeMemory::new(),
            rng: rand::thread_rng(),
        }
    }
}

impl Default for SnakeCore {
    fn default() -> Self {
        Self::new()
    }
}

impl libretro_backend::Core for SnakeCore {
    fn info() -> CoreInfo {
        CoreInfo::new( "Snake", env!( "CARGO_PKG_VERSION" ))
            .supports_roms_with_extension( "nes" )
    }

    fn on_load_game(&mut self, game_data: GameData) -> LoadGameResult {
        if game_data.is_empty() {
            return LoadGameResult::Failed( game_data );
        }

        let romresult = if let Some( data ) = game_data.data() {
            Rom::new(data)
        } else if let Some( path ) = game_data.path() {
            Rom::from_file(path)
        }
        else {
            unreachable!();
        };

        match romresult {
            Ok( rom ) => {
                self.game = Some(game_data);
                self.memory.rom = Some(rom);
                self.cpu.reset = true;

                let av_info = AudioVideoInfo::new()
                    .video(32, 32, 10.0, PixelFormat::ARGB8888)
                    //.audio( 44100.0 )
                    .region(Region::NTSC);

                LoadGameResult::Success( av_info )
            }
            Err( _ ) => {
                LoadGameResult::Failed( game_data )
            }
        }
    }

    fn on_unload_game(&mut self) -> GameData {
        self.game.take().unwrap()
    }



    fn on_run(&mut self, handle: &mut RuntimeHandle) {

        // clock the cpu until a vram write is made
        //while self.memory.vram_updated == false {
            if handle.is_key_pressed(Key::W) { // W
                self.memory.write(0x00FF, 0x77);
            }
            else if handle.is_key_pressed(Key::S) { // S
                self.memory.write(0x00FF, 0x73);
            }
            else if handle.is_key_pressed(Key::A) { // A
                self.memory.write(0x00FF, 0x61);
            }
            else if handle.is_key_pressed(Key::D) { // D
                self.memory.write(0x00FF, 0x64);
            }

            // setup access to random number value
            self.memory.write(0x00FE, self.rng.gen_range(1,16) as u8);

            self.cpu.clock(&mut self.memory);
            //::std::thread::sleep(std::time::Duration::new(0, 70_000));
        //}

        if self.memory.vram_updated {
            handle.upload_video_frame(&self.memory.vram);
            self.memory.vram_updated = false;
        }
        
    }

    fn on_reset(&mut self) {
        self.cpu.reset = true;
        // flush video memory to black
        self.memory.ram = [0;0x800];
        self.memory.vram = [0;32*32*4];
        self.memory.vram_updated = false;
    }
}

libretro_core!( SnakeCore );

// use game::SnakeGame;
// use sdl2::pixels::PixelFormatEnum;

// pub fn run_snake()
// {
//     // init SDL2
//     let sdl_context = sdl2::init().unwrap();
//     let video_subsystem = sdl_context.video().unwrap();
//     let window = video_subsystem
//          .window("Snake game - Step", (32.0 * 10.0) as u32, (32.0 * 10.0) as u32)
//          .position_centered()
//          .build().unwrap();

//     let mut canvas = window.into_canvas().present_vsync().build().unwrap();
//     let mut event_pump = sdl_context.event_pump().unwrap();
//     canvas.set_scale(10.0, 10.0).unwrap();

//     let creator = canvas.texture_creator();
//     let texture = creator.create_texture_target(PixelFormatEnum::RGB24, 32, 32).unwrap();

//     // Main emulator loop
//     let mut game = SnakeGame::new(&create_fake_snake_rom_bytes(), &mut event_pump, &mut canvas, texture);
//     game.run_mode = game::RunMode::StepInstruction;
    
//     game.init();

//     loop
//     {
//         game.update_once();     // right now the game will kill itself on screen input quit

//         match game.run_mode {
//             //game::RunMode::Run => std::thread::sleep(std::time::Duration::new(0, 70_000)),
//             _ => {}// do nothing
//         }
//     }
// }

// fn create_fake_snake_rom_bytes() -> Vec<u8> {
//     let header = vec![0x4E, 0x45, 0x53, 0x1A, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 ];

//     let code = vec![
//     0x20, 0x06, 0x86, 0x20, 0x38, 0x86, 0x20, 0x0d, 0x86, 0x20, 0x2a, 0x86, 0x60, 0xa9, 0x02, 0x85, // 0x600
//     0x02, 0xa9, 0x06, 0x85, 0x03, 0xa9, 0x11, 0x85, 0x10, 0xa9, 0x10, 0x85, 0x12, 0xa9, 0x0f, 0x85, // 0x610    
//     0x14, 0xa9, 0x04, 0x85, 0x11, 0x85, 0x13, 0x85, 0x15, 0x60, 0xa5, 0xfe, 0x85, 0x00, 0xa5, 0xfe, // 0x620
//     0x29, 0x03, 0x18, 0x69, 0x02, 0x85, 0x01, 0x60, 0x20, 0x4d, 0x86, 0x20, 0x8d, 0x86, 0x20, 0xc3, // 0x630 
//     0x86, 0x20, 0x19, 0x87, 0x20, 0x20, 0x87, 0x20, 0x2d, 0x87, 0x4c, 0x38, 0x86, 0xa5, 0xff, 0xc9, // 0x640
//     0x77, 0xf0, 0x0d, 0xc9, 0x64, 0xf0, 0x14, 0xc9, 0x73, 0xf0, 0x1b, 0xc9, 0x61, 0xf0, 0x22, 0x60, // 0x650
//     0xa9, 0x04, 0x24, 0x02, 0xd0, 0x26, 0xa9, 0x01, 0x85, 0x02, 0x60, 0xa9, 0x08, 0x24, 0x02, 0xd0, // 0x660
//     0x1b, 0xa9, 0x02, 0x85, 0x02, 0x60, 0xa9, 0x01, 0x24, 0x02, 0xd0, 0x10, 0xa9, 0x04, 0x85, 0x02, // 0x670
//     0x60, 0xa9, 0x02, 0x24, 0x02, 0xd0, 0x05, 0xa9, 0x08, 0x85, 0x02, 0x60, 0x60, 0x20, 0x94, 0x86, // 0x680
//     0x20, 0xa8, 0x86, 0x60, 0xa5, 0x00, 0xc5, 0x10, 0xd0, 0x0d, 0xa5, 0x01, 0xc5, 0x11, 0xd0, 0x07, // 0x690
//     0xe6, 0x03, 0xe6, 0x03, 0x20, 0x2a, 0x86, 0x60, 0xa2, 0x02, 0xb5, 0x10, 0xc5, 0x10, 0xd0, 0x06, // 0x6A0
//     0xb5, 0x11, 0xc5, 0x11, 0xf0, 0x09, 0xe8, 0xe8, 0xe4, 0x03, 0xf0, 0x06, 0x4c, 0xaa, 0x86, 0x4c, // 0x6B0
//     0x35, 0x87, 0x60, 0xa6, 0x03, 0xca, 0x8a, 0xb5, 0x10, 0x95, 0x12, 0xca, 0x10, 0xf9, 0xa5, 0x02, // 0x6C0
//     0x4a, 0xb0, 0x09, 0x4a, 0xb0, 0x19, 0x4a, 0xb0, 0x1f, 0x4a, 0xb0, 0x2f, 0xa5, 0x10, 0x38, 0xe9, // 0x6D0
//     0x20, 0x85, 0x10, 0x90, 0x01, 0x60, 0xc6, 0x11, 0xa9, 0x01, 0xc5, 0x11, 0xf0, 0x28, 0x60, 0xe6, // 0x6E0
//     0x10, 0xa9, 0x1f, 0x24, 0x10, 0xf0, 0x1f, 0x60, 0xa5, 0x10, 0x18, 0x69, 0x20, 0x85, 0x10, 0xb0, // 0x6F0
//     0x01, 0x60, 0xe6, 0x11, 0xa9, 0x06, 0xc5, 0x11, 0xf0, 0x0c, 0x60, 0xc6, 0x10, 0xa5, 0x10, 0x29, // 0x700
//     0x1f, 0xc9, 0x1f, 0xf0, 0x01, 0x60, 0x4c, 0x35, 0x87, 0xa0, 0x00, 0xa5, 0xfe, 0x91, 0x00, 0x60, // 0x710
//     0xa6, 0x03, 0xa9, 0x00, 0x81, 0x10, 0xa2, 0x00, 0xa9, 0x01, 0x81, 0x10, 0x60, 0xa2, 0x00, 0xea, // 0x720
//     0xea, 0xca, 0xd0, 0xfb, 0x60];                                                                  // 0x730

//     let mut rom_bytes = Vec::new();
//     rom_bytes.extend(header);
//     rom_bytes.extend(&[0; 0x600]);  // pre 0s
//     rom_bytes.extend(code);
//     rom_bytes.extend(&[0;0x7FFB - 0x734]);        // post 0s
//     rom_bytes.extend(&[0x0, 0x86,0,0]);

//     rom_bytes
// }