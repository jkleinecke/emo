
mod common;
mod system;
mod bus;
mod mos6502;
mod debugger;
mod snake;

use cursive;
use crate::debugger::ui;
use crate::snake::SnakeGame;

use sdl2::pixels::PixelFormatEnum;

fn main() {
    // Cursive init
    // let mut siv = cursive::default();
    // siv.add_layer( ui::layout() );
    
    // let mut runner = match siv.try_runner() {
    //     Ok(c) => c,
    //     Err(e) => {println!("ERROR: {}", e); panic!("Failed to initialize cursive")},
    // };

    // runner.refresh();

    // init SDL2
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let window = video_subsystem
         .window("Snake game", (32.0 * 10.0) as u32, (32.0 * 10.0) as u32)
         .position_centered()
         .build().unwrap();

    let mut canvas = window.into_canvas().present_vsync().build().unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();
    canvas.set_scale(10.0, 10.0).unwrap();

    let creator = canvas.texture_creator();
    let texture = creator.create_texture_target(PixelFormatEnum::RGB24, 32, 32).unwrap();

    // Main emulator loop
    let mut game = SnakeGame::new(&mut event_pump, &mut canvas, texture);
    game.run_mode = snake::RunMode::StepInstruction;
    
    game.init();
    //let instructions = game.dissassemble().unwrap();

    loop
    {
        game.update_once();     // right now the game will kill itself on screen input quit

        // let cpu_state = game.cpu_state();
        // println!("{}", cpu_state);

        // let mut memory = [0u8;512];
        // game.copy_ram(&mut memory, 0x00, 512);

        // ui::update(&mut runner, &cpu_state, &instructions[cpu_state.pc as usize..16], &memory[..]);
        //runner.step();

        //if runner.is_running() == false
        //{
        //    break;
        //}
        match game.run_mode {
            snake::RunMode::Run => std::thread::sleep(std::time::Duration::new(0, 70_000)),
            _ => {}// do nothing
        }
    }
}
