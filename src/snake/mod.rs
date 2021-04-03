
mod game;

use game::SnakeGame;
use sdl2::pixels::PixelFormatEnum;

pub fn run_snake()
{
    // init SDL2
    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let window = video_subsystem
         .window("Snake game - Step", (32.0 * 10.0) as u32, (32.0 * 10.0) as u32)
         .position_centered()
         .build().unwrap();

    let mut canvas = window.into_canvas().present_vsync().build().unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();
    canvas.set_scale(10.0, 10.0).unwrap();

    let creator = canvas.texture_creator();
    let texture = creator.create_texture_target(PixelFormatEnum::RGB24, 32, 32).unwrap();

    // Main emulator loop
    let mut game = SnakeGame::new(&mut event_pump, &mut canvas, texture);
    game.run_mode = game::RunMode::StepInstruction;
    
    game.init();

    loop
    {
        game.update_once();     // right now the game will kill itself on screen input quit

        match game.run_mode {
            game::RunMode::Run => std::thread::sleep(std::time::Duration::new(0, 100_000)),
            _ => {}// do nothing
        }
    }
}