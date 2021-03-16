
mod common;
mod system;
mod mapper;
mod bus;
mod operations;
mod cpu6502;
mod tests;

use sdl2::event::Event;
use sdl2::EventPump;
use sdl2::keyboard::Keycode;

use sdl2::render::Texture;
use sdl2::pixels::PixelFormatEnum;
use rand::Rng;
use rand::prelude::ThreadRng;

use std::cell::RefCell;
use std::rc::Rc;
use std::cmp;

use crate::operations::*;

use crate::common::WORD;
use crate::system::Clocked;
use crate::cpu6502::Cpu6502;

//use crate::system::Nes;
//use crate::bus::Bus;
use crate::mapper::Mapper;
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

struct Snake {
    pub ram: [u8;0xffff],
    vram: [u8;32*32*3],
    vram_dirty: bool,
    rng: ThreadRng,
}

impl Mapper for Snake {
    fn read(&mut self, addr:u16) -> u8 {
        self.ram[addr as usize]
    }
    fn write(&mut self, addr:u16, value:u8) {
        self.ram[addr as usize] = value;

        if addr >= 0x200 && addr < 0x600 {
            self.vram_dirty = true;
            let (b1,b2,b3) = color(value).rgb();
            let pixel_idx = ((addr - 0x200) * 3) as usize;
            self.vram[pixel_idx + 0] = b1;
            self.vram[pixel_idx + 1] = b2;
            self.vram[pixel_idx + 2] = b3;
        }
    }
}

impl Snake {
    pub fn new() -> Self {
        Snake {
            ram: [0;0xffff],
            vram: [0;32*32*3],
            vram_dirty: false,
            rng: rand::thread_rng(),
        }
    }

    pub fn load(&mut self, offset: usize, data: &Vec<u8>)
    {
        self.ram[offset .. (offset + data.len())].copy_from_slice(&data[..]);
    }

    pub fn update_rand(&mut self) {
        self.ram[0x00FE] = self.rng.gen_range(1,16) as u8;
    }

    pub fn update_dirty_texture(&mut self, texture: &mut Texture ) -> bool {
        let result = self.vram_dirty;

        if self.vram_dirty
        {
            println!("vram dirty!!");
            self.vram_dirty = false;
            texture.update(None, &self.vram, 32 * 3).unwrap();
        }

        result 
    }

    pub fn copy_vram(&self, texture: &mut Texture) {
        texture.update(None, &self.vram, 32 * 3).unwrap();
    }

    
    fn handle_user_input(&mut self, event_pump: &mut EventPump)
    {
        for event in event_pump.poll_iter()
        {
            match event
            {
                Event::Quit {..} | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    std::process::exit(0);
                },
                Event::KeyDown { keycode: Some(Keycode::W), .. } => {
                    self.write(0xffu16, 0x77);
                }
                Event::KeyDown { keycode: Some(Keycode::S), .. } => {
                    self.write(0xffu16, 0x73);
                }
                Event::KeyDown { keycode: Some(Keycode::A), .. } => {
                    self.write(0xffu16, 0x61);
                }
                Event::KeyDown { keycode: Some(Keycode::D), .. } => {
                    self.write(0xffu16, 0x64);
                }
                _ => {/* do nothing */}
            }
        }
    }
}

fn init_game() -> (Rc<RefCell<Snake>>,Cpu6502)
{
    let mapper = Rc::new(RefCell::new(Snake::new()));
    let mut cpu = Cpu6502::new(mapper.clone());

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

    mapper.borrow_mut().load(0x600, &game_code);
    cpu.pc = 0x600;

    (mapper,cpu)
}

use cursive::{Cursive,CursiveRunner,CursiveRunnable};
use cursive::views::*;
use cursive::view::*;
use cursive::utils::markup::*;

struct Ui {

}

impl Ui {

    pub fn update(siv: &mut Cursive, cpu: &Cpu6502, data: &[u8;0xFFFF])
    {
        Ui::update_processor_info(siv, cpu);
        Ui::update_instr(siv, cpu, data);
        Ui::update_memory(siv, cpu.pc, data);
    }

    pub fn update_instr(siv: &mut Cursive, cpu: &Cpu6502, data: &[u8;0xFFFF])
    {
        let mut decoded = DecodedOp::from_ir(cpu.ir);
        let mut iraddr = cpu.pc - decoded.opsize as u16;    // current loaded ir has already moved the pc its full size...

        //let max_rows = cmp::min(16, 0xFFFFu16 - iraddr);

        for i in 0..16
        {
            decoded = DecodedOp::from_ir(data[iraddr as usize + i]);
            
            siv.call_on_name(format!("iraddr {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{:#06X}",iraddr + i as u16).as_str())
                )
            });
            
            siv.call_on_name(format!("ir {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{:02X}",decoded.ir).as_str())
                )
            });

            siv.call_on_name(format!("opcode {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{}",decoded.opcode).as_str())
                )
            });

            siv.call_on_name(format!("addrmode {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{}",decoded.addr_mode).as_str())
                )
            });

            siv.call_on_name(format!("data {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{}{}",
                        match decoded.opsize { 
                            3 => format!("{:02X}",data[iraddr as usize+2]),
                            _ => format!(""),
                        },
                        match decoded.opsize {
                            0 | 1 => format!(""),
                            _ => format!("{:02X}",data[iraddr as usize+1]),
                        }).as_str())
                )
            });

            siv.call_on_name(format!("opsize {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{}",decoded.opsize).as_str())
                )
            });

            siv.call_on_name(format!("cycles {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{}",decoded.cycles).as_str())
                )
            });
            
            siv.call_on_name(format!("oops {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(format!("{}",decoded.oops).as_str())
                )
            });

            iraddr = iraddr.wrapping_add(decoded.opsize as u16);
        }
    }

    pub fn layout() -> ResizedView<Dialog> {
        let mut memview = LinearLayout::vertical();

        for i in 0..32 {
            memview.add_child(
                TextView::new("0xAA   00 11 22 33  44 55 66 77  88 99 AA BB  CC DD EE FF").with_name(format!("addr-{}", i).as_str())
            )
        }

        let mut instrview = LinearLayout::vertical();
        
        instrview.add_child
            (
                LinearLayout::horizontal()
                    .child
                    (
                        TextView::new("addr").fixed_width(9)
                    )
                    .child
                    (
                        TextView::new("ir").fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("op").fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("mode").fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("data").fixed_width(6)
                    )
                    .child
                    (
                        TextView::new("size").fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("cyc").fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("oops").fixed_width(5)
                    )
            );

        instrview.add_child(
                TextView::new("---------------------------------------------")
            );

        for i in 0..16 {
            instrview.add_child
            (
                LinearLayout::horizontal()
                    .child
                    (
                        TextView::new("0x0000:").with_name(format!("iraddr {}", i).as_str()).fixed_width(9)
                    )
                    .child
                    (
                        TextView::new("aaa").with_name(format!("ir {}", i).as_str()).fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("aaa").with_name(format!("opcode {}", i).as_str()).fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("bbb").with_name(format!("addrmode {}", i).as_str()).fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("0000").with_name(format!("data {}", i).as_str()).fixed_width(6)
                    )
                    .child
                    (
                        TextView::new("0000").with_name(format!("opsize {}", i).as_str()).fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("0000").with_name(format!("cycles {}", i).as_str()).fixed_width(5)
                    )
                    .child
                    (
                        TextView::new("0000").with_name(format!("oops {}", i).as_str()).fixed_width(5)
                    )
            );
        }

        Dialog::around
        (
            LinearLayout::horizontal()
                .child
                (
                    Dialog::around(memview).title("Memory")
                )
                .child
                (
                    LinearLayout::vertical()
                    .child
                    (
                        Dialog::around
                        (
                            LinearLayout::horizontal()
                            .child(
                                Ui::cpu_field("Status", "status", 9)
                            )
                            //.child( Ui::padding() )
                            .child(
                                Ui::cpu_field("PC", "pc", 10)
                            )
                            //.child( Ui::padding() )
                            .child(
                                Ui::cpu_field("IR", "ir", 3)
                            )
                            //.child( Ui::padding() )
                            .child(
                                Ui::cpu_field("A", "a", 3)
                            )
                            //.child( Ui::padding() )
                            .child(
                                Ui::cpu_field("X", "x", 3)
                            )
                            //.child( Ui::padding() )
                            .child(
                                Ui::cpu_field("Y", "y", 3)
                            )
                            //.child( Ui::padding() )
                            .child(
                                Ui::cpu_field("SP", "sp", 3)
                            )
                            .child(
                                Ui::cpu_field("Cycles", "remcycles", 6)
                            )
                        ).title("Processor Info")
                    )
                    .child
                    (
                        Dialog::around
                        (
                            instrview
                        )
                        .title("Instructions")
                    )
                )
        )
        .button("quit", |s| s.quit())
        .title("NES 6502 Debugger")
        .full_screen()
    }

    fn update_processor_info(siv: &mut Cursive, cpu: &Cpu6502)
    {
        siv.call_on_name("status", |v: &mut TextView| {
            v.set_content(format!("{}",cpu.status))
        });
        siv.call_on_name("pc", |v: &mut TextView| {
            v.set_content(format!("{:#06X}",cpu.pc))
        });
        siv.call_on_name("ir", |v: &mut TextView| {
            v.set_content(format!("{:02X}",cpu.ir))
        });
        siv.call_on_name("a", |v: &mut TextView| {
            v.set_content(format!("{:02X}",cpu.a))
        });
        siv.call_on_name("x", |v: &mut TextView| {
            v.set_content(format!("{:02X}",cpu.x))
        });
        siv.call_on_name("y", |v: &mut TextView| {
            v.set_content(format!("{:02X}",cpu.y))
        });
        siv.call_on_name("sp", |v: &mut TextView| {
            v.set_content(format!("{:02X}",cpu.sp))
        });
        siv.call_on_name("remcycles", |v: &mut TextView| {
            v.set_content(format!("{}",cpu.ir_cycles))
        });
    }

    
    fn update_memory(siv: &mut Cursive, offset: u16, data: &[u8;0xFFFF])
    {
        let aligned = offset & 0xFFF0;
        let max_rows = cmp::min(32, 0xFFFFu16 - aligned);

        for i in 0..max_rows
        {
            
            let base = aligned + (i << 4);

            let mut text = StyledString::plain("");

            if i < max_rows
            {
                text.append(StyledString::plain(format!("{:#06x} ", base).as_str()));
                
                for k in 0..4
                {
                    text.append(format!("  {:02x} ", data[base as usize + (k*4) + 0]).as_str());
                    text.append(format!("{:02x} ", data[base as usize + (k*4) + 1]).as_str());
                    text.append(format!("{:02x} ", data[base as usize + (k*4) + 2]).as_str());
                    text.append(format!("{:02x}", data[base as usize + (k*4) + 3]).as_str());
                }
            }

            siv.call_on_name(format!("addr-{}", i).as_str(), |v: &mut TextView| {
                v.set_content(text)
            }); 
        }
    }

    fn cpu_field(title: &str, name: &str, width: usize) -> LinearLayout {
        LinearLayout::vertical()

        .child(
            TextView::new(title)
        )
        .child(
            TextView::new("").with_name(name).fixed_width(width)
        )
        
    }

    fn padding() -> ResizedView<DummyView> { DummyView.fixed_width(3) }
}

fn main() {

    // Cursive init
    let mut siv = cursive::default();
    siv.add_layer(
        Ui::layout()
    );
    let should_clock_cpu = Rc::new(RefCell::new(false));
    let did_clock_cpu = should_clock_cpu.clone();

    siv.add_global_callback(' ', move |_| *(should_clock_cpu.borrow_mut()) = true);

    let mut runner = siv.runner();

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
    let mut texture = creator.create_texture_target(PixelFormatEnum::RGB24, 32, 32).unwrap();

    let (mapper, mut cpu) = init_game();
    cpu.clock();

    mapper.borrow_mut().copy_vram(&mut texture);
    canvas.copy(&texture, None, None).unwrap();
    canvas.present();

    Ui::update(&mut runner, &cpu, &mapper.borrow().ram);
    runner.refresh();

    loop
    {
        {
            let mut map = mapper.borrow_mut();
            map.handle_user_input(&mut event_pump);
        }

        if runner.step()
        {
            if *(did_clock_cpu.borrow()) == true
            {
                cpu.ir_cycles = 0;  // just reset back to 0 since we don't care about timing
                cpu.clock();
                
                let mut map = mapper.borrow_mut();
                
                Ui::update(&mut runner, &cpu, &map.ram);
                runner.refresh();

                map.update_rand();
                if map.update_dirty_texture(&mut texture)
                {
                    canvas.copy(&texture, None, None).unwrap();

                    canvas.present();
                }
            
            }

            if runner.is_running() == false
            {
                break;
            }
        }
    }
    // loop
    // {
    //     println!("IR: {:#X} PC: {:#X} SP: {:#X} A: {:03} X: {:03} Y:{:03}", cpu.ir, cpu.pc, cpu.sp, cpu.a, cpu.x, cpu.y);
        



    //     // sleep to slow the game down a little bit
    //     ::std::thread::sleep(std::time::Duration::new(0, 70_000));
    // }
}
