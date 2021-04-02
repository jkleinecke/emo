


pub mod ui {

    use crate::mos6502::{State,DecodedInstruction};
    use crate::common::{Byte,Word};
    use cursive::{Cursive};
    use cursive::views::*;
    use cursive::view::*;
    use cursive::utils::markup::*;

    use std::cmp;

    pub fn update(siv: &mut Cursive, cpu: &State, instructions: &[DecodedInstruction], mem: &[Byte])
    {
        update_processor_info(siv, cpu);
        update_instr(siv, instructions);
        update_memory(siv, 0x00, mem);
    }

    pub fn update_instr(siv: &mut Cursive, instr_stream: &[DecodedInstruction])
    {
        for i in 0..16
        {
            let valid = i < instr_stream.len();
            let instr = instr_stream[i];

            let (iraddr,hex,dissassembly) = match valid {
                true => (
                    format!("{:#06X}:", instr.address as Word),
                    format!("{:02X} {:02X} {:02X}", instr.dump[0], instr.dump[1], instr.dump[2] ),
                    format!("{} {:04X}", instr_stream[i].code.opcode, instr_stream[i].operand.value),
                ),
                false => ("".to_string(),"".to_string(),"".to_string()),
            };
            // let (iraddr:&str,hex:&str,dissassembly:&str) = match i {
            //     0..instr_stream.len() => (
            //         format!("{:#06X}", instr_stream[i].address as u16).as_str(),
            //         format!("{:#06X}", instr_stream[i].address as u16).as_str(),
            //         format!("{:#06X}", instr_stream[i].address as u16).as_str(),
            //     ),
            //     _ => ("","",""),
            // };
                        
            siv.call_on_name(format!("iraddr {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(iraddr.as_str())
                )
            });
            
            siv.call_on_name(format!("hex {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(hex.as_str())
                )
            });

            siv.call_on_name(format!("dissassembly {}", i).as_str(), |v: &mut TextView| {
                v.set_content(
                    StyledString::plain(dissassembly.as_str())
                )
            });

        }
    }

    fn update_processor_info(siv: &mut Cursive, cpu: &State)
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
            v.set_content(format!("{}",cpu.ir_cycles+1))
        });
    }

    
    fn update_memory(siv: &mut Cursive, offset: Word, data: &[Byte])
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
                        TextView::new("Address").fixed_width(9)
                    )
                    .child
                    (
                        TextView::new("Hexdump").fixed_width(10)
                    )
                    .child
                    (
                        TextView::new("Dissassembly").fixed_width(12)
                    )
            );

        instrview.add_child(
                TextView::new("-------------------------------")
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
                        TextView::new("aaa").with_name(format!("hex {}", i).as_str()).fixed_width(10)
                    )
                    .child
                    (
                        TextView::new("aaa").with_name(format!("dissassemby {}", i).as_str()).fixed_width(12)
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
                                cpu_field("Status", "status", 9)
                            )
                            //.child( padding() )
                            .child(
                                cpu_field("PC", "pc", 10)
                            )
                            //.child( padding() )
                            .child(
                                cpu_field("IR", "ir", 3)
                            )
                            //.child( padding() )
                            .child(
                                cpu_field("A", "a", 3)
                            )
                            //.child( padding() )
                            .child(
                                cpu_field("X", "x", 3)
                            )
                            //.child( padding() )
                            .child(
                                cpu_field("Y", "y", 3)
                            )
                            //.child( padding() )
                            .child(
                                cpu_field("SP", "sp", 3)
                            )
                            .child(
                                cpu_field("Cycles", "remcycles", 6)
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

    fn cpu_field(title: &str, name: &str, width: usize) -> LinearLayout {
        LinearLayout::vertical()

        .child(
            TextView::new(title)
        )
        .child(
            TextView::new("").with_name(name).fixed_width(width)
        )
        
    }

    //fn padding() -> ResizedView<DummyView> { DummyView.fixed_width(3) }
}
