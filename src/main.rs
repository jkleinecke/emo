
#[macro_use]
mod common;
mod cartridge;
mod nes;
mod mos6502;
mod snake;

extern crate clap;
use clap::{App,SubCommand,AppSettings};

fn main() {
    let matches = App::new("Emo the emulator")
        .version("0.0.1")
        .setting(AppSettings::ArgRequiredElseHelp)
        .subcommand(SubCommand::with_name("run")
            .about("Runs the given emulation")
            .setting(AppSettings::ArgRequiredElseHelp)
            .subcommand(SubCommand::with_name("snake")
                    .about("6502 emulation of Snake")
                )
        )
        .get_matches();

    if let Some(run) = matches.subcommand_matches("run") {
        if run.is_present("snake") {
            snake::run_snake();
        }
    }
}
