
mod common;
mod system;
mod bus;
mod operations;
mod cpu6502;
mod tests;

extern crate bitflags;

use crate::system::Nes;

fn main() {
    let _nes = Nes::new();


}
