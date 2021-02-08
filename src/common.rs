#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

pub trait MemoryBus {
    fn load(&mut self, ptr: u16) -> u8;
    fn store(&mut self, ptr: u16, value: u8);
}

pub trait Clocked {
    fn clock(&mut self);
}

pub struct WORD {}
impl WORD {
    pub fn hi(v: u16) -> u8 {
        v.to_be_bytes()[0]
    }

    pub fn lo(v: u16) -> u8 {
        v.to_be_bytes()[1]
    }

    pub fn make(hi:u8,lo:u8) -> u16 {
        u16::from_be_bytes([hi,lo])
    }
}

// pub trait Savable {
//     fn save(&self, fh: &mut Write);
//     fn load(&mut self, fh: &mut Read);
// }

// pub fn get_bit(x: u8, i: u8) -> u8 {
//     return (x >> i) & 1;
// }


pub fn test_bit(x: u8, i: u8) -> bool {
    return (x >> i) & 1 == 1;
}

// pub fn run_clocks(x: &mut dyn Clocked, num_clocks: u32) {
//     for _i in 0..num_clocks {
//         x.clock();
//     }
// }

pub fn ternary<T>(cond: bool, on_true: T, on_false: T) -> T {
    if cond {
        on_true
    } else {
        on_false
    }
}