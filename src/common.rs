#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

pub trait WORD {
    fn hi(&self) -> u8;
    fn lo(&self) -> u8;
    fn make(hi:u8, lo:u8) -> u16;
    fn set_hi(&mut self, hi:u8);
    fn set_lo(&mut self, lo:u8); 
}

impl WORD for u16 {
    fn hi(&self) -> u8 {
        ((self << 8) & 0xFF) as u8
    }
    fn lo(&self) -> u8 {
        (self & 0xFF) as u8
    }
    fn make(hi:u8, lo:u8) -> u16
    {
        return ((hi as u16) << 8) | (lo as u16);
    }
    fn set_hi(&mut self, hi:u8)
    {
        *self = ((hi as u16) << 8) | (self.lo() as u16) as u16;
    }

    fn set_lo(&mut self, lo:u8)
    {
        *self = ((self.hi() as u16) << 8) | (lo as u16) as u16;
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