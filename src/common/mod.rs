#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]


use std::cmp::Eq;
use std::convert::From;
use std::default::Default;
use std::fmt::{
	Binary,
	Debug,
	Display,
	LowerHex,
	UpperHex,
};
use std::ops::{
	Not,
	BitAnd,
	BitAndAssign,
	BitOrAssign,
	Shl,
	ShlAssign,
	Shr,
	ShrAssign,
};

pub trait WORD {
    fn hi(&self) -> u8;
    fn lo(&self) -> u8;
    fn make(hi:u8, lo:u8) -> u16;
    fn set_hi(&mut self, hi:u8);
    fn set_lo(&mut self, lo:u8); 
}

impl WORD for u16 {
    fn hi(&self) -> u8 {
        ((self >> 8) & 0xFF) as u8
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

pub trait Clocked {
    fn clock(&mut self);  
}

// pub trait Savable {
//     fn save(&self, fh: &mut Write);
//     fn load(&mut self, fh: &mut Read);
// }

// pub fn get_bit(x: u8, i: u8) -> u8 {
//     return (x >> i) & 1;
// }


// pub fn run_clocks(x: &mut dyn Clocked, num_clocks: u32) {
//     for _i in 0..num_clocks {
//         x.clock();
//     }
// }


pub trait BitTest:
    Binary
    + BitAnd<Self, Output=Self>
    + BitAndAssign<Self>
    + BitOrAssign<Self>
    //  Permit indexing into a generic array
    + Copy
    + Debug
    //  `BitVec` cannot push new elements without this. (Well, it CAN, but
    //  `mem::uninitialized` is Considered Harmful.)
    + Default
    + Display
    //  Permit testing a value against 1 in `get()`.
    + Eq
    //  Rust treats numeric literals in code as vaguely typed and does not make
    //  them concrete until long after trait expansion, so this enables building
    //  a concrete Self value from a numeric literal.
    + From<u8>
    + LowerHex
    + Not<Output=Self>
    + Shl<u8, Output=Self>
    + ShlAssign<u8>
    + Shr<u8, Output=Self>
    + ShrAssign<u8>
    //  Allow direct access to a concrete implementor type.
    + Sized
    + UpperHex
{
    fn set(&mut self, place:u8, value:bool)
    {
        //  Blank the selected bit
		*self &= !(Self::from(1u8) << place);
		//  Set the selected bit
		*self |= Self::from(value as u8) << place;
    }

    fn get(&self, place:u8) -> bool
    {
        self.bit(place)
    }
    
    fn bit(&self, place:u8) -> bool
    {
        (*self >> place) & Self::from(1) == Self::from(1)
    }

    fn on(&self, place:u8) -> bool
    {
        self.bit(place)
    }
    
    fn off(&self, place:u8) -> bool
    {
        !self.bit(place)
    }

    fn flip(&mut self, place:u8) {
        self.set(place, !self.get(place));
    }
}

impl BitTest for u8 {}