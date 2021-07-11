#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(dead_code)]
#![allow(unused_variables)]

pub mod serialization;

pub use std::cmp::Eq;
pub use std::convert::From;
pub use std::default::Default;
pub use std::fmt::{
	Binary,
	Debug,
	Display,
	LowerHex,
	UpperHex,
    Formatter,
    Result,
};
pub use std::ops::{
	Not,
	BitAnd,
	BitAndAssign,
	BitOrAssign,
	Shl,
	ShlAssign,
	Shr,
	ShrAssign,
};
pub use std::convert::{
    Into
};

#[macro_export]
macro_rules! ternary {
    ($condition: expr, $_true: expr, $_false: expr) => {
        if $condition { $_true } else { $_false }
    };
}

pub type Byte = u8;
pub type Word = u16;

pub trait WORD {
    fn hi(&self) -> Byte;
    fn lo(&self) -> Byte;
    fn make(hi:Byte, lo:Byte) -> Word;
    fn set_hi(&mut self, hi:Byte);
    fn set_lo(&mut self, lo:Byte); 
}

impl WORD for Word {
    fn hi(&self) -> Byte {
        ((self >> 8) & 0xFF) as Byte
    }
    fn lo(&self) -> Byte {
        (self & 0xFF) as Byte
    }
    fn make(hi:Byte, lo:Byte) -> Word
    {
        return ((hi as Word) << 8) | (lo as Word);
    }
    fn set_hi(&mut self, hi:Byte)
    {
        *self = ((hi as Word) << 8) | (self.lo() as Word) as Word;
    }

    fn set_lo(&mut self, lo:Byte)
    {
        *self = ((self.hi() as Word) << 8) | (lo as Word) as Word;
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

pub type Bit = bool;

pub trait BitOps {
    fn on(&self) -> bool;
    fn off(&self) -> bool;
}

impl BitOps for Bit {
    fn on(&self) -> bool { *self == true }
    fn off(&self) -> bool { *self == false }
}

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
    
    fn bit(&self, place:u8) -> Bit
    {
        ((*self >> place) & Self::from(1)) == Self::from(1)
    }
}

impl BitTest for Byte {}