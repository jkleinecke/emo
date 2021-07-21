use super::{Byte,Word,Bit,BitTest,ternary};

pub use std::convert::{
	From,
    Into
};

pub use std::fmt::{
    Display,
    Debug,
    Formatter,
    Result,
};

pub use std::cmp::{
    PartialEq
};

pub use std::ops::{
    BitAndAssign,
	BitOrAssign,  
};

#[derive(Default,Copy,Clone)]
pub struct StatusRegister {
    pub carry: Bit, // 0
    pub zero: Bit,  // 1
    pub interrupt: Bit, // 2
    pub decimal: Bit,   //3
    
    pub overflow: Bit,  // 6
    pub negative: Bit,  // 7
}

impl PartialEq<StatusRegister> for StatusRegister {
    fn eq(&self, other: &StatusRegister) -> bool {
        self.to_byte() == other.to_byte()
    }
}

impl Debug for StatusRegister {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let n = ternary!(self.negative, 'N', 'n');
        let v = ternary!(self.overflow, 'V', 'v');
        let b = 'b';
        let d = ternary!(self.decimal, 'D', 'd');
        let i = ternary!(self.interrupt, 'I', 'i');
        let z = ternary!(self.zero, 'Z', 'z');
        let c = ternary!(self.carry, 'C', 'c');

        write!(f, "{}{}-{}{}{}{}{}", n,v,b,d,i,z,c)
    }
}

impl Display for StatusRegister {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let n = ternary!(self.negative, 'N', 'n');
        let v = ternary!(self.overflow, 'V', 'v');
        let b = 'b';
        let d = ternary!(self.decimal, 'D', 'd');
        let i = ternary!(self.interrupt, 'I', 'i');
        let z = ternary!(self.zero, 'Z', 'z');
        let c = ternary!(self.carry, 'C', 'c');

        write!(f, "{}{}-{}{}{}{}{}", n,v,b,d,i,z,c)
    }
}

impl From<&str> for StatusRegister {
    fn from(status_str:&str) -> Self
    {
        let mut ret = StatusRegister::from(0x20);
        let mut it = status_str.chars();


        ret.negative = it.next() == Some('N');
        ret.overflow = it.next() == Some('V');
        it.next();  // don't care about unused
        it.next();  // don't care about break
        ret.decimal = it.next() == Some('D');
        ret.interrupt = it.next() == Some('I');
        ret.zero = it.next() == Some('Z');
        ret.carry = it.next() == Some('C');

        ret
    }
}

impl From<Byte> for StatusRegister {
    fn from(src: Byte) -> Self {
        Self {
            carry: (src >> 0) & 1 == 1,
            zero: (src >> 1) & 1 == 1,
            interrupt: (src >> 2) & 1 == 1,
            decimal: (src >> 3) & 1 == 1,
            //r#break: (src >> 4) & 1 == 1,
            //unused: (src >> 5) & 1 == 1,
            overflow: (src >> 6) & 1 == 1,
            negative: (src >> 7) & 1 == 1,
        }
    }
}

impl BitAndAssign<Byte> for StatusRegister {
    fn bitand_assign(&mut self, rhs: Byte) {
        self.carry &= (rhs >> 0) & 1 == 1;
        self.zero &= (rhs >> 1) & 1 == 1;
        self.interrupt &= (rhs >> 2) & 1 == 1;
        self.decimal &= (rhs >> 3) & 1 == 1;
        //self.r#break &= (rhs >> 4) & 1 == 1;
        //self.unused &= (rhs >> 5) & 1 == 1;
        self.overflow &= (rhs >> 6) & 1 == 1;
        self.negative &= (rhs >> 7) & 1 == 1;
    }
}


impl BitOrAssign<Byte> for StatusRegister {
    fn bitor_assign(&mut self, rhs: Byte) {
        self.carry |= (rhs >> 0) & 1 == 1;
        self.zero |= (rhs >> 1) & 1 == 1;
        self.interrupt |= (rhs >> 2) & 1 == 1;
        self.decimal |= (rhs >> 3) & 1 == 1;
        //self.r#break |= (rhs >> 4) & 1 == 1;
        //self.unused |= (rhs >> 5) & 1 == 1;
        self.overflow |= (rhs >> 6) & 1 == 1;
        self.negative |= (rhs >> 7) & 1 == 1;
    }
}

impl StatusRegister {
    pub fn to_byte(&self) -> Byte {
        self.carry as Byte
        | (self.zero as Byte) << 1
        | (self.interrupt as Byte) << 2
        | (self.decimal as Byte) << 3
        //| (self.r#break as Byte) << 4
        //| (self.unused as Byte) << 5
        | 1 << 5    // unused bit
        | (self.overflow as Byte) << 6
        | (self.negative as Byte) << 7
    } 
}

pub struct Registers {
    pub ac: Byte,
    pub x: Byte,
    pub y: Byte,
    pub pc: Word,
    pub sp: Byte,
    pub p: StatusRegister
}
