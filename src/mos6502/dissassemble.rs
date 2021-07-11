
use super::{WORD,Word,Byte};
use super::operations::*;
use std::collections::BTreeMap;
use std::fmt::{
    Display,Formatter,Debug
};

#[derive(Debug,Copy,Clone)]
pub struct DecodedOp
{
    pub ir: u8,

    pub opcode: Operation,
    pub addr_mode: AddressingMode,
    pub opsize: u8,
    
    pub cycles: u8,
    pub oops: bool,
}

#[derive(Debug,Copy,Clone)]
pub struct Operand
{
    pub addr_mode: AddressingMode,
    pub value: u16,
}

impl Display for Operand
{
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self.addr_mode {
            abs => write!(f, "${:04x}   ", self.value),
            acc => write!(f, "A       "),
            imm => write!(f, "#${:02x}    ", self.value & 0xff),
            imp => write!(f, "        "),
            izx => write!(f, "(${:02x},X) ", self.value & 0xff),
            izy => write!(f, "(${:02x},Y) ", self.value & 0xff),
            zp =>  write!(f, "${:02x}     ", self.value & 0xff),
            zpx => write!(f, "${:02x},X   ", self.value & 0xff),
            zpy => write!(f, "${:02x},Y   ", self.value & 0xff),
            rel => write!(f, "${:02x}     ", self.value & 0xff),
            abx => write!(f, "${:04x},X ", self.value),
            aby => write!(f, "${:04x},Y ", self.value),
            ind => write!(f, "(${:04x}) ", self.value),
        }
    }
}

#[derive(Debug,Copy,Clone)]
pub struct DecodedInstruction
{
    pub address: u16,
    pub code: DecodedOp,
    pub operand: Operand,
    pub dump: [u8;3],
}

impl DecodedOp
{
    pub fn from_ir(ir: u8) -> Self
    {
        let (operation, addr_mode, c1, c2) = OPCODE_TABLE[ir as usize];
        let opsize = match ir {
            0x00 => 2,  // break is size 2 despite the implied addressing mode
            _ => ADDR_OPSIZE_TABLE[addr_mode as usize]
        };

        DecodedOp {
            ir,  // read direct from memory, just in case we have the opsize incorrect
            opcode: operation,
            addr_mode,
            opsize,
            cycles: c1,
            oops: match c2 {
                0 => false,
                _ => true,
            }
        }
    }
}

#[derive(Debug,Copy,Clone,PartialEq)]
pub enum DissassemblyError
{
    Error(&'static str),
}

pub fn dissassemble(instr_stream: &[u8], stream_offset: u16) -> Result<BTreeMap<u16, DecodedInstruction>, DissassemblyError>
{
    let mut instructions = BTreeMap::new();

    let mut index = 0;

    while index < instr_stream.len()
    {
        let code = DecodedOp::from_ir(instr_stream[index]);
        let address = stream_offset + index as u16;
        let mut dump = [0u8;3];
        dump[0..code.opsize as usize].copy_from_slice(&instr_stream[index..index + code.opsize as usize]);

        if index + code.opsize as usize > instr_stream.len()
        {
            return Err(DissassemblyError::Error("Unexpected end of instruction stream!"));
        }

        let lo = match code.opsize {
            0 | 1 => 0,
            _ => instr_stream[index+1],
        };

        let hi = match code.opsize {
            3 => instr_stream[index+2],
            _ => 0,
        };

        let operand = Operand {
            addr_mode: code.addr_mode,
            value: Word::make(hi,lo)
        };

        index += code.opsize as usize;

        instructions.insert(
                address,
                DecodedInstruction {
                    address,
                    code,
                    operand,
                    dump,
                }
            );
    }

    Ok(instructions)
}