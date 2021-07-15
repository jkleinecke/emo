
// impl Display for Operand
// {
//     fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
//         match self.addr_mode {
//             abs => write!(f, "${:04x}   ", self.value),
//             acc => write!(f, "A       "),
//             imm => write!(f, "#${:02x}    ", self.value & 0xff),
//             imp => write!(f, "        "),
//             izx => write!(f, "(${:02x},X) ", self.value & 0xff),
//             izy => write!(f, "(${:02x},Y) ", self.value & 0xff),
//             zp =>  write!(f, "${:02x}     ", self.value & 0xff),
//             zpx => write!(f, "${:02x},X   ", self.value & 0xff),
//             zpy => write!(f, "${:02x},Y   ", self.value & 0xff),
//             rel => write!(f, "${:02x}     ", self.value & 0xff),
//             abx => write!(f, "${:04x},X ", self.value),
//             aby => write!(f, "${:04x},Y ", self.value),
//             ind => write!(f, "(${:04x}) ", self.value),
//         }
//     }
// }

