#![allow(unused_imports)]
#![allow(non_upper_case_globals)]
#![allow(dead_code)]
#![allow(unused_variables)]

mod bus;
pub mod utilities;
pub mod cpu;
mod cartridge;

pub use self::bus::Bus;
pub use self::cpu::{
    Cpu,
    MemoryMapped,
    State,
    trace
};
pub use utilities::*;
pub use self::cartridge::{
    Rom,
    Mirroring
};

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn testrom_line1() {
        let testrombytes = include_bytes!("../../test_files/nestest.nes");
        let truthlog = include_str!("../../test_files/nestest_no_cycle.log");
        let mut truthlines = truthlog.lines();

        let testrom = Rom::new(testrombytes).unwrap();
        let mut membus = Bus::from_cartridge(&testrom);
        let mut cpu = Cpu::new();

        // first reset the cpu
        cpu.reset = true;
        cpu.clock(&mut membus);
        cpu.ir_cycles = 0;      // skip the 7 reset cycles for the test
        cpu.regs.pc = 0xC000;   // is this supposed to be what is happening?  otherwise this test won't work correctly...

        let t1 = trace(&mut cpu.regs, &mut membus);
        let line = truthlines.next().unwrap();
        assert_eq!(line, t1);
    }

    #[test]
    fn testrom_cpulog_c68b() {
        let testrombytes = include_bytes!("../../test_files/nestest.nes");
        let truthlog = include_str!("../../test_files/nestest_no_cycle.log");
        let mut truthlines = truthlog.lines();

        let testrom = Rom::new(testrombytes).unwrap();
        let mut membus = Bus::from_cartridge(&testrom);
        let mut cpu = Cpu::new();

        // first reset the cpu
        cpu.reset = true;
        cpu.clock(&mut membus);
        cpu.ir_cycles = 0;      // skip the 7 reset cycles for the test
        cpu.regs.pc = 0xC000;   // is this supposed to be what is happening?  otherwise this test won't work correctly...

        let mut lineno = 1;
        while let Some(line) = truthlines.next() {

            if cpu.regs.pc == 0xC68B {
                // not testing the APU here, so no need to go further...
                break;
            }

            let t1 = trace(&mut cpu.regs, &mut membus);
            assert_eq!(line, t1);

            println!("{}: {} - ✔️", lineno, t1);

            cpu.clock(&mut membus);
            cpu.ir_cycles = 0;

            lineno += 1;
        }
    }
}

