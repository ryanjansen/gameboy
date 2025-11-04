mod cpu;
use cpu::CPU;
use std::fmt;

fn main() {
    let ROM: [u8; 0xFFFF] = [0; 0xFFFF];
    let my_cpu: CPU = cpu::CPU::new(ROM);
    println!("{:?}", my_cpu);
    my_cpu.run();
    println!("{my_cpu:?}");
}
