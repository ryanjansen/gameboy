use gameboy::cpu::CPU;

fn main() {
    let rom: [u8; 0xFFFF] = [0; 0xFFFF];
    let my_cpu: CPU = CPU::new(rom);
    println!("{:?}", my_cpu);
    my_cpu.run();
    println!("{my_cpu:?}");
}
