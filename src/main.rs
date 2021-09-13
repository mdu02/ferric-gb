use crate::cpu::CPU;
use crate::ram::RAM;
use std::env::args;

mod cpu;
mod register;
mod ram;

fn main() {
    //initialize internals

    //load ROM
    let args: Vec<String> = args().collect();
    let filename = &args[1];
    let mut ram = RAM::new();
    ram.load_rom(String::from(filename));
    //ram.core_dump();
    let mut cpu = CPU::new(ram);
    //setup user interface

    //main loop
    loop {
        cpu.cycle();
    }
}
