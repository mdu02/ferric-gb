pub struct RAM {
    memory: [u8; 0x1000]
}

impl RAM{
    pub fn new() -> RAM {
        let mut ram = RAM {memory: [0; 4096]};
        ram
    }

    pub fn read_byte(&self, adr: u16) -> u8 {
        self.memory[adr as usize]
    }

    pub fn write_byte(&mut self, adr: u16, val: u8) {
        self.memory[adr as usize] = val
    }
}