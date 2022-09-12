use std::fs::read;

pub struct RAM {
    memory: [u8; 0x10000],
    pub header_checksum: u8
}

impl RAM {
    pub fn new() -> RAM {
        let mut ram = RAM { memory: [0; 0x10000] , header_checksum: 0};
        ram
    }

    pub fn read_byte(&self, adr: u16) -> u8 {
        self.memory[adr as usize]
    }


    pub fn read_word(&self, adr: u16) -> u16 {
        // note that Z80 is little endian!
        ((self.memory[(adr + 1) as usize] as u16) << 8)  + (self.memory[adr as usize]) as u16
    }

    pub fn write_byte(&mut self, adr: u16, val: u8) {
        self.memory[adr as usize] = val
    }

    pub fn load_rom(&mut self, filename: String) {
        let rom_bytes = read(filename).unwrap();
        let mut address: u16 = 0;
        for i in &rom_bytes{
            self.write_byte(address, *i);
        address += 1;
        }

        //checksum
        let mut cs: u8 = 0;
        for i in 0x134..0x14D{
            cs = cs.wrapping_sub(self.read_byte(i)).wrapping_sub(1);
        }
        self.header_checksum = cs;
    }

    pub fn core_dump(&self) {
        print!("Offset");
        for i in 0..16{
            print!(" {:02X}", i);
        }
        println!();
        for i in 0..4096{
            print!("0x{:04X}", i*16);
            for j in 0..16{
                print!(" {:02X}", self.read_byte(i*16 + j))
            }
            println!();
        }

    }
}