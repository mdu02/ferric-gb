use crate::register::Register;
use crate::ram::RAM;

pub enum WideReg {
    AF,
    BC,
    DE,
    HL,
    SP,
    PC
}

pub enum NarrowReg {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
    HLInd //this is a pseudoregister that is a indirect of HL
}

pub enum Flag {
    Z,
    N,
    H,
    C
}

pub struct CPU {
    ram: RAM,
    af: Register,
    bc: Register,
    de: Register,
    hl: Register,
    sp: Register,
    pc: Register
}

impl CPU{
    pub fn new(ram: RAM) -> CPU{
        let checksum_zero = ram.header_checksum == 0;
        let mut cpu = CPU{
            ram,
            af: if checksum_zero {Register::new(0x01_80)} else {Register::new(0x01_B0)},
            bc: Register::new(0x0013),
            de: Register::new(0x00D8),
            hl: Register::new(0x014D),
            sp: Register::new(0xFFFE),
            pc: Register::new(0x0100)
        };
        cpu
    }
    pub fn cycle(&mut self){
        //fetch
        let curr_address = self.pc.read_reg();
        let instruction = self.ram.read_byte(curr_address);
        self.pc.next_instruction();

        //nibbles
        let op_x = (instruction & 0o300) >> 6;
        let op_y = (instruction & 0o070) >> 3;
        let op_z = (instruction & 0o007);

        //commonly encountered expressions
        let op_p = op_y >> 1;
        let op_q = op_y % 2;

        //decode/exec
        match (op_x, op_y, op_z){
            //NOP, does absolutely nothing
            (0, 0, 0) => {}
            //JP nn
            (3, 0, 3) => {
                let nn = self.ram.read_word(self.pc.read_reg());
                self.pc.write_reg(nn) // no need to inc pc because changed immediately
            }
            _ => {
                panic!("Missing Opcode {:#05o}", instruction);
            }
        }
    }
    fn read_narrow_reg(&self, reg: NarrowReg) -> u8{
        match reg{
            NarrowReg::A => self.af.read_top(),
            NarrowReg::B => self.bc.read_top(),
            NarrowReg::C => self.bc.read_bottom(),
            NarrowReg::D => self.de.read_top(),
            NarrowReg::E => self.de.read_bottom(),
            NarrowReg::H => self.hl.read_top(),
            NarrowReg::L => self.hl.read_bottom(),
            NarrowReg::HLInd => self.ram.read_byte(self.hl.read_reg())
        }
    }

    fn read_wide_reg(&self, reg: WideReg) -> u16{
        match reg{
            WideReg::AF => self.af.read_reg(),
            WideReg::BC => self.bc.read_reg(),
            WideReg::DE => self.de.read_reg(),
            WideReg::HL => self.hl.read_reg(),
            WideReg::SP => self.sp.read_reg(),
            WideReg::PC => self.pc.read_reg()
        }
    }

    fn write_narrow_reg(&mut self, reg: NarrowReg, val: u8){
        match reg{
            NarrowReg::A => {self.af.write_top(val);}
            NarrowReg::B => {self.bc.write_top(val); }
            NarrowReg::C => {self.bc.write_bottom(val);}
            NarrowReg::D => {self.de.write_top(val);}
            NarrowReg::E => {self.de.write_bottom(val);}
            NarrowReg::H => {self.hl.write_top(val);}
            NarrowReg::L => {self.hl.write_bottom(val);}
            NarrowReg::HLInd => {self.ram.write_byte(self.hl.read_reg(), val);}
        }
    }

    fn write_wide_reg(&mut self, reg: WideReg, val:u16){
        match reg{
            WideReg::AF => {self.af.write_reg(val);}
            WideReg::BC => {self.bc.write_reg(val);}
            WideReg::DE => {self.de.write_reg(val);}
            WideReg::HL => {self.hl.write_reg(val);}
            WideReg::SP => {self.sp.write_reg(val);}
            WideReg::PC => {self.pc.write_reg(val);}
        }
    }

    fn get_flag(&self, f: Flag) -> bool{
        match f{ //and with relevant bit
            Flag::Z => (self.af.read_reg() & (1 << 7)) != 0,
            Flag::N => (self.af.read_reg() & (1 << 6)) != 0,
            Flag::H => (self.af.read_reg() & (1 << 5)) != 0,
            Flag::C => (self.af.read_reg() & (1 << 4)) != 0
        }
    }

    fn set_flag(&mut self, f : Flag, val: bool){
        match f{ //bitmask out bit then and with boolean
            Flag::Z => {self.af.write_reg(self.af.read_reg() & 0xFF7F | ((val as u16) << 7));}
            Flag::N => {self.af.write_reg(self.af.read_reg() & 0xFFBF | ((val as u16) << 6));}
            Flag::H => {self.af.write_reg(self.af.read_reg() & 0xFFDF | ((val as u16) << 5));}
            Flag::C => {self.af.write_reg(self.af.read_reg() & 0xFFEF | ((val as u16) << 4));}
        }
    }

    fn reg_lookup(index: u8) -> NarrowReg{
        match index{
            0 => NarrowReg::B,
            1 => NarrowReg::C,
            2 => NarrowReg::D,
            3 => NarrowReg::E,
            4 => NarrowReg::H,
            5 => NarrowReg::L,
            6 => NarrowReg::HLInd,
            7 => NarrowReg::A,
            _ => panic!("Should not be reachable")
        }
    }
    fn wide_reg_lookup_1(index: u8) -> WideReg{
        match index{
            0 => WideReg::BC,
            1 => WideReg::DE,
            2 => WideReg::HL,
            3 => WideReg::SP,
            _ => panic!("Should not be reachable")
        }
    }
    fn wide_reg_lookup_2(index: u8) -> WideReg{
        match index{
            0 => WideReg::BC,
            1 => WideReg::DE,
            2 => WideReg::HL,
            3 => WideReg::AF,
            _ => panic!("Should not be reachable")
        }
    }

    fn condition_lookup(&self, index: u8) -> bool{
        match index{
            0 => !(self.get_flag(Flag::Z)),
            1 => self.get_flag(Flag::Z),
            2 => !(self.get_flag(Flag::C)),
            3 => self.get_flag(Flag::C),
            _ => panic!("Should not be reachable")
        }
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flags(){
        let mut ram = RAM::new();
        let mut cpu = CPU::new(ram);
        cpu.set_flag(Flag::Z, true);
        cpu.set_flag(Flag::N, false);
        cpu.set_flag(Flag::H, true);
        cpu.set_flag(Flag::C, false);
        assert_eq!(true, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::C));
    }

    #[test]
    fn jp_nn_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o303);
        ram.write_byte(0x101, 0xDE);
        ram.write_byte(0x102, 0xAC);
        let mut cpu = CPU::new(ram);
        cpu.cycle();
        assert_eq!(0xACDE, cpu.pc.read_reg());
    }
}