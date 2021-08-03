use crate::register::Register;

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
}

pub enum Flag {
    Z,
    N,
    H,
    C
}

pub struct CPU {
    af: Register,
    bc: Register,
    de: Register,
    hl: Register,
    sp: Register,
    pc: Register
}

impl CPU{
    pub fn new() -> CPU{
       let mut cpu = CPU{
           af: Register::new(0b0000001_10000000),
           bc: Register::new(0x0013),
           de: Register::new(0x00D8),
           hl: Register::new(0x014D),
           sp: Register::new(0xFFFE),
           pc: Register::new(0x100)
       };
        cpu
    }

    fn read_narrow_reg(&self, reg: NarrowReg) -> u8{
        match reg{
            NarrowReg::A=>{
                self.af.read_top()
            }
            NarrowReg::B=>{
                self.bc.read_top()
            }
            NarrowReg::C=>{
                self.bc.read_bottom()
            }
            NarrowReg::D=>{
                self.de.read_top()
            }
            NarrowReg::E=>{
                self.de.read_bottom()
            }
            NarrowReg::H=>{
                self.hl.read_top()
            }
            NarrowReg::L=>{
                self.hl.read_bottom()
            }
        }
    }

    fn read_wide_reg(&self, reg: WideReg) -> u16{
        match reg{
            WideReg::AF=>{
                self.af.read_reg()
            }
            WideReg::BC=>{
                self.bc.read_reg()
            }
            WideReg::DE=>{
                self.de.read_reg()
            }
            WideReg::HL=>{
                self.hl.read_reg()
            }
            WideReg::SP=>{
                self.sp.read_reg()
            }
            WideReg::PC=>{
                self.pc.read_reg()
            }
        }
    }

    fn write_narrow_reg(&mut self, reg: NarrowReg, val: u8){
        match reg{
            NarrowReg::A=>{
                self.af.write_top(val);
            }
            NarrowReg::B=>{
                self.bc.write_top(val);
            }
            NarrowReg::C=>{
                self.bc.write_bottom(val);
            }
            NarrowReg::D=>{
                self.de.write_top(val);
            }
            NarrowReg::E=>{
                self.de.write_bottom(val);
            }
            NarrowReg::H=>{
                self.hl.write_top(val);
            }
            NarrowReg::L=>{
                self.hl.write_bottom(val);
            }
        }
    }

    fn write_wide_reg(&mut self, reg: WideReg, val:u16){
        match reg{
            WideReg::AF=>{
                self.af.write_reg(val);
            }
            WideReg::BC=>{
                self.bc.write_reg(val);
            }
            WideReg::DE=>{
                self.de.write_reg(val);
            }
            WideReg::HL=>{
                self.hl.write_reg(val);
            }
            WideReg::SP=>{
                self.sp.write_reg(val);
            }
            WideReg::PC=>{
                self.pc.write_reg(val);
            }
        }
    }

    fn get_flag(&self, f: Flag) -> bool{
        match f{ //and with relevant bit
            Flag::Z =>{
                (self.af.read_reg() & (1 << 7)) == 0
            }
            Flag::N =>{
                (self.af.read_reg() & (1 << 6)) == 0
            }
            Flag::H =>{
                (self.af.read_reg() & (1 << 5)) == 0
            }
            Flag::C =>{
                (self.af.read_reg() & (1 << 4)) == 0
            }
        }
    }

    fn set_flag(&mut self, f : Flag, val: bool){
        match f{ //bitmask out bit then and with boolean
            Flag::Z =>{
                self.af.write_reg(self.af.read_reg() & 0xFF7F | ((val as u16) << 7));
            }
            Flag::N =>{
                self.af.write_reg(self.af.read_reg() & 0xFFBF | ((val as u16) << 6));
            }
            Flag::H=>{
                self.af.write_reg(self.af.read_reg() & 0xFFDF | ((val as u16) << 5));
            }
            Flag::C =>{
                self.af.write_reg(self.af.read_reg() & 0xFFEF | ((val as u16) << 4));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flags(){
        let mut cpu = CPU::new();
        cpu.set_flag(Flag::Z, true);
        cpu.set_flag(Flag::N, false);
        cpu.set_flag(Flag::H, true);
        cpu.set_flag(Flag::C, false);
        assert_eq!(true, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::C));
    }
}