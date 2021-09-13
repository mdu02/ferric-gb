pub struct Register {
    value: u16
}

// note that technically registers are stored little-endian in the z80
// however, since we only interact through interfaces, our internal structure is big-endian
// if this causes trouble in the future, please fix
impl Register {
    pub fn new(value: u16) -> Register {
        let mut reg: Register = Register{value};
        reg
    }

    pub fn read_reg(&self) -> u16 {
        self.value
    }

    pub fn write_reg(&mut self, val: u16) {
        self.value = val
    }

    pub fn read_high(&self) -> u8 {
        (self.value >> 8) as u8
    }

    pub fn write_high(&mut self, val: u8) {
        self.value = self.value & 0x00FF | ((val as u16) << 8);
    }

    pub fn read_low(&self) -> u8 {
        (self.value & 0xFF) as u8
    }

    pub fn write_low(&mut self, val: u8) {
        self.value = self.value & 0xFF00 | (val as u16);
    }
    pub fn next_instruction(&mut self) { //do i REALLY need to test this...
        self.value += 1;
    }

}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_write(){
        let mut reg = Register::new(0);
        reg.write_reg(0xABCD);
        assert_eq!(0xABCD, reg.read_reg());
    }

    #[test]
    fn read_high_test(){
        let mut reg = Register::new(0);
        reg.write_reg(0xCA32);
        assert_eq!(0xCA, reg.read_high());
    }

    #[test]
    fn write_high_test(){
        let mut reg = Register::new(0);
        reg.write_reg(0xC6E3);
        reg.write_high(0xA5);
        assert_eq!(0xA5E3, reg.read_reg());
    }

    #[test]
    fn read_low_test(){
        let mut reg = Register::new(0);
        reg.write_reg(0x2032);
        assert_eq!(0x32, reg.read_low());
    }

    #[test]
    fn write_low_test(){
        let mut reg = Register::new(0);
        reg.write_reg(0xE7C1);
        reg.write_low(0xA2);
        assert_eq!(0xE7A2, reg.read_reg());
    }

}
