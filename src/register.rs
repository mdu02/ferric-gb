pub struct Register {
    value: u16
}

impl Register {
    pub fn new(value: u16) -> Register {
        let mut reg: Register = Register{value };
        reg
    }

    pub fn read_reg(&self) -> u16 {
        self.value
    }

    pub fn write_reg(&mut self, val: u16) {
        self.value = val
    }

    pub fn read_top(&self) -> u8 {
        (self.value >> 8) as u8
    }

    pub fn write_top(&mut self, val: u8) {
        self.value = self.value & 0x00FF | ((val as u16) << 8);
    }

    pub fn read_bottom(&self) -> u8 {
        (self.value & 0xFF) as u8
    }

    pub fn write_bottom(&mut self, val: u8) {
        self.value = self.value & 0xFF00 | (val as u16);
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
    fn read_top(){
        let mut reg = Register::new(0);
        reg.write_reg(0xCA32);
        assert_eq!(0xCA, reg.read_top());
    }

    #[test]
    fn write_top(){
        let mut reg = Register::new(0);
        reg.write_reg(0xC6E3);
        reg.write_top(0xA5);
        assert_eq!(0xA5E3, reg.read_reg());
    }

    #[test]
    fn read_bottom(){
        let mut reg = Register::new(0);
        reg.write_reg(0x2032);
        assert_eq!(0x32, reg.read_bottom());
    }

    #[test]
    fn write_bottom(){
        let mut reg = Register::new(0);
        reg.write_reg(0xE7C1);
        reg.write_bottom(0xA2);
        assert_eq!(0xE7A2, reg.read_reg());
    }

}
