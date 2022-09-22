use crate::register::Register;
use crate::ram::RAM;

#[derive(Clone, Copy)]
pub enum WideReg {
    AF,
    BC,
    DE,
    HL,
    SP,
    PC
}

#[derive(Clone, Copy)]
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
    ime: bool,
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
            ime: false,
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
        let instruction = self.immediate_narrow();

        //nibbles
        let op_x = (instruction & 0o300) >> 6;
        let op_y = (instruction & 0o070) >> 3;
        let op_z = instruction & 0o007;

        //commonly encountered expressions
        let op_p = op_y >> 1;
        let op_q = op_y % 2;

        //decode/exec
        match (op_x, op_y, op_z){
            // NOP, does absolutely nothing
            (0, 0, 0) => {}

            // LD (nn), SP load intermediate stack pointer
            (0, 1, 0) => {
                //fetch
                let nn = self.immediate_wide();
                let sp = self.read_wide_reg(WideReg::SP);
                self.ram.write_byte(nn, (sp & 0xFF) as u8);
                self.ram.write_byte(nn + 1, (sp >> 8) as u8);
            }

            // STOP
            (0, 2, 0) => { todo!()}

            // JR d
            (0, 3, 0) => {
                //fetch
                let e8 = self.immediate_narrow();
                let pc = self.read_wide_reg(WideReg::PC);
                self.write_wide_reg(WideReg::PC, pc + e8);
            }

            // JR condition, d
            (0, 4..=7, 0) => {
                //fetch
                let e8 = self.immediate_narrow();
                if self.condition_lookup(op_y - 4) {
                    let pc = self.read_wide_reg(WideReg::PC);
                    self.write_wide_reg(WideReg::PC, pc + e8);
                }
            }
            // LD Wide and ADD HL
            (0, _, 1) => {
                if op_q == 0 { // LD Wide
                    let reg = CPU::wide_reg_lookup_1(op_p);
                    //fetch
                    let nn = self.immediate_wide();
                    self.write_wide_reg(reg, nn);
                } else { // ADD HL
                    let hl_val = self.read_wide_reg(WideReg::HL);
                    let reg = CPU::wide_reg_lookup_1(op_p);
                    let rpp = self.read_wide_reg(reg);
                    let res = hl_val.wrapping_add(rpp);
                    self.set_flag(Flag::N, false);
                    self.set_flag(Flag::H, ((hl_val & 0xFFF) + (rpp & 0xFFF)) > 0xFFF); // take lowest 12 bits, add, see if overflow
                    let carry_flag = ((hl_val as u32 & 0xFFFF) + (rpp as u32 & 0xFFFF)) > 0xFFFF; //cast to u16 and similar as above
                    self.set_flag(Flag::C, carry_flag);
                    self.write_wide_reg(WideReg::HL, res);
                }
            }

            // Indirect Loads
            (0, _, 2) => {
                let reg = match op_p {
                    0 => {
                        self.read_wide_reg(WideReg::BC)
                    }
                    1 => {
                        self.read_wide_reg(WideReg::DE)
                    }
                    2 => {
                        let reg_before = self.read_wide_reg(WideReg::HL);
                        self.write_wide_reg(WideReg::HL, reg_before.wrapping_add(1));
                        reg_before
                    }
                    3 => {
                        let reg_before = self.read_wide_reg(WideReg::HL);
                        self.write_wide_reg(WideReg::HL, reg_before.wrapping_sub(1));
                        reg_before
                    }
                    _ => panic!("Should not be reachable")
                };
                if op_q == 0 {
                    let a_val = self.read_narrow_reg(NarrowReg::A);
                    self.ram.write_byte(reg, a_val);
                } else {
                    let reg_ind = self.ram.read_byte(reg);
                    self.write_narrow_reg(NarrowReg::A, reg_ind);
                }
            }

            // INC/DEC Wide
            (0, _, 3) => {
                if op_q == 0 { //INC
                    let reg = CPU::wide_reg_lookup_1(op_p);
                    let ry = self.read_wide_reg(reg);
                    self.write_wide_reg(reg, ry.wrapping_add(1));
                } else { //DEC
                    let reg = CPU::wide_reg_lookup_1(op_p);
                    let ry = self.read_wide_reg(reg);
                    self.write_wide_reg(reg, ry.wrapping_sub(1));
                }
            }

            // INC Narrow
            (0, _, 4) => {
                let reg = CPU::reg_lookup(op_y);
                let ry = self.read_narrow_reg(reg);
                self.set_flag(Flag::N, false);
                if (ry & 0xF) == 0xF {
                    self.set_flag(Flag::H, true);
                } else {
                    self.set_flag(Flag::H, false)
                }
                if ry == 0xFF {
                    self.set_flag(Flag::Z, true);
                    self.write_narrow_reg(reg, 0);
                } else {
                    self.set_flag(Flag::Z, false);
                    self.write_narrow_reg(reg, ry + 1);
                }
            }

            // DEC Narrow
            (0, _, 5) => {
                let reg = CPU::reg_lookup(op_y);
                let ry = self.read_narrow_reg(reg);
                self.set_flag(Flag::N, true);
                if (ry & 0xF) == 0 {
                    self.set_flag(Flag::H, true);
                } else {
                    self.set_flag(Flag::H, false)
                }
                if ry == 1 {
                    self.set_flag(Flag::Z, true)
                } else {
                    self.set_flag(Flag::Z, false);
                }
                self.write_narrow_reg(reg, ry.wrapping_sub(1));
            }

            // LD 8-bit immediate
            (0, _, 6) => {
                let reg = CPU::reg_lookup(op_y);
                //fetch
                let n = self.immediate_narrow();
                self.write_narrow_reg(reg, n);
            }

            // RLCA, RotateLeftCarryA
            (0, 0, 7) => {
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let left_bit = (a_val & 0x80) != 0;
                self.set_flag(Flag::C, left_bit);
                self.write_narrow_reg(NarrowReg::A, (a_val << 1) + left_bit as u8);
            }
            // RRCA, RotateRightCarryA
            (0, 1, 7) => {
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let right_bit = (a_val & 1) != 0;
                self.set_flag(Flag::C, right_bit);
                self.write_narrow_reg(NarrowReg::A, (a_val >> 1) + (0x80 * right_bit as u8));
            }
            // RLA, RotateLeftA
            (0, 2, 7) => {
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let c_bit = self.get_flag(Flag::C);
                self.set_flag(Flag::C, (a_val & 0x80) != 0); // left bit to C
                self.write_narrow_reg(NarrowReg::A, (a_val << 1) + c_bit as u8);
            }
            // RRA, RotateRightA
            (0, 3, 7) => {
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let c_bit = self.get_flag(Flag::C);
                self.set_flag(Flag::C, (a_val & 1) != 0); // right bit to C
                self.write_narrow_reg(NarrowReg::A, (a_val >> 1) + (0x80 * c_bit as u8));
            }
            //DAA (the DEVIL addition adjustment) // based on nesdev forum post
            (0, 4, 7) => {
                let add = !self.get_flag(Flag::N);
                let mut a_val = self.read_narrow_reg(NarrowReg::A);
                if add {
                    if self.get_flag(Flag::C) || (a_val > 0x99) {
                        a_val = a_val.wrapping_add(0x60);
                        self.set_flag(Flag::C, true);
                    }
                    if self.get_flag(Flag::H) || ((a_val & 0x0F) > 0x09) {
                        a_val = a_val.wrapping_add(0x06);
                    }
                } else {
                    if self.get_flag(Flag::C) {
                        a_val = a_val.wrapping_sub(0x60);
                    }
                    if self.get_flag(Flag::H) {
                        a_val = a_val.wrapping_sub(0x06);
                    }
                }
                self.write_narrow_reg(NarrowReg::A, a_val);
                self.set_flag(Flag::Z, a_val == 0);
                self.set_flag(Flag::H, false);
            }
            // CPL - ComPLement Accumulator (needs test)
            (0, 5, 7) => {
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::H, true);
                let acc_compl = 0xFF - self.read_narrow_reg(NarrowReg::A);
                self.write_narrow_reg(NarrowReg::A, acc_compl);
            }
            // SCF - SetCarryFlag (needs test)
            (0, 6, 7) => {
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, true);
            }
            // CCF - ComplementCarryFlag (needs test)
            (0, 7, 7) => {
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, !self.get_flag(Flag::C));
            }
            // HALT
            (1, 6, 6) => {
                todo!();
            }
            // LD from one register to another
            (1, _, _) => {
                let ry = CPU::reg_lookup(op_y);
                let rz = CPU::reg_lookup(op_z);
                self.write_narrow_reg(ry, self.read_narrow_reg(rz));
            }

            //various arithmetic operations
            (2, _, _) => {
                let reg = CPU::reg_lookup(op_z);
                let rz = self.read_narrow_reg(reg);
                self.arith_lookup_exec(op_y, rz);
            }
            // Conditional RETs
            (3, 0..=3, 0) => {

            }
            // Conditional JPs, some loads
            (3, 0..=3, 2) => {
                let nn = self.immediate_wide();
                if self.condition_lookup(op_y) {
                    self.pc.write_reg(nn);
                }
            }
            //JP nn
            (3, 0, 3) => {
                let nn = self.immediate_wide();
                self.pc.write_reg(nn);
            }
            // CB
            (3, 1, 3) => {
                todo!()
            }
            // DI
            (3, 6, 3) => {
                self.ime = false;
            }
            // EI
            (3, 7, 3) => {
                self.ime = true;
            }
            // catch
            (3, _, 3) => {
                panic!("Removed instructions")
            }
            // conditional PUSH
            (3, _, 4) => {
                // removed instructions will result in panic at condition check
                // fetch
                let nn = self.immediate_wide();
                // call
                if self.condition_lookup(op_y){
                    let pc = self.read_wide_reg(WideReg::PC);
                    self.push_stack(pc);
                    self.write_wide_reg(WideReg::PC, nn);
                }
            }
            // PUSH, calls
            (3, _, 5) => {
                if op_q == 0 {
                    let rp2_reg = self.read_wide_reg(CPU::wide_reg_lookup_2(op_p));
                    self.push_stack(rp2_reg);
                } else { // op_q == 1
                    if op_p == 1 { // CALL nn
                        // fetch
                        let nn = self.immediate_wide();
                        // call
                        let pc = self.read_wide_reg(WideReg::PC);
                        self.push_stack(pc);
                        self.write_wide_reg(WideReg::PC, nn);
                    } else {
                        panic!("Invalid instruction")
                    }
                }
            }
            // Arithmetic, immediate
            (3, _, 6) => {
                //fetch
                let n8 = self.immediate_narrow();
                self.arith_lookup_exec(op_y, n8);
            }
            // ReSTart
            (3, _, 7) => {
                let p = op_y * 8;
                let pc = self.read_wide_reg(WideReg::PC);
                self.push_stack(pc);
                self.write_wide_reg(WideReg::PC, p as u16);
            }
            _ => {
                panic!("Missing Opcode {:#05o}", instruction);
            }
        }
    }
    fn read_narrow_reg(&self, reg: NarrowReg) -> u8{
        match reg{
            NarrowReg::A => self.af.read_high(),
            NarrowReg::B => self.bc.read_high(),
            NarrowReg::C => self.bc.read_low(),
            NarrowReg::D => self.de.read_high(),
            NarrowReg::E => self.de.read_low(),
            NarrowReg::H => self.hl.read_high(),
            NarrowReg::L => self.hl.read_low(),
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
            NarrowReg::A => {self.af.write_high(val);}
            NarrowReg::B => {self.bc.write_high(val); }
            NarrowReg::C => {self.bc.write_low(val);}
            NarrowReg::D => {self.de.write_high(val);}
            NarrowReg::E => {self.de.write_low(val);}
            NarrowReg::H => {self.hl.write_high(val);}
            NarrowReg::L => {self.hl.write_low(val);}
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
        match f{
            //bitmask out bit then and with boolean
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

    fn arith_lookup_exec(&mut self, index: u8, number: u8){
        match index{
            0 => { // ADD
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let res = a_val.wrapping_add(number);
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, ((a_val & 0xF) + (number & 0xF)) > 0xF); // take lowest 4 bits, add, see if overflow
                let carry_flag = ((a_val as u16 & 0xFF) + (number as u16 & 0xFF)) > 0xFF; //cast to u16 and similar as above
                self.set_flag(Flag::C, carry_flag);
                self.write_narrow_reg(NarrowReg::A, res);
            },
            1 => { // ADC
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let c_bit = self.get_flag(Flag::C);
                let res = a_val.wrapping_add(number).wrapping_add(c_bit as u8);
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, ((a_val & 0xF) + (number & 0xFF) + c_bit as u8) > 0xF); //as above
                let carry_flag = ((a_val as u16 & 0xFF) + (number as u16 & 0xFF) + c_bit as u16) > 0xFF;
                self.set_flag(Flag::C, carry_flag);
                self.write_narrow_reg(NarrowReg::A, res);
            },
            2 => { // SUB
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let res = a_val.wrapping_sub(number);
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::H, (a_val & 0xF) < (number & 0xF)); //underflow simply if one is smaller than other
                self.set_flag(Flag::C, a_val < number);
                self.write_narrow_reg(NarrowReg::A, res);
            },
            3 => { // SUBC
                let a_val = self.read_narrow_reg(NarrowReg::A);
                let c_bit = self.get_flag(Flag::C);
                let res = a_val.wrapping_sub(number).wrapping_sub(c_bit as u8);
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::H, (a_val & 0xF) < ((number & 0xF) + 1));
                self.set_flag(Flag::C, (a_val as u16) < (number as u16 + c_bit as u16));
                self.write_narrow_reg(NarrowReg::A, res);
            },
            4 => { // AND
                let res = self.read_narrow_reg(NarrowReg::A) & number;
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, true);
                self.set_flag(Flag::C, false);
                self.write_narrow_reg(NarrowReg::A, res);
            },
            5 => { // XOR
                let res = self.read_narrow_reg(NarrowReg::A) ^ number;
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, false);
                self.write_narrow_reg(NarrowReg::A, res);
            },
            6 => { // OR
                let res = self.read_narrow_reg(NarrowReg::A) | number;
                self.set_flag(Flag::Z, (res == 0));
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, false);
                self.write_narrow_reg(NarrowReg::A, res);
            },
            7 => { // CP
                let a_val = self.read_narrow_reg(NarrowReg::A);
                self.set_flag(Flag::Z, a_val == number);
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::H, (a_val & 0xF) < (number & 0xF)); //underflow simply if one is smaller than other
                self.set_flag(Flag::C, a_val < number);
            },
            _ => panic!("Should not be reachable")
        }
    }
    fn immediate_narrow(&mut self) -> u8{
        let curr_address = self.pc.read_reg();
        self.pc.next_instruction();
        self.ram.read_byte( curr_address)
    }
    fn immediate_wide(&mut self) -> u16{
        let curr_address = self.pc.read_reg();
        self.pc.next_instruction();
        self.pc.next_instruction();
        self.ram.read_word( curr_address)
    }
    fn push_stack(&mut self, nn: u16){
        let mut sp = self.read_wide_reg(WideReg::SP);
        sp = sp - 1;
        self.ram.write_byte(sp, (nn >> 8) as u8);
        sp = sp - 1;
        self.ram.write_byte(sp, (nn & 0xFF) as u8);
        self.write_wide_reg(WideReg::SP, sp);
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

    #[test]
    fn add_reg_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o201);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0x88);
        cpu.write_narrow_reg(NarrowReg::C, 0x88);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0x10, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn adc_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o211);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0x88);
        cpu.write_narrow_reg(NarrowReg::C, 0x87);
        cpu.set_flag(Flag::C, true);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0x10, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn sub_reg_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o221);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0x88);
        cpu.write_narrow_reg(NarrowReg::C, 0x89);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(true, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0xFF, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn subc_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o231);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0x88);
        cpu.write_narrow_reg(NarrowReg::C, 0x88);
        cpu.set_flag(Flag::C, true);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(true, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0xFF, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn and_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o241);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b10011010);
        cpu.write_narrow_reg(NarrowReg::C, 0b01100101);
        cpu.cycle();
        assert_eq!(true, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::C));
        assert_eq!(0, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn xor_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o251);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b10011010);
        cpu.write_narrow_reg(NarrowReg::C, 0b01100101);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(false, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::C));
        assert_eq!(0b11111111, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn or_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o261);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b10011010);
        cpu.write_narrow_reg(NarrowReg::C, 0b00110011);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(false, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::C));
        assert_eq!(0b10111011, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn cp_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o271);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0x3A);
        cpu.write_narrow_reg(NarrowReg::C, 0x3B);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(true, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0x3A, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn hl_ind_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o206);
        ram.write_byte(0xC000, 0x88);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0x88);
        cpu.write_wide_reg(WideReg::HL, 0xC000);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::Z));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0x10, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn ld_reg_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o101);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::B, 0xAB);
        cpu.write_narrow_reg(NarrowReg::C, 0xCD);
        cpu.cycle();
        assert_eq!(0xCD, cpu.read_narrow_reg(NarrowReg::B));
    }

    #[test]
    fn rlca_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o007);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b00100011);
        cpu.set_flag(Flag::C, true);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::C));
        assert_eq!(0b01000110, cpu.read_narrow_reg(NarrowReg::A));
    }

    #[test]
    fn rrca_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o017);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b00100011);
        cpu.set_flag(Flag::C, false);
        cpu.cycle();
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0b10010001, cpu.read_narrow_reg(NarrowReg::A));
    }

    #[test]
    fn rla_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o027);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b10100011);
        cpu.set_flag(Flag::C, false);
        cpu.cycle();
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0b01000110, cpu.read_narrow_reg(NarrowReg::A));
    }

    #[test]
    fn rra_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o037);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b10100011);
        cpu.set_flag(Flag::C, false);
        cpu.cycle();
        assert_eq!(true, cpu.get_flag(Flag::C));
        assert_eq!(0b01010001, cpu.read_narrow_reg(NarrowReg::A));
    }
    #[test]
    fn cpl_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o057);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::A, 0b10100011);
        cpu.cycle();
        assert_eq!(0b01011100, cpu.read_narrow_reg(NarrowReg::A));
    }

    #[test]
    fn ccf_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o067);
        let mut cpu = CPU::new(ram);
        cpu.set_flag(Flag::N, true);
        cpu.set_flag(Flag::H, true);
        cpu.set_flag(Flag::C, true);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(false, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::C));
    }

    #[test]
    fn scf_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o077);
        let mut cpu = CPU::new(ram);
        cpu.set_flag(Flag::N, true);
        cpu.set_flag(Flag::H, true);
        cpu.set_flag(Flag::C, false);
        cpu.cycle();
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(false, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::C));
    }

    #[test]
    fn inc_wide_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o003);
        let mut cpu = CPU::new(ram);
        cpu.write_wide_reg(WideReg::BC, 0xABCD);
        cpu.cycle();
        assert_eq!(0xABCE, cpu.read_wide_reg(WideReg::BC));
    }

    #[test]
    fn dec_wide_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o013);
        let mut cpu = CPU::new(ram);
        cpu.write_wide_reg(WideReg::BC, 0xABCD);
        cpu.cycle();
        assert_eq!(0xABCC, cpu.read_wide_reg(WideReg::BC));
    }

    #[test]
    fn inc_narrow_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o004);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::B, 0xFF);
        cpu.set_flag(Flag::N, true);
        cpu.set_flag(Flag::H, false);
        cpu.set_flag(Flag::Z, false);
        cpu.cycle();
        assert_eq!(0x00, cpu.read_narrow_reg(NarrowReg::B));
        assert_eq!(false, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(true, cpu.get_flag(Flag::Z));
    }

    #[test]
    fn dec_narrow_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o005);
        let mut cpu = CPU::new(ram);
        cpu.write_narrow_reg(NarrowReg::B, 0xD0);
        cpu.set_flag(Flag::N, false);
        cpu.set_flag(Flag::H, false);
        cpu.set_flag(Flag::Z, true);
        cpu.cycle();
        assert_eq!(0xCF, cpu.read_narrow_reg(NarrowReg::B));
        assert_eq!(true, cpu.get_flag(Flag::N));
        assert_eq!(true, cpu.get_flag(Flag::H));
        assert_eq!(false, cpu.get_flag(Flag::Z));
    }

    #[test]
    fn ld_narrow_test() {
        let mut ram = RAM::new();
        ram.write_byte(0x100, 0o006);
        ram.write_byte(0x101, 0xAB);
        let mut cpu = CPU::new(ram);
        cpu.cycle();
        assert_eq!(0xAB, cpu.read_narrow_reg(NarrowReg::B));
    }
}