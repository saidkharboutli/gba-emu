#ifndef GBA_EMU_TEST_CPU_SM83_HPP
#define GBA_EMU_TEST_CPU_SM83_HPP
#include <cstdint>

#include "../bus/bus.hpp"
#include "../constants.hpp"

enum Flag { FLAG_Z = 7, FLAG_N = 6, FLAG_H = 5, FLAG_C = 4 };

typedef struct RegisterFile {
    /* 16-bit: AF */
    union {
        struct {
            uint8_t A;
            uint8_t F;
        };
        uint16_t AF;
    };

    /* 16-bit: BC */
    union {
        struct {
            uint8_t B;
            uint8_t C;
        };
        uint16_t BC;
    };

    /* 16-bit: DE */
    union {
        struct {
            uint8_t D;
            uint8_t E;
        };
        uint16_t DE;
    };

    /* 16-bit: HL */
    union {
        struct {
            uint8_t H;
            uint8_t L;
        };
        uint16_t HL;
    };

    /* 16-bit Stack Pointer */
    uint16_t SP;

    /* 16-bit Program Counter */
    uint16_t PC;

} RegisterFile;

class SM83 {
   private:
    Bus* bus;

    RegisterFile reg;

    uint8_t instr;
    uint8_t data;

    uint8_t instr_cycles_left;
    uint64_t t_cycle;
    uint64_t m_cycle;

    /*************************************************************************
     * MISC HELPERS
     *************************************************************************/
    inline void set_flag(uint8_t flag, uint8_t value) {
        reg.F = (reg.F & ~((uint8_t)1 << flag)) | ((uint8_t)value << flag);
    }
    inline uint8_t get_flag(uint8_t flag) {
        return reg.F & ((uint8_t)1 << flag);
    }

    /*************************************************************************
     * REGISTER SOURCE AND DESTINATION SELECTORS
     * Select register based on x0-7 octal groups on opcode table
     *************************************************************************/
    uint8_t s_regB() { return reg.B; }
    uint8_t s_regC() { return reg.C; }
    uint8_t s_regD() { return reg.D; }
    uint8_t s_regE() { return reg.E; }
    uint8_t s_regH() { return reg.H; }
    uint8_t s_regL() { return reg.L; }
    uint8_t s_regA() { return reg.A; }
    uint8_t s_regHL_ind() { return bus->read(reg.HL); }
    uint8_t (SM83::* source_select[8])() = {
        &SM83::s_regB, &SM83::s_regC, &SM83::s_regD,      &SM83::s_regE,
        &SM83::s_regH, &SM83::s_regL, &SM83::s_regHL_ind, &SM83::s_regA};

    void d_regB(uint8_t n8) { reg.B = n8; }
    void d_regC(uint8_t n8) { reg.C = n8; }
    void d_regD(uint8_t n8) { reg.D = n8; }
    void d_regE(uint8_t n8) { reg.E = n8; }
    void d_regH(uint8_t n8) { reg.H = n8; }
    void d_regL(uint8_t n8) { reg.L = n8; }
    void d_regA(uint8_t n8) { reg.A = n8; }
    void d_regHL_ind(uint8_t n8) { bus->write(reg.HL, n8); }
    void (SM83::* dest_select[8])(uint8_t) = {
        &SM83::d_regB, &SM83::d_regC, &SM83::d_regD,      &SM83::d_regE,
        &SM83::d_regH, &SM83::d_regL, &SM83::d_regHL_ind, &SM83::d_regA};

    /*************************************************************************
     * ARITHMETIC FUNCTION SELECTOR
     * Select arithmetic function based on x0-7 octal groups on opcode table
     * TODO: Review signage for these functions and flags
     *************************************************************************/
    void ADD(uint8_t val) {
        set_flag(FLAG_C, (reg.A & val) >> 7);
        reg.A += val;
    }
    void ADC(uint8_t val) {
        if (reg.A + val + get_flag(FLAG_C) > 0xFF) {
            set_flag(FLAG_C, 1);
        }
        reg.A += val + get_flag(FLAG_C);
    }
    void SUB(uint8_t val) {
        if (val > reg.A) {
            set_flag(FLAG_C, 1);
        }
        reg.A -= val;
        set_flag(FLAG_N, 1);
    }
    void SBC(uint8_t val) {
        if (val + get_flag(FLAG_C) > reg.A) {
            set_flag(FLAG_C, 1);
        }
        reg.A -= (val + get_flag(FLAG_C));
        set_flag(FLAG_N, 1);
    }
    void AND(uint8_t val) { reg.A &= val; }
    void XOR(uint8_t val) { reg.A ^= val; }
    void OR(uint8_t val) { reg.A |= val; }
    void CP(uint8_t val) {
        if (val > reg.A)
            set_flag(FLAG_C, 1);
        else if (val == reg.A)
            set_flag(FLAG_Z, 1);
        if ((val & 0x0F) > (reg.A & 0x0F)) set_flag(FLAG_H, 1);
        set_flag(FLAG_N, 1);
    }
    void (SM83::* arith_select[8])(uint8_t) = {
        &SM83::ADD, &SM83::ADC, &SM83::SUB, &SM83::SBC,
        &SM83::AND, &SM83::XOR, &SM83::OR,  &SM83::CP};

    /*************************************************************************
     * LD INSTRUCTIONS
     *************************************************************************/
    inline void LD_rr(uint8_t reg_select_d, uint8_t reg_select_s) {
        (this->*dest_select[reg_select_d])(
            (this->*source_select[reg_select_s])());
    }
    inline void LD_rn8(uint8_t reg_select_d, uint8_t n8) {
        (this->*dest_select[reg_select_d])(n8);
    }
    inline void LD_ind_r16r8(uint16_t* regD, uint8_t* regS) {
        bus->write(*regD, *regS);
    }
    inline void LD_r8_ind_r16(uint8_t* regD, uint16_t* regS) {
        *regD = bus->read(*regS);
    }
    inline void r16n16(uint16_t* regD, uint16_t n16) { *regD = n16; }

    /* Unfortunate Single Use */
    inline void LD_ind_n16r8(uint16_t addr, uint8_t* regS) {
        bus->write(addr, *regS);
    }
    /* Unfortunate Single Use */
    inline void LD_r8_ind_n16(uint8_t* regD, uint16_t n16) {
        *regD = bus->read(n16);
    }

    /*************************************************************************
     * INC INSTRUCTIONS
     *************************************************************************/
    inline void INC_r(uint8_t reg_select) {
        // this is terrible i think
        (this->*dest_select[reg_select])((this->*source_select[reg_select])() +
                                         1);
    }
    inline void INC_r16(uint16_t* regD) { *regD++; }

    /*************************************************************************
     * DEC INSTRUCTIONS
     *************************************************************************/
    inline void DEC_r(uint8_t reg_select) {
        // this is terrible i think
        (this->*dest_select[reg_select])((this->*source_select[reg_select])() -
                                         1);
    }
    inline void DEC_r16(uint16_t* regD) { *regD--; }

    /*************************************************************************
     * ARITH INSTRUCTIONS
     *************************************************************************/
    void ARITH_rr(uint8_t op_select, uint8_t reg_select) {
        (this->*arith_select[op_select])((this->*source_select[reg_select])());
    }
    /* For OPCODES 0306 - 0376 */
    void ARITH_rn8(uint8_t op_select, uint8_t n8) {
        (this->*arith_select[op_select])(n8);
    }
    /* For 00X1*/
    void ADD_r16r16(uint16_t* regD, uint16_t* regS) { *regD += *regS; }

    /*************************************************************************
     * JUMP INSTRUCTIONS
     *************************************************************************/
    void JR(int8_t signed_offset) { reg.PC += signed_offset + 1; }
    void JR_cc(uint8_t flag_selector, int8_t signed_offset) {
        if (get_flag(signed_offset)) reg.PC += signed_offset + 1;
    }
    void JP(uint16_t addr) { reg.PC = addr; }
    void JP_cc(uint8_t flag_selector, uint16_t addr) {
        if (get_flag(flag_selector)) reg.PC = addr;
    }

   public:
    SM83(Bus* bus);

    /* MAIN LOOP */
    uint8_t fetch();
    void decode(uint8_t instr);
    uint8_t execute(uint8_t decoded);

    ~SM83();
};
#endif  // GBA_EMU_TEST_CPU_SM83_HPP
