#ifndef GBA_EMU_TEST_CPU_SM83_HPP
#define GBA_EMU_TEST_CPU_SM83_HPP
#include <cstdint>

#include "../bus/bus.hpp"

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
    Bus *bus;

    RegisterFile reg;

    uint8_t instr;
    uint8_t data;

    uint8_t instr_cycles_left;
    uint64_t t_cycle;
    uint64_t m_cycle;

    /*************************************************************************
     * MISC HELPERS
     *************************************************************************/
    uint8_t get_flag(const uint8_t flag) const {
        return reg.F & (static_cast<uint8_t>(1) << flag);
    }

    void set_flag(const uint8_t flag, const uint8_t value) {
        reg.F = (reg.F & ~(static_cast<uint8_t>(1) << flag)) | (value << flag);
    }

    void set_ZNHC(const uint8_t Z, const uint8_t N, const uint8_t H, const uint8_t C) {
        reg.F = (reg.F & 0x00) | (Z << FLAG_Z | N << FLAG_N | H << FLAG_H | C << FLAG_C);
    }

    /*************************************************************************
     * REGISTER SOURCE AND DESTINATION SELECTORS
     * Select register based on x0-7 octal groups on opcode table
     *************************************************************************/
    uint8_t s_regB() const { return reg.B; }
    uint8_t s_regC() const { return reg.C; }
    uint8_t s_regD() const { return reg.D; }
    uint8_t s_regE() const { return reg.E; }
    uint8_t s_regH() const { return reg.H; }
    uint8_t s_regL() const { return reg.L; }
    uint8_t s_regA() const { return reg.A; }
    uint8_t s_regHL_ind() const { return bus->read(reg.HL); }

    uint8_t (SM83::*source_select[8])() const = {
        &SM83::s_regB, &SM83::s_regC, &SM83::s_regD, &SM83::s_regE,
        &SM83::s_regH, &SM83::s_regL, &SM83::s_regHL_ind, &SM83::s_regA
    };

    void d_regB(const uint8_t n8) { reg.B = n8; }
    void d_regC(const uint8_t n8) { reg.C = n8; }
    void d_regD(const uint8_t n8) { reg.D = n8; }
    void d_regE(const uint8_t n8) { reg.E = n8; }
    void d_regH(const uint8_t n8) { reg.H = n8; }
    void d_regL(const uint8_t n8) { reg.L = n8; }
    void d_regA(const uint8_t n8) { reg.A = n8; }
    void d_regHL_ind(const uint8_t n8) { bus->write(reg.HL, n8); }

    void (SM83::*dest_select[8])(uint8_t) = {
        &SM83::d_regB, &SM83::d_regC, &SM83::d_regD, &SM83::d_regE,
        &SM83::d_regH, &SM83::d_regL, &SM83::d_regHL_ind, &SM83::d_regA
    };

    /*************************************************************************
     * ARITHMETIC FUNCTION SELECTOR
     * Select arithmetic function based on x0-7 octal groups on opcode table
     * TODO: Review signage for these functions and flags
     *************************************************************************/
    void ADD(const uint8_t val) {
        const uint8_t res = reg.A + val;
        set_ZNHC(res == 0, 0, 0, res < reg.A);
        reg.A = res;
    }

    void ADC(const uint8_t val) {
        const uint8_t res = reg.A + val + get_flag(FLAG_C);
        set_ZNHC(res == 0, 0, 0, res < reg.A + get_flag(FLAG_C));
        reg.A = res;
    }

    void SUB(const uint8_t val) {
        const uint8_t res = reg.A - val;
        set_ZNHC(res == 0, 1, 0, res > reg.A);
        reg.A = res;
    }

    void SBC(const uint8_t val) {
        const uint8_t res = reg.A - (val + get_flag(FLAG_C));
        set_ZNHC(res == 0, 1, 0, res > reg.A + get_flag(FLAG_C));
        reg.A = res;
    }

    void AND(const uint8_t val) {
        const uint8_t res = reg.A & val;
        set_ZNHC(res == 0, 0, 1, 0);
        reg.A = res;
    }

    void XOR(const uint8_t val) {
        const uint8_t res = reg.A ^ val;
        set_ZNHC(res == 0, 0, 0, 0);
        reg.A = res;
    }

    void OR(const uint8_t val) {
        const uint8_t res = reg.A | val;
        set_ZNHC(res == 0, 0, 0, 0);
        reg.A = res;
    }

    void CP(const uint8_t val) {
        const uint8_t res = reg.A - val;
        set_ZNHC(res == 0, 1, 0, res > reg.A);
    }

    void (SM83::*arith_select[8])(uint8_t) = {
        &SM83::ADD, &SM83::ADC, &SM83::SUB, &SM83::SBC,
        &SM83::AND, &SM83::XOR, &SM83::OR, &SM83::CP
    };

    /*************************************************************************
     * LD INSTRUCTIONS
     *************************************************************************/
    void LD_rr(const uint8_t reg_select_d, const uint8_t reg_select_s) {
        (this->*dest_select[reg_select_d])(
            (this->*source_select[reg_select_s])());
    }

    void LD_rn8(const uint8_t reg_select_d, const uint8_t n8) {
        (this->*dest_select[reg_select_d])(n8);
    }

    void LD_ind_r16r8(const uint16_t *regD, const uint8_t *regS) {
        bus->write(*regD, *regS);
    }

    void LD_r8_ind_r16(uint8_t *regD, const uint16_t *regS) {
        *regD = bus->read(*regS);
    }

    static void r16n16(uint16_t *regD, const uint16_t n16) { *regD = n16; }

    /* Unfortunate Single Use */
    void LD_ind_n16r8(const uint16_t addr, const uint8_t *regS) {
        bus->write(addr, *regS);
    }

    /* Unfortunate Single Use */
    void LD_r8_ind_n16(uint8_t *regD, const uint16_t n16) {
        *regD = bus->read(n16);
    }

    /*************************************************************************
     * INC INSTRUCTIONS
     *************************************************************************/
    void INC_r(const uint8_t reg_select) {
        // this is terrible I think
        (this->*dest_select[reg_select])((this->*source_select[reg_select])() +
                                         1);
    }

    static void INC_r16(uint16_t *regD) { (*regD)++; }

    /*************************************************************************
     * DEC INSTRUCTIONS
     *************************************************************************/
    void DEC_r(const uint8_t reg_select) {
        // this is terrible I think
        (this->*dest_select[reg_select])((this->*source_select[reg_select])() -
                                         1);
    }

    static void DEC_r16(uint16_t *regD) { (*regD)--; }

    /*************************************************************************
     * ARITH INSTRUCTIONS
     *************************************************************************/
    void ARITH_rr(const uint8_t op_select, const uint8_t reg_select) {
        (this->*arith_select[op_select])((this->*source_select[reg_select])());
    }

    /* For OPCODES 0306 - 0376 */
    void ARITH_rn8(const uint8_t op_select, const uint8_t n8) {
        (this->*arith_select[op_select])(n8);
    }

    /* For 00X1 */
    static void ADD_r16r16(uint16_t *regD, const uint16_t *regS) { *regD += *regS; }

    /*************************************************************************
     * JUMP INSTRUCTIONS
     *************************************************************************/
    void JR(const int8_t signed_offset) { reg.PC += signed_offset + 1; }

    void JR_cc(const uint8_t flag_selector, const int8_t signed_offset) {
        if (get_flag(flag_selector)) reg.PC += signed_offset + 1;
    }

    void JP(const uint16_t addr) { reg.PC = addr; }

    void JP_cc(const uint8_t flag_selector, const uint16_t addr) {
        if (get_flag(flag_selector)) reg.PC = addr;
    }

public:
    explicit SM83(Bus *bus);

    /* MAIN LOOP */
    uint8_t fetch();

    uint8_t execute(uint8_t decoded);

    ~SM83() = default;
};
#endif  // GBA_EMU_TEST_CPU_SM83_HPP
