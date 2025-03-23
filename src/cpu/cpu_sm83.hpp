#ifndef CPU_SM_83_HPP
#define CPU_SM_83_HPP

#include <cstdint>

#include "bus/bus.hpp"

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

    /*
     * LD FUNCTION HELPERS
     * Read register x0-7 relative to octal groups of instruction opcode table
     */
    uint8_t s_regB() { return reg.B; }
    uint8_t s_regC() { return reg.C; }
    uint8_t s_regD() { return reg.D; }
    uint8_t s_regE() { return reg.E; }
    uint8_t s_regH() { return reg.H; }
    uint8_t s_regL() { return reg.L; }
    uint8_t s_regA() { return reg.A; }
    uint8_t s_regHL_ind() { return bus->read(reg.HL); }
    uint8_t (SM83::* source_select[8])() = {
        &SM83::s_regB, &SM83::s_regC, &SM83::s_regD, &SM83::s_regE,
        &SM83::s_regH, &SM83::s_regL, &SM83::s_regA, &SM83::s_regHL_ind};

    uint8_t d_regB(uint8_t data) { reg.B = data; }
    uint8_t d_regC(uint8_t data) { reg.C = data; }
    uint8_t d_regD(uint8_t data) { reg.D = data; }
    uint8_t d_regE(uint8_t data) { reg.E = data; }
    uint8_t d_regH(uint8_t data) { reg.H = data; }
    uint8_t d_regL(uint8_t data) { reg.L = data; }
    uint8_t d_regA(uint8_t data) { reg.A = data; }
    uint8_t d_regHL_ind(uint8_t data) { bus->write(reg.HL, data); }
    uint8_t (SM83::* dest_select[8])(uint8_t) = {
        &SM83::d_regB, &SM83::d_regC, &SM83::d_regD, &SM83::d_regE,
        &SM83::d_regH, &SM83::d_regL, &SM83::d_regA, &SM83::d_regHL_ind};

    /* ARITH HELPERS */
    inline void set_flag(uint8_t flag, bool value) {
        reg.F = (reg.F & ~((uint8_t)1 << flag)) | ((uint8_t)value << flag);
    }

    /* INSTRUCTION GROUPS */
    inline void LD_r8r8(uint8_t* regD, uint8_t* regS) { *regD = *regS; }
    inline void LD_r8n8(uint8_t* regD, uint8_t data) { *regD = data; }
    inline void LD_r16n16(uint16_t* regD, uint16_t data) { *regD = data; }
    inline void LD_ind_r16r8(uint8_t reg_select, uint8_t* regS) {
        (this->*dest_select[reg_select])(*regS);
    }
    inline void LD_ind_r16n8(uint8_t reg_select, uint8_t data) {
        (this->*dest_select[reg_select])(data);
    }

   public:
    SM83(Bus* bus);

    /* MAIN LOOP */
    uint8_t fetch();
    void decode(uint8_t instr);
    uint8_t execute(uint8_t decoded);

    ~SM83();
};

#endif