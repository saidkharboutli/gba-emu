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

    /* HELPERS */
    inline void set_flag(uint8_t flag, bool value) {
        reg.F = (reg.F & ~((uint8_t)1 << flag)) | ((uint8_t)value << flag);
    }

    inline void LD_mem(uint16_t addr, uint8_t data) { bus->write(addr, data); }
    inline void LD_reg(uint16_t* reg, uint8_t data) { *reg = data; }

   public:
    SM83(Bus* bus);

    /* MAIN LOOP */
    uint8_t fetch();
    void decode(uint8_t instr);
    uint8_t execute(uint8_t decoded);

    ~SM83();
};

#endif