#ifndef CPU_SM_83_HPP
#define CPU_SM_83_HPP

#include <cstdint>

#include "bus/bus.hpp"

enum Flag { FLAG_Z = 7, FLAG_N = 6, FLAG_H = 5, FLAG_C = 4 };

typedef struct RegisterFile {
    /* 16-bit: AF */
    uint8_t a;
    uint8_t f;

    /* 16-bit: BC */
    uint8_t b;
    uint8_t c;

    /* 16-bit: DE */
    uint8_t d;
    uint8_t e;

    /* 16-bit: HL */
    uint8_t h;
    uint8_t l;

    uint16_t sp;

    uint16_t pc;
} RegisterFile;

class SM83 {
   private:
    Bus* bus;
    RegisterFile reg;
    uint8_t instr;

   public:
    SM83(Bus* bus);

    /* MAIN LOOP */
    uint8_t fetch();
    uint8_t decode(uint8_t instr);
    uint8_t execute(uint8_t decoded);

    /* ARITH HELPERS */
    inline void set_flag(uint8_t flag, bool value) {
        reg.f = (reg.f & ~((uint8_t)1 << flag)) | ((uint8_t)value << flag);
    }

    ~SM83();
};

#endif