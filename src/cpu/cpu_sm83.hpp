#ifndef CPU_SM_83_HPP
#define CPU_SM_83_HPP

#include <cstdint>

#include "bus/bus.hpp"

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

   public:
    SM83(Bus* bus);

    uint8_t fetch();

    uint8_t decode(uint8_t instr);

    uint8_t execute(uint8_t decoded);

    ~SM83();
};

#endif