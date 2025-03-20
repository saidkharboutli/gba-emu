#include "cpu_sm83.hpp"

SM83::SM83(Bus* bus) {
    SM83::reg = {};
    SM83::bus = bus;
}

uint8_t SM83::fetch() {
    instr = bus->read(reg.PC);
    reg.PC++;
    return instr;
}

/* DO NOT NEED TO DECODE? */
// uint8_t SM83::decode(uint8_t instr) { reg.pc++; }

uint8_t SM83::execute(uint8_t decoded) {
    if (instr == 0x00) {
        /* NOP */
    } else if (instr == 0x01) {
        /* LD BC, n16 */
    }
}

SM83::~SM83() {}
