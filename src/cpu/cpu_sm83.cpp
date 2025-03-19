#include "cpu_sm83.hpp"

SM83::SM83(Bus* bus) {
    SM83::reg = {};
    SM83::bus = bus;
}

uint8_t SM83::fetch() {
    instr = bus->read(reg.pc);
    reg.pc++;
    return instr;
}

/* DO NOT NEED TO DECODE? */
// uint8_t SM83::decode(uint8_t instr) { reg.pc++; }

uint8_t SM83::execute(uint8_t decoded) {
    switch (instr) {
        case 0x00:
            /* NOP */
            break;
        case 0x01:
            /* LD BC, n16 */
            break;
        default:
            break;
    }
}

SM83::~SM83() {}
