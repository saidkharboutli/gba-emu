#include "cpu_sm83.hpp"

SM83::SM83(Bus* bus) {
    SM83::reg = {};
    SM83::bus = bus;
}

uint8_t SM83::fetch() {
    uint8_t instr = bus->read(reg.pc);
    return instr;
}

uint8_t SM83::decode(uint8_t instr) { reg.pc++; }

uint8_t SM83::execute(uint8_t decoded) {}

SM83::~SM83() {}
