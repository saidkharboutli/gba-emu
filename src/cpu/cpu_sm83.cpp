#include "cpu_sm83.hpp"

SM83::SM83(Bus *bus) {
    // NOLINT(*-pro-type-member-init)
    SM83::reg = {};
    SM83::bus = bus;

    SM83::instr = 0x00;
    SM83::data = 0x00;

    SM83::instr_cycles_left = 0;
    SM83::t_cycle = 0x00;
    SM83::m_cycle = 0x00;
}

uint8_t SM83::fetch() {
    data = bus->read(reg.PC);

    if (!instr_cycles_left) instr = data;
    reg.PC++;

    // if (!(reg.PC % 100)) std::cout << "PC: " << reg.PC << std::endl;

    return 0;
}

/* DO NOT NEED TO DECODE? */
// uint8_t SM83::decode(uint8_t instr) { reg.pc++; }

/*
 * Takes data read from memory and the instr class uint8_t and performs if/else
 * chain comparison to convert opcodes to their respective operations. Octal
 * representation is used for opcodes, due to the Game Boy developers designing
 * the opcodes around octal groupings.
 *
 * @param uint8_t data - the data read from memory
 * @returns uint8_t error - error code
 */
uint8_t SM83::execute(uint8_t decoded) {
    if (instr == 0000) {
        /* NOP */
    } else if (instr == 0001) {
        /* LD BC, n16 */
    }

    return 0;
}
