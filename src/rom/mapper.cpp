#include "mapper.hpp"

/*
 * The mapper is the interface between the CPU and the ROM. Calls to read and
 * write go through the mapper to the ROM banks.
 */

ROMMapper::ROMMapper() {
    rom_bank0 = new uint8_t[0x4000];
    rom_bankN = new uint8_t[0x4000];

    eram = new uint8_t[0x2000];
}

uint8_t ROMMapper::read_rom(const uint16_t addr, uint8_t bank) {
    // bank undefined for now
    if (addr < 0x4000) {
        return rom_bank0[addr];
    }
    return rom_bankN[addr - 0x4000];
}

uint8_t ROMMapper::read_eram(const uint16_t addr) { return eram[addr - ERAM_START]; }

void ROMMapper::write_eram(const uint16_t addr, const uint8_t data) {
    eram[addr - ERAM_START] = data;
}

ROMMapper::~ROMMapper() {
    delete rom_bank0;
    delete rom_bankN;
    delete eram;
}
