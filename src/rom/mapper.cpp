#include "mapper.hpp"

/*
 * The mapper is the interface between the CPU and the ROM. Calls to read and
 * write go through the mapper to the ROM banks.
 */

ROMMapper::ROMMapper() {
    rom_bank0 = (uint8_t*)malloc(0x4000);
    rom_bankN = (uint8_t*)malloc(0x4000);
}

uint8_t ROMMapper::read(uint16_t addr, uint8_t bank) {
    // bank undefined for now

    if (addr < 0x4000) {
        return rom_bank0[addr];
    }
    return rom_bankN[addr - 0x4000];
}

ROMMapper::~ROMMapper() {
    delete rom_bank0;
    delete rom_bankN;
}