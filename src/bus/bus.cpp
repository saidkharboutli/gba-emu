#include "bus.hpp"

Bus::Bus(InternalMemoryUnit* imu) { Bus::imu = imu; }

/*
    Current implementation isn't great, most common access ranges should be
    checked first for maximum performance.
*/
uint8_t Bus::read(uint8_t addr) {
    if (addr > 0xFFFF) {
        // This check happens every cycle, but probably never actually hits
        std::cout << "Invalid address" << std::endl;
    } else if (addr == IE_REGISTER) {
        /* IE Register */
    } else if (addr >= HRAM_START) {
        /* HRAM */
    } else if (addr >= IO_REGISTERS_START) {
        /* I/O Registers */
    } else if (addr >= UNUSABLE_START) {
        /* Unusable */
    } else if (addr >= OAM_START) {
        /* OAM */
    } else if (addr >= ECHO_RAM_START) {
        /* Echo RAM */
    } else if (addr >= IWRAM_START) {
        /* IWRAM */
        return Bus::imu->read(addr - IWRAM_START);
    } else if (addr >= EWRAM_START) {
        /* EWRAM */
    } else if (addr >= VRAM_START) {
        /* VRAM */
    } else if (addr >= ROM_START) {
        /* ROM */
    } else if (addr >= 0x0000) {
        /* BIOS */
    } else {
        std::cout << "Invalid address" << std::endl;
    }

    return 0;  // 0 is not satisfying to indicate error
}

void Bus::write(uint8_t addr) {
    if (addr > 0xFFFF) {
        // This check happens every cycle, but probably never actually hits
        std::cout << "Invalid address" << std::endl;
    } else if (addr == IE_REGISTER) {
        /* IE Registers */
    } else if (addr >= HRAM_START) {
        /* HRAM */
    } else if (addr >= IO_REGISTERS_START) {
        /* I/O Registers */
    } else if (addr >= UNUSABLE_START) {
        /* Unusable */
    } else if (addr >= OAM_START) {
        /* OAM */
    } else if (addr >= ECHO_RAM_START) {
        /* Echo RAM */
    } else if (addr >= IWRAM_START) {
        /* IWRAM */
        Bus::imu->write(addr - IWRAM_START);
        return;
    } else if (addr >= EWRAM_START) {
        /* EWRAM */
    } else if (addr >= VRAM_START) {
        /* VRAM */
    } else if (addr >= ROM_START) {
        /* ROM */
    } else if (addr >= 0x0000) {
        /* BIOS */
    } else {
        std::cout << "Invalid address" << std::endl;
    }
}

Bus::~Bus() {}