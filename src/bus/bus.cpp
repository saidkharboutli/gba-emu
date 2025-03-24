#include "bus.hpp"

Bus::Bus(InternalMemoryUnit* imu, ROMMapper* rom_mapper, PPU* ppu) {
    Bus::imu = imu;
    Bus::rom_mapper = rom_mapper;
    Bus::ppu = ppu;
}

/*
    Current implementation isn't great, most common access ranges should be
    checked first for maximum performance.
*/
uint8_t Bus::read(uint16_t addr) {
    if (addr > 0xFFFF) {
        // This check happens every cycle, but probably never actually hits
        std::cout << "Invalid address: " << addr << std::endl;
    } else if (addr == IE_REGISTER) { /* IE Register */
        return imu->read_iereg();
    } else if (addr >= HRAM_START) { /* HRAM */
        return imu->read_hram(addr);
    } else if (addr >= IO_REGISTERS_START) { /* I/O Registers */
        return imu->read_ioreg(addr);
    } else if (addr >= UNUSABLE_START) { /* Unusable */
        std::cout << "Invalid Unusable Region address: " << addr << std::endl;
    } else if (addr >= OAM_START) { /* OAM */
        return ppu->oam_read(addr);
    } else if (addr >= ECHO_RAM_START) { /* Echo RAM */
        std::cout << "Invalid Echo RAM address: " << addr << std::endl;
    } else if (addr >= IWRAM_START) { /* IWRAM */
        return Bus::imu->read_iwram(addr);
    } else if (addr >= ERAM_START) { /* EWRAM */
        return Bus::rom_mapper->read_eram(addr);
    } else if (addr >= VRAM_START) { /* VRAM */
        return ppu->vram_read(addr);
    } else if (addr >= ROM_START) { /* ROM */
        return Bus::rom_mapper->read_rom(addr, 0x0);
    } else if (addr >= 0x0000) { /* TODO: BIOS */
        return 0;
    } else {
        std::cout << "Invalid address: " << addr << std::endl;
    }

    return 0;  // 0 is not satisfying to indicate error
}

void Bus::write(uint16_t addr, uint8_t data) {
    if (addr > 0xFFFF) {
        // This check happens every cycle, but probably never actually hits
        std::cout << "Invalid address" << std::endl;
    } else if (addr == IE_REGISTER) { /* IE Registers */
        Bus::imu->write_iereg(data);
    } else if (addr >= HRAM_START) { /* HRAM */
        Bus::imu->write_hram(addr, data);
    } else if (addr >= IO_REGISTERS_START) { /* I/O Registers */
        Bus::imu->write_ioreg(addr, data);
    } else if (addr >= UNUSABLE_START) { /* Unusable */
        std::cout << "Unusable Area Writes not allowed: " << addr << std::endl;
    } else if (addr >= OAM_START) { /* OAM */
        Bus::ppu->oam_write(addr, data);
    } else if (addr >= ECHO_RAM_START) { /* Echo RAM */
        std::cout << "ECHO RAM Writes not allowed: " << addr << std::endl;
    } else if (addr >= IWRAM_START) { /* IWRAM */
        Bus::imu->write_iwram(addr, data);
    } else if (addr >= ERAM_START) { /* EWRAM */
        Bus::rom_mapper->write_eram(addr, data);
    } else if (addr >= VRAM_START) { /* VRAM */
        Bus::ppu->vram_write(addr, data);
    } else if (addr >= ROM_START) { /* ROM */
        std::cout << "ROM Writes not allowed: " << addr << std::endl;
    } else if (addr >= 0x0000) { /* BIOS */
        std::cout << "BIOS Writes not allowed: " << addr << std::endl;
    } else {
        std::cout << "Invalid address: " << addr << std::endl;
    }
    return;
}

Bus::~Bus() {}