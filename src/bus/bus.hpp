#ifndef BUS_HPP
#define BUS_HPP

enum MemoryMap {
    ROM_START = 0x0100,
    VRAM_START = 0x8000,
    ERAM_START = 0xA000,
    IWRAM_START = 0xC000,
    ECHO_RAM_START = 0xE000,
    OAM_START = 0xFE00,
    UNUSABLE_START = 0xFEA0,
    IO_REGISTERS_START = 0xFF00,
    HRAM_START = 0xFF80,
    IE_REGISTER = 0xFFFF
};

#include <cstdint>
#include <iostream>

/* BUS COMPONENTS */
#include "memory/imu.hpp"
#include "ppu/ppu.hpp"
#include "rom/mapper.hpp"

class Bus {
   private:
    InternalMemoryUnit* imu;
    ROMMapper* rom_mapper;
    PPU* ppu;

   public:
    Bus(InternalMemoryUnit* imu, ROMMapper* rom_mapper, PPU* ppu);
    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t data);
    ~Bus();
};

#endif