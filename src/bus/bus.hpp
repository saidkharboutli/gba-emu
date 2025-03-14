#ifndef BUS_HPP
#define BUS_HPP

enum MemoryMap {
    ROM_START = 0x4000,
    VRAM_START = 0x8000,
    EWRAM_START = 0xA000,
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

#include "memory/imu.hpp"

class Bus {
   private:
    InternalMemoryUnit* imu;

   public:
    Bus(InternalMemoryUnit* imu);

    uint8_t read(uint8_t addr);

    void write(uint8_t addr);

    ~Bus();
};

#endif