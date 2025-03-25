#ifndef BUS_HPP
#define BUS_HPP

#include <cstdint>
#include <iostream>

/* BUS COMPONENTS */
#include "../constants.hpp"
#include "../memory/imu.hpp"
#include "../ppu/ppu.hpp"
#include "../rom/mapper.hpp"

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