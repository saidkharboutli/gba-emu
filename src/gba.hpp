#ifndef GBA_HPP
#define GBA_HPP

#include "bus/bus.hpp"
#include "cpu/cpu_sm83.hpp"

class GBA {
   private:
    /* Bus to Connect Components */
    Bus *bus;

    /* Bus Components */
    InternalMemoryUnit *imu;
    ROMMapper *rom_mapper;
    PPU *ppu;
    SM83 *cpu_sm83;

   public:
    GBA();
    void run();
    ~GBA();
};

#endif