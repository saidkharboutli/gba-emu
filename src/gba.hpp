#ifndef GBA_HPP
#define GBA_HPP

#include "bus/bus.hpp"
#include "cpu/cpu_sm83.hpp"
#include "memory/imu.hpp"

class GBA {
   private:
    /* Bus to Connect Components */
    Bus *bus;

    /* Components */
    InternalMemoryUnit *imu;
    SM83 *cpu_sm83;

   public:
    GBA();
    void run();
    ~GBA();
};

#endif