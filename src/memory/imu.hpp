#ifndef INTERNAL_MEMORY_UNIT_HPP
#define INTERNAL_MEMORY_UNIT_HPP

#include <cstdint>
#include <cstdlib>

#include "bus/bus.hpp"

class InternalMemoryUnit {
   private:
    uint8_t* iwram_b0;
    uint8_t* iwram_b1;

   public:
    InternalMemoryUnit();
    uint8_t read(uint8_t addr);
    void write(uint8_t addr);
    ~InternalMemoryUnit();
};

#endif