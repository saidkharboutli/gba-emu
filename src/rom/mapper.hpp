#ifndef ROM_MAPPER_HPP
#define ROM_MAPPER_HPP

#include <cstdint>
#include <cstdlib>

#include "bus/bus.hpp"

class ROMMapper {
   private:
    uint8_t* rom_bank0;
    uint8_t* rom_bankN;

   public:
    ROMMapper();
    uint8_t read(uint16_t addr, uint8_t bank);
    ~ROMMapper();
};

#endif