#ifndef ROM_MAPPER_HPP
#define ROM_MAPPER_HPP

#include <cstdint>
#include <cstdlib>

#include "bus/bus.hpp"

class ROMMapper {
   private:
    uint8_t* rom_bank0;
    uint8_t* rom_bankN;

    uint8_t* eram;

   public:
    ROMMapper();

    uint8_t read_rom(uint16_t addr, uint8_t bank);

    uint8_t read_eram(uint16_t addr);
    void write_eram(uint16_t addr, uint8_t data);

    ~ROMMapper();
};

#endif