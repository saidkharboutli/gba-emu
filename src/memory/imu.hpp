#ifndef INTERNAL_MEMORY_UNIT_HPP
#define INTERNAL_MEMORY_UNIT_HPP

#include <cstdint>
#include <cstdlib>

#include "bus/bus.hpp"

class InternalMemoryUnit {
   private:
    uint8_t* iwram_bank0;
    uint8_t* iwram_bank1;
    uint8_t* hram;
    uint8_t* io_reg;

    uint8_t ie_reg;

   public:
    InternalMemoryUnit();

    uint8_t read_iwram(uint16_t addr);
    void write_iwram(uint16_t addr, uint8_t data);

    uint8_t read_iereg();
    void write_iereg(uint8_t data);

    uint8_t read_ioreg(uint16_t addr);
    void write_ioreg(uint16_t addr, uint8_t data);

    uint8_t read_hram(uint16_t addr);
    void write_hram(uint16_t addr, uint8_t data);

    ~InternalMemoryUnit();
};

#endif