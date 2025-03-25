#ifndef PPU_HPP
#define PPU_HPP

#include <cstdint>
#include <cstdlib>

#include "../constants.hpp"

class PPU {
   private:
    uint8_t* oam;
    uint8_t* vram;

   public:
    PPU();

    uint8_t vram_read(uint16_t addr);
    void vram_write(uint16_t addr, uint8_t data);

    uint8_t oam_read(uint16_t addr);
    void oam_write(uint16_t addr, uint8_t data);

    ~PPU();
};

#endif