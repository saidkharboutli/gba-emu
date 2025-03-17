#include "ppu.hpp"

PPU::PPU() {
    PPU::oam = (uint8_t*)malloc(0x00A0);
    PPU::vram = (uint8_t*)malloc(0x2000);
}

/*
 * VRAM
 */
uint8_t PPU::vram_read(uint16_t addr) { return vram[addr - VRAM_START]; }

void PPU::vram_write(uint16_t addr, uint8_t data) {
    vram[addr - VRAM_START] = data;
}

/*
 * OAM
 */
uint8_t PPU::oam_read(uint16_t addr) { return oam[addr - OAM_START]; }

void PPU::oam_write(uint16_t addr, uint8_t data) {
    oam[addr - OAM_START] = data;
}

PPU::~PPU() {
    delete oam;
    delete vram;
}