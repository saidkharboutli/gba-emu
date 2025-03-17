#include "imu.hpp"

/*
 * The Internal Memory Unit (IMU) handles all access of internal memory, besides
 * CPU-internal registers.
 */

InternalMemoryUnit::InternalMemoryUnit() {
    iwram_bank0 = (uint8_t*)malloc(0x1000);
    iwram_bank1 = (uint8_t*)malloc(0x1000);
    io_reg = (uint8_t*)malloc(0x0080);
    hram = (uint8_t*)malloc(0x007F);

    ie_reg = 0;
}

/* IWRAM */
uint8_t InternalMemoryUnit::read_iwram(uint16_t addr) {
    if (addr - IWRAM_START < 0x1000) {
        return iwram_bank0[addr - IWRAM_START];
    }
    return iwram_bank1[addr - IWRAM_START - 0x1000];
}
void InternalMemoryUnit::write_iwram(uint16_t addr, uint8_t data) {
    if (addr - IWRAM_START < 0x1000) {
        iwram_bank0[addr - IWRAM_START] = data;
    }
    iwram_bank1[addr - IWRAM_START - 0x1000] = data;
}

/* IE REGISTER */
uint8_t InternalMemoryUnit::read_iereg() { return ie_reg; }
void InternalMemoryUnit::write_iereg(uint8_t data) { ie_reg = data; }

/* IO REGISTERS */
uint8_t InternalMemoryUnit::read_ioreg(uint16_t addr) {
    return io_reg[addr - IO_REGISTERS_START];
}
void InternalMemoryUnit::write_ioreg(uint16_t addr, uint8_t data) {
    io_reg[addr - IO_REGISTERS_START] = data;
}

/* HRAM */
uint8_t InternalMemoryUnit::read_hram(uint16_t addr) {
    return hram[addr - HRAM_START];
}
void InternalMemoryUnit::write_hram(uint16_t addr, uint8_t data) {
    hram[addr - HRAM_START] = data;
}

InternalMemoryUnit::~InternalMemoryUnit() {
    delete iwram_bank0;
    delete iwram_bank1;
    delete io_reg;
    delete hram;
}
