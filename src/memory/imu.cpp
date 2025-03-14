#include "imu.hpp"

InternalMemoryUnit::InternalMemoryUnit() {
    iwram_b0 = (uint8_t*)malloc(0x1000);
    iwram_b1 = (uint8_t*)malloc(0x1000);
}

uint8_t InternalMemoryUnit::read(uint8_t addr) {
    if (addr < 0x1000) {
        return iwram_b0[addr];
    }

    return iwram_b1[addr - 0x1000];
}

void InternalMemoryUnit::write(uint8_t addr) {
    if (addr < 0x1000) {
        iwram_b0[addr];
    }

    iwram_b1[addr - 0x1000];
}

InternalMemoryUnit::~InternalMemoryUnit() {
    delete iwram_b0;
    delete iwram_b1;
}
