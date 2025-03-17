#include "gba.hpp"

GBA::GBA() {
    /* BUS COMPONENTS */
    GBA::imu = new InternalMemoryUnit();
    GBA::rom_mapper = new ROMMapper();
    GBA::ppu = new PPU();

    /* BUS */
    GBA::bus = new Bus(imu, rom_mapper, ppu);

    /* CPU */
    GBA::cpu_sm83 = new SM83(bus);
}

void GBA::run() {
    while (true) {
        cpu_sm83->execute(cpu_sm83->decode(cpu_sm83->fetch()));
    }
}

GBA::~GBA() {
    // may not even need delete, not malloc
    delete imu;
    delete rom_mapper;
    delete ppu;
    delete bus;
    delete cpu_sm83;
}
