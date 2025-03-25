#include "gba.hpp"

#include <chrono>

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
    int i = 0;
    while (i < TEST_CYCLE_COUNT) {
        i++;
        cpu_sm83->execute(cpu_sm83->fetch());
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

int main() {
    using namespace std;
    using namespace chrono;

    GBA* gba = new GBA();
    auto start = high_resolution_clock ::now();
    gba->run();
    auto stop = high_resolution_clock ::now();

    auto duration_us = duration_cast<microseconds>(stop - start);
    cout << "Execution Time: " << duration_us.count() << "us" << endl;
    cout << "Max Clock: " << (float)TEST_CYCLE_COUNT / (duration_us.count())
         << "MHz" << endl;

    delete gba;
}
