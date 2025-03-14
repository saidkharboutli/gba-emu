#include "gba.hpp"

GBA::GBA() {
    /* BUS COMPONENTS */
    GBA::imu = new InternalMemoryUnit();

    /* BUS */
    GBA::bus = new Bus(imu);

    /* CPU */
    GBA::cpu_sm83 = new SM83(bus);
}

void GBA::run() {
    while (true) {
        cpu_sm83->execute(cpu_sm83->decode(cpu_sm83->fetch()));
    }
}

GBA::~GBA() {
    delete bus;
    delete imu;
    delete cpu_sm83;
}
