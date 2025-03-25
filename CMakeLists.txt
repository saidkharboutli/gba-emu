cmake_minimum_required(VERSION 3.30)
project(gba_emu)

set(CMAKE_CXX_STANDARD 20)

add_executable(gba_emu
        src/constants.hpp
        src/memory/imu.cpp src/memory/imu.hpp
        src/ppu/ppu.cpp src/ppu/ppu.hpp
        src/rom/mapper.cpp src/rom/mapper.hpp
        src/bus/bus.cpp src/bus/bus.hpp
        src/cpu/cpu_sm83.cpp src/cpu/cpu_sm83.hpp
        src/gba.cpp src/gba.hpp
)
