
cmake_minimum_required(VERSION 3.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_C_COMPILER sdcc-sdcc)
set(CMAKE_C_FLAGS "-mmcs51")

set(CMAKE_ASM-SDASM_COMPILER sdcc-sdas8051)

set(CMAKE_EXE_LINKER_FLAGS "-mmcs51")

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_FLAGS_INIT "")
set(CMAKE_EXE_LINKER_FLAGS_INIT "")

project(fx2fifo C ASM-SDASM)
set(FX2FW
    fx2fw/delay.c
    fx2fw/discon.c
    fx2fw/EZRegs.c
    fx2fw/fw.c
    fx2fw/get_strd.c
    fx2fw/i2c.c
    fx2fw/i2c_rw.c
    fx2fw/resume.c
    fx2fw/delayms.asm
    fx2fw/susp.asm
    fx2fw/USBJmpTb.asm
)
add_executable(fx2fifo dscr.asm slave.c ${FX2FW})
target_include_directories(fx2fifo PRIVATE ${CMAKE_SOURCE_DIR}/fx2fw)
