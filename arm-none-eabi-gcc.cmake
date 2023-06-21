message(STATUS "С боженькой. Погнали ")
if( NOT ARM_NONE_EABI_TOOLCHAIN_PATH )
    set(ARM_NONE_EABI_TOOLCHAIN_PATH "C:/Software/EmbeddedGCC/")
    message(STATUS "No ARM_NONE_EABI_TOOLCHAIN_PATH specified, using default: " ${ARM_NONE_EABI_TOOLCHAIN_PATH})
else()
    message(STATUS " ARM_NONE_EABI_TOOLCHAIN_PATH specified using: " ${ARM_NONE_EABI_TOOLCHAIN_PATH})
    file(TO_CMAKE_PATH "${ARM_NONE_EABI_TOOLCHAIN_PATH}" ARM_NONE_EABI_TOOLCHAIN_PATH)
endif()

if(NOT NORDIC_TARGET_TRIPLET)
    set(NORDIC_TARGET_TRIPLET "arm-none-eabi")
    message(STATUS "No NORDIC_TARGET_TRIPLET specified, using default: " ${NORDIC_TARGET_TRIPLET})
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_SYSROOT  "${ARM_NONE_EABI_TOOLCHAIN_PATH}/${NORDIC_TARGET_TRIPLET}")
set(TOOLCHAIN_BIN_PATH "${ARM_NONE_EABI_TOOLCHAIN_PATH}/bin")
set(TOOLCHAIN_INC_PATH "${ARM_NONE_EABI_TOOLCHAIN_PATH}/${NORDIC_TARGET_TRIPLET}/include")
set(TOOLCHAIN_LIB_PATH "${ARM_NONE_EABI_TOOLCHAIN_PATH}/${NORDIC_TARGET_TRIPLET}/lib")


find_program(CMAKE_OBJCOPY NAMES ${NORDIC_TARGET_TRIPLET}-objcopy PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJDUMP NAMES ${NORDIC_TARGET_TRIPLET}-objdump PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_SIZE NAMES ${NORDIC_TARGET_TRIPLET}-size PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_DEBUGGER NAMES ${NORDIC_TARGET_TRIPLET}-gdb PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CPPFILT NAMES ${NORDIC_TARGET_TRIPLET}-c++filt PATHS ${TOOLCHAIN_BIN_PATH})


set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
find_program(CMAKE_C_COMPILER NAMES ${NORDIC_TARGET_TRIPLET}-gcc PATHS ${TOOLCHAIN_BIN_PATH} )
find_program(CMAKE_CXX_COMPILER NAMES ${NORDIC_TARGET_TRIPLET}-c++ PATHS ${TOOLCHAIN_BIN_PATH} )
find_program(CMAKE_ASM_COMPILER NAMES ${NORDIC_TARGET_TRIPLET} PATHS ${TOOLCHAIN_BIN_PATH} )
find_program(CMAKE_SIZE_UTIL NAMES ${NORDIC_TARGET_TRIPLET}-size PATHS ${TOOLCHAIN_BIN_PATH} )
find_program(CMAKE_OBJCOPY NAMES ${NORDIC_TARGET_TRIPLET}-objcopy PATHS ${TOOLCHAIN_BIN_PATH} )


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CPUFLAGS "-mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")

set(CMAKE_CXX_FLAGS ${CPUFLAGS} CACHE INTERNAL "")
set(CMAKE_C_FLAGS ${CPUFLAGS} CACHE INTERNAL "")

