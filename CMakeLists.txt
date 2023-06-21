cmake_minimum_required(VERSION 3.15.3)

project(ARTERY VERSION 0.0.1 LANGUAGES CXX C ASM)

enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)


# Add preprocessor definitions to the compilation of source files
add_compile_definitions(AT32F403AVGT7)
add_compile_definitions(__START=main)
#add_compile_definitions(__STARTUP_CLEAR_BSS)


add_compile_options(
	-O0
	-ggdb3					# info for gdb
	-Wall
	-Wextra
	-Wshadow
	#-Wsign-conversion
	-ffunction-sections
	-fdata-sections
	-Wl,--gc-sections 		# delete not use code
	$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)


add_link_options(
	-T${CMAKE_SOURCE_DIR}/libraries/cmsis/cm4/device_support/startup/gcc/linker/AT32F403AxG_FLASH.ld
	-lc
	-lm
	-lnosys
	-lstdc++
	--specs=nano.specs
	--specs=nosys.specs
	-Wl,--gc-sections
	-Wl,--print-memory-usage
	-Wl,--undefined=uxTopUsedPriority
	-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)


#Add directory manufacturer's librares
add_subdirectory(libraries)

#Add includes directory
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/MAIN/inc/
	${CMAKE_CURRENT_SOURCE_DIR}/middlewares/
	${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_drivers/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_class/cdc
	${CMAKE_CURRENT_SOURCE_DIR}/Platform
)

#Add src files
set(SRC
	${CMAKE_CURRENT_SOURCE_DIR}/MAIN/src/app.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_drivers/src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_drivers/src/usbd_int.c
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_drivers/src/usbd_sdr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_class/cdc/cdc_class.c
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/usbd_class/cdc/cdc_desc.c
	)



add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} stdlibraries)


add_custom_target(hex
	ALL
	COMMAND ${CMAKE_OBJCOPY} ${PROJECT_NAME} ${PROJECT_NAME}.elf
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
	COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME} ${PROJECT_NAME}.bin
	DEPENDS ${PROJECT_NAME}
	COMMENT "Generating Hex file ${PROJECT_NAME}.hex"
)
