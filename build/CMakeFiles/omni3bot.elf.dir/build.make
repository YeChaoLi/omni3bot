# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chao/Workspace/Hackathon/omni3bot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chao/Workspace/Hackathon/omni3bot/build

# Include any dependencies generated for this target.
include CMakeFiles/omni3bot.elf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/omni3bot.elf.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/omni3bot.elf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/omni3bot.elf.dir/flags.make

project_elf_src_esp32s3.c:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating project_elf_src_esp32s3.c"
	/usr/bin/cmake -E touch /home/chao/Workspace/Hackathon/omni3bot/build/project_elf_src_esp32s3.c

CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj: CMakeFiles/omni3bot.elf.dir/flags.make
CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj: project_elf_src_esp32s3.c
CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj: CMakeFiles/omni3bot.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj"
	/home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj -MF CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj.d -o CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj -c /home/chao/Workspace/Hackathon/omni3bot/build/project_elf_src_esp32s3.c

CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.i"
	/home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/chao/Workspace/Hackathon/omni3bot/build/project_elf_src_esp32s3.c > CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.i

CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.s"
	/home/chao/.espressif/tools/xtensa-esp-elf/esp-15.1.0_20250607/xtensa-esp-elf/bin/xtensa-esp32s3-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/chao/Workspace/Hackathon/omni3bot/build/project_elf_src_esp32s3.c -o CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.s

# Object files for target omni3bot.elf
omni3bot_elf_OBJECTS = \
"CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj"

# External object files for target omni3bot.elf
omni3bot_elf_EXTERNAL_OBJECTS =

omni3bot.elf: CMakeFiles/omni3bot.elf.dir/project_elf_src_esp32s3.c.obj
omni3bot.elf: CMakeFiles/omni3bot.elf.dir/build.make
omni3bot.elf: esp-idf/xtensa/libxtensa.a
omni3bot.elf: esp-idf/esp_timer/libesp_timer.a
omni3bot.elf: esp-idf/esp_mm/libesp_mm.a
omni3bot.elf: esp-idf/esp_driver_gpio/libesp_driver_gpio.a
omni3bot.elf: esp-idf/esp_pm/libesp_pm.a
omni3bot.elf: esp-idf/mbedtls/libmbedtls.a
omni3bot.elf: esp-idf/esp_app_format/libesp_app_format.a
omni3bot.elf: esp-idf/esp_bootloader_format/libesp_bootloader_format.a
omni3bot.elf: esp-idf/app_update/libapp_update.a
omni3bot.elf: esp-idf/esp_partition/libesp_partition.a
omni3bot.elf: esp-idf/efuse/libefuse.a
omni3bot.elf: esp-idf/bootloader_support/libbootloader_support.a
omni3bot.elf: esp-idf/esp_system/libesp_system.a
omni3bot.elf: esp-idf/esp_common/libesp_common.a
omni3bot.elf: esp-idf/esp_rom/libesp_rom.a
omni3bot.elf: esp-idf/log/liblog.a
omni3bot.elf: esp-idf/heap/libheap.a
omni3bot.elf: esp-idf/esp_security/libesp_security.a
omni3bot.elf: esp-idf/esp_hw_support/libesp_hw_support.a
omni3bot.elf: esp-idf/freertos/libfreertos.a
omni3bot.elf: esp-idf/newlib/libnewlib.a
omni3bot.elf: esp-idf/pthread/libpthread.a
omni3bot.elf: esp-idf/cxx/libcxx.a
omni3bot.elf: esp-idf/soc/libsoc.a
omni3bot.elf: esp-idf/hal/libhal.a
omni3bot.elf: esp-idf/spi_flash/libspi_flash.a
omni3bot.elf: esp-idf/esp_event/libesp_event.a
omni3bot.elf: esp-idf/nvs_flash/libnvs_flash.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
omni3bot.elf: esp-idf/esp_driver_uart/libesp_driver_uart.a
omni3bot.elf: esp-idf/esp_driver_usb_serial_jtag/libesp_driver_usb_serial_jtag.a
omni3bot.elf: esp-idf/esp_vfs_console/libesp_vfs_console.a
omni3bot.elf: esp-idf/vfs/libvfs.a
omni3bot.elf: esp-idf/lwip/liblwip.a
omni3bot.elf: esp-idf/esp_netif/libesp_netif.a
omni3bot.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
omni3bot.elf: esp-idf/esp_coex/libesp_coex.a
omni3bot.elf: esp-idf/esp_wifi/libesp_wifi.a
omni3bot.elf: esp-idf/esp_driver_spi/libesp_driver_spi.a
omni3bot.elf: esp-idf/esp_gdbstub/libesp_gdbstub.a
omni3bot.elf: esp-idf/bt/libbt.a
omni3bot.elf: esp-idf/esp_hid/libesp_hid.a
omni3bot.elf: esp-idf/esp_driver_ledc/libesp_driver_ledc.a
omni3bot.elf: esp-idf/esp_driver_pcnt/libesp_driver_pcnt.a
omni3bot.elf: esp-idf/esp_driver_gptimer/libesp_driver_gptimer.a
omni3bot.elf: esp-idf/esp_driver_mcpwm/libesp_driver_mcpwm.a
omni3bot.elf: esp-idf/esp_driver_i2s/libesp_driver_i2s.a
omni3bot.elf: esp-idf/sdmmc/libsdmmc.a
omni3bot.elf: esp-idf/esp_driver_sd_intf/libesp_driver_sd_intf.a
omni3bot.elf: esp-idf/esp_driver_sdmmc/libesp_driver_sdmmc.a
omni3bot.elf: esp-idf/esp_driver_sdspi/libesp_driver_sdspi.a
omni3bot.elf: esp-idf/esp_driver_rmt/libesp_driver_rmt.a
omni3bot.elf: esp-idf/esp_driver_tsens/libesp_driver_tsens.a
omni3bot.elf: esp-idf/esp_driver_sdm/libesp_driver_sdm.a
omni3bot.elf: esp-idf/esp_driver_i2c/libesp_driver_i2c.a
omni3bot.elf: esp-idf/esp_driver_twai/libesp_driver_twai.a
omni3bot.elf: esp-idf/driver/libdriver.a
omni3bot.elf: esp-idf/esp-idf-lib__i2cdev/libesp-idf-lib__i2cdev.a
omni3bot.elf: esp-idf/esp-idf-lib__qmi8658c/libesp-idf-lib__qmi8658c.a
omni3bot.elf: esp-idf/espressif__led_strip/libespressif__led_strip.a
omni3bot.elf: esp-idf/main/libmain.a
omni3bot.elf: esp-idf/esp_hid/libesp_hid.a
omni3bot.elf: esp-idf/bt/libbt.a
omni3bot.elf: esp-idf/esp-idf-lib__qmi8658c/libesp-idf-lib__qmi8658c.a
omni3bot.elf: esp-idf/esp-idf-lib__i2cdev/libesp-idf-lib__i2cdev.a
omni3bot.elf: esp-idf/driver/libdriver.a
omni3bot.elf: esp-idf/esp_driver_ledc/libesp_driver_ledc.a
omni3bot.elf: esp-idf/esp_driver_pcnt/libesp_driver_pcnt.a
omni3bot.elf: esp-idf/esp_driver_gptimer/libesp_driver_gptimer.a
omni3bot.elf: esp-idf/esp_driver_mcpwm/libesp_driver_mcpwm.a
omni3bot.elf: esp-idf/esp_driver_i2s/libesp_driver_i2s.a
omni3bot.elf: esp-idf/esp_driver_sdmmc/libesp_driver_sdmmc.a
omni3bot.elf: esp-idf/esp_driver_sd_intf/libesp_driver_sd_intf.a
omni3bot.elf: esp-idf/esp_driver_sdspi/libesp_driver_sdspi.a
omni3bot.elf: esp-idf/sdmmc/libsdmmc.a
omni3bot.elf: esp-idf/esp_driver_tsens/libesp_driver_tsens.a
omni3bot.elf: esp-idf/esp_driver_sdm/libesp_driver_sdm.a
omni3bot.elf: esp-idf/esp_driver_i2c/libesp_driver_i2c.a
omni3bot.elf: esp-idf/esp_driver_twai/libesp_driver_twai.a
omni3bot.elf: esp-idf/espressif__led_strip/libespressif__led_strip.a
omni3bot.elf: esp-idf/esp_driver_spi/libesp_driver_spi.a
omni3bot.elf: esp-idf/esp_driver_rmt/libesp_driver_rmt.a
omni3bot.elf: esp-idf/xtensa/libxtensa.a
omni3bot.elf: esp-idf/esp_timer/libesp_timer.a
omni3bot.elf: esp-idf/esp_mm/libesp_mm.a
omni3bot.elf: esp-idf/esp_driver_gpio/libesp_driver_gpio.a
omni3bot.elf: esp-idf/esp_pm/libesp_pm.a
omni3bot.elf: esp-idf/mbedtls/libmbedtls.a
omni3bot.elf: esp-idf/esp_app_format/libesp_app_format.a
omni3bot.elf: esp-idf/esp_bootloader_format/libesp_bootloader_format.a
omni3bot.elf: esp-idf/app_update/libapp_update.a
omni3bot.elf: esp-idf/esp_partition/libesp_partition.a
omni3bot.elf: esp-idf/efuse/libefuse.a
omni3bot.elf: esp-idf/bootloader_support/libbootloader_support.a
omni3bot.elf: esp-idf/esp_system/libesp_system.a
omni3bot.elf: esp-idf/esp_common/libesp_common.a
omni3bot.elf: esp-idf/esp_rom/libesp_rom.a
omni3bot.elf: esp-idf/log/liblog.a
omni3bot.elf: esp-idf/heap/libheap.a
omni3bot.elf: esp-idf/esp_security/libesp_security.a
omni3bot.elf: esp-idf/esp_hw_support/libesp_hw_support.a
omni3bot.elf: esp-idf/freertos/libfreertos.a
omni3bot.elf: esp-idf/newlib/libnewlib.a
omni3bot.elf: esp-idf/pthread/libpthread.a
omni3bot.elf: esp-idf/cxx/libcxx.a
omni3bot.elf: esp-idf/soc/libsoc.a
omni3bot.elf: esp-idf/hal/libhal.a
omni3bot.elf: esp-idf/spi_flash/libspi_flash.a
omni3bot.elf: esp-idf/esp_event/libesp_event.a
omni3bot.elf: esp-idf/nvs_flash/libnvs_flash.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
omni3bot.elf: esp-idf/esp_driver_uart/libesp_driver_uart.a
omni3bot.elf: esp-idf/esp_driver_usb_serial_jtag/libesp_driver_usb_serial_jtag.a
omni3bot.elf: esp-idf/esp_vfs_console/libesp_vfs_console.a
omni3bot.elf: esp-idf/vfs/libvfs.a
omni3bot.elf: esp-idf/lwip/liblwip.a
omni3bot.elf: esp-idf/esp_netif/libesp_netif.a
omni3bot.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
omni3bot.elf: esp-idf/esp_coex/libesp_coex.a
omni3bot.elf: esp-idf/esp_wifi/libesp_wifi.a
omni3bot.elf: esp-idf/esp_gdbstub/libesp_gdbstub.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/p256-m/libp256m.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_coex/lib/esp32s3/libcoexist.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libcore.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libespnow.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libmesh.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libnet80211.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libpp.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libsmartconfig.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libwapi.a
omni3bot.elf: esp-idf/xtensa/libxtensa.a
omni3bot.elf: esp-idf/esp_timer/libesp_timer.a
omni3bot.elf: esp-idf/esp_mm/libesp_mm.a
omni3bot.elf: esp-idf/esp_driver_gpio/libesp_driver_gpio.a
omni3bot.elf: esp-idf/esp_pm/libesp_pm.a
omni3bot.elf: esp-idf/mbedtls/libmbedtls.a
omni3bot.elf: esp-idf/esp_app_format/libesp_app_format.a
omni3bot.elf: esp-idf/esp_bootloader_format/libesp_bootloader_format.a
omni3bot.elf: esp-idf/app_update/libapp_update.a
omni3bot.elf: esp-idf/esp_partition/libesp_partition.a
omni3bot.elf: esp-idf/efuse/libefuse.a
omni3bot.elf: esp-idf/bootloader_support/libbootloader_support.a
omni3bot.elf: esp-idf/esp_system/libesp_system.a
omni3bot.elf: esp-idf/esp_common/libesp_common.a
omni3bot.elf: esp-idf/esp_rom/libesp_rom.a
omni3bot.elf: esp-idf/log/liblog.a
omni3bot.elf: esp-idf/heap/libheap.a
omni3bot.elf: esp-idf/esp_security/libesp_security.a
omni3bot.elf: esp-idf/esp_hw_support/libesp_hw_support.a
omni3bot.elf: esp-idf/freertos/libfreertos.a
omni3bot.elf: esp-idf/newlib/libnewlib.a
omni3bot.elf: esp-idf/pthread/libpthread.a
omni3bot.elf: esp-idf/cxx/libcxx.a
omni3bot.elf: esp-idf/soc/libsoc.a
omni3bot.elf: esp-idf/hal/libhal.a
omni3bot.elf: esp-idf/spi_flash/libspi_flash.a
omni3bot.elf: esp-idf/esp_event/libesp_event.a
omni3bot.elf: esp-idf/nvs_flash/libnvs_flash.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
omni3bot.elf: esp-idf/esp_driver_uart/libesp_driver_uart.a
omni3bot.elf: esp-idf/esp_driver_usb_serial_jtag/libesp_driver_usb_serial_jtag.a
omni3bot.elf: esp-idf/esp_vfs_console/libesp_vfs_console.a
omni3bot.elf: esp-idf/vfs/libvfs.a
omni3bot.elf: esp-idf/lwip/liblwip.a
omni3bot.elf: esp-idf/esp_netif/libesp_netif.a
omni3bot.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
omni3bot.elf: esp-idf/esp_coex/libesp_coex.a
omni3bot.elf: esp-idf/esp_wifi/libesp_wifi.a
omni3bot.elf: esp-idf/esp_gdbstub/libesp_gdbstub.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/p256-m/libp256m.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_coex/lib/esp32s3/libcoexist.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libcore.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libespnow.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libmesh.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libnet80211.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libpp.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libsmartconfig.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libwapi.a
omni3bot.elf: esp-idf/xtensa/libxtensa.a
omni3bot.elf: esp-idf/esp_timer/libesp_timer.a
omni3bot.elf: esp-idf/esp_mm/libesp_mm.a
omni3bot.elf: esp-idf/esp_driver_gpio/libesp_driver_gpio.a
omni3bot.elf: esp-idf/esp_pm/libesp_pm.a
omni3bot.elf: esp-idf/mbedtls/libmbedtls.a
omni3bot.elf: esp-idf/esp_app_format/libesp_app_format.a
omni3bot.elf: esp-idf/esp_bootloader_format/libesp_bootloader_format.a
omni3bot.elf: esp-idf/app_update/libapp_update.a
omni3bot.elf: esp-idf/esp_partition/libesp_partition.a
omni3bot.elf: esp-idf/efuse/libefuse.a
omni3bot.elf: esp-idf/bootloader_support/libbootloader_support.a
omni3bot.elf: esp-idf/esp_system/libesp_system.a
omni3bot.elf: esp-idf/esp_common/libesp_common.a
omni3bot.elf: esp-idf/esp_rom/libesp_rom.a
omni3bot.elf: esp-idf/log/liblog.a
omni3bot.elf: esp-idf/heap/libheap.a
omni3bot.elf: esp-idf/esp_security/libesp_security.a
omni3bot.elf: esp-idf/esp_hw_support/libesp_hw_support.a
omni3bot.elf: esp-idf/freertos/libfreertos.a
omni3bot.elf: esp-idf/newlib/libnewlib.a
omni3bot.elf: esp-idf/pthread/libpthread.a
omni3bot.elf: esp-idf/cxx/libcxx.a
omni3bot.elf: esp-idf/soc/libsoc.a
omni3bot.elf: esp-idf/hal/libhal.a
omni3bot.elf: esp-idf/spi_flash/libspi_flash.a
omni3bot.elf: esp-idf/esp_event/libesp_event.a
omni3bot.elf: esp-idf/nvs_flash/libnvs_flash.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
omni3bot.elf: esp-idf/esp_driver_uart/libesp_driver_uart.a
omni3bot.elf: esp-idf/esp_driver_usb_serial_jtag/libesp_driver_usb_serial_jtag.a
omni3bot.elf: esp-idf/esp_vfs_console/libesp_vfs_console.a
omni3bot.elf: esp-idf/vfs/libvfs.a
omni3bot.elf: esp-idf/lwip/liblwip.a
omni3bot.elf: esp-idf/esp_netif/libesp_netif.a
omni3bot.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
omni3bot.elf: esp-idf/esp_coex/libesp_coex.a
omni3bot.elf: esp-idf/esp_wifi/libesp_wifi.a
omni3bot.elf: esp-idf/esp_gdbstub/libesp_gdbstub.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/p256-m/libp256m.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_coex/lib/esp32s3/libcoexist.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libcore.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libespnow.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libmesh.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libnet80211.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libpp.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libsmartconfig.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libwapi.a
omni3bot.elf: esp-idf/xtensa/libxtensa.a
omni3bot.elf: esp-idf/esp_timer/libesp_timer.a
omni3bot.elf: esp-idf/esp_mm/libesp_mm.a
omni3bot.elf: esp-idf/esp_driver_gpio/libesp_driver_gpio.a
omni3bot.elf: esp-idf/esp_pm/libesp_pm.a
omni3bot.elf: esp-idf/mbedtls/libmbedtls.a
omni3bot.elf: esp-idf/esp_app_format/libesp_app_format.a
omni3bot.elf: esp-idf/esp_bootloader_format/libesp_bootloader_format.a
omni3bot.elf: esp-idf/app_update/libapp_update.a
omni3bot.elf: esp-idf/esp_partition/libesp_partition.a
omni3bot.elf: esp-idf/efuse/libefuse.a
omni3bot.elf: esp-idf/bootloader_support/libbootloader_support.a
omni3bot.elf: esp-idf/esp_system/libesp_system.a
omni3bot.elf: esp-idf/esp_common/libesp_common.a
omni3bot.elf: esp-idf/esp_rom/libesp_rom.a
omni3bot.elf: esp-idf/log/liblog.a
omni3bot.elf: esp-idf/heap/libheap.a
omni3bot.elf: esp-idf/esp_security/libesp_security.a
omni3bot.elf: esp-idf/esp_hw_support/libesp_hw_support.a
omni3bot.elf: esp-idf/freertos/libfreertos.a
omni3bot.elf: esp-idf/newlib/libnewlib.a
omni3bot.elf: esp-idf/pthread/libpthread.a
omni3bot.elf: esp-idf/cxx/libcxx.a
omni3bot.elf: esp-idf/soc/libsoc.a
omni3bot.elf: esp-idf/hal/libhal.a
omni3bot.elf: esp-idf/spi_flash/libspi_flash.a
omni3bot.elf: esp-idf/esp_event/libesp_event.a
omni3bot.elf: esp-idf/nvs_flash/libnvs_flash.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_ringbuf/libesp_ringbuf.a
omni3bot.elf: esp-idf/esp_driver_uart/libesp_driver_uart.a
omni3bot.elf: esp-idf/esp_driver_usb_serial_jtag/libesp_driver_usb_serial_jtag.a
omni3bot.elf: esp-idf/esp_vfs_console/libesp_vfs_console.a
omni3bot.elf: esp-idf/vfs/libvfs.a
omni3bot.elf: esp-idf/lwip/liblwip.a
omni3bot.elf: esp-idf/esp_netif/libesp_netif.a
omni3bot.elf: esp-idf/wpa_supplicant/libwpa_supplicant.a
omni3bot.elf: esp-idf/esp_coex/libesp_coex.a
omni3bot.elf: esp-idf/esp_wifi/libesp_wifi.a
omni3bot.elf: esp-idf/esp_gdbstub/libesp_gdbstub.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedtls.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedcrypto.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/library/libmbedx509.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/everest/libeverest.a
omni3bot.elf: esp-idf/mbedtls/mbedtls/3rdparty/p256-m/libp256m.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_coex/lib/esp32s3/libcoexist.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libcore.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libespnow.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libmesh.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libnet80211.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libpp.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libsmartconfig.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_wifi/lib/esp32s3/libwapi.a
omni3bot.elf: /home/chao/esp/master/esp-idf/components/xtensa/esp32s3/libxt_hal.a
omni3bot.elf: esp-idf/pthread/libpthread.a
omni3bot.elf: esp-idf/newlib/libnewlib.a
omni3bot.elf: esp-idf/cxx/libcxx.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_phy/libesp_phy.a
omni3bot.elf: esp-idf/esp_system/ld/memory.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.api.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.bt_funcs.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.libgcc.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.wdt.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.version.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.ble_cca.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.ble_test.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.libc.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/esp_rom/esp32s3/ld/esp32s3.rom.newlib.ld
omni3bot.elf: /home/chao/esp/master/esp-idf/components/soc/esp32s3/ld/esp32s3.peripherals.ld
omni3bot.elf: esp-idf/esp_system/ld/sections.ld
omni3bot.elf: CMakeFiles/omni3bot.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable omni3bot.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/omni3bot.elf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/omni3bot.elf.dir/build: omni3bot.elf
.PHONY : CMakeFiles/omni3bot.elf.dir/build

CMakeFiles/omni3bot.elf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/omni3bot.elf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/omni3bot.elf.dir/clean

CMakeFiles/omni3bot.elf.dir/depend: project_elf_src_esp32s3.c
	cd /home/chao/Workspace/Hackathon/omni3bot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chao/Workspace/Hackathon/omni3bot /home/chao/Workspace/Hackathon/omni3bot /home/chao/Workspace/Hackathon/omni3bot/build /home/chao/Workspace/Hackathon/omni3bot/build /home/chao/Workspace/Hackathon/omni3bot/build/CMakeFiles/omni3bot.elf.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/omni3bot.elf.dir/depend

