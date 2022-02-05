# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "ASM"
  "C"
  "CXX"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_ASM
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_divider/divider.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_divider/divider.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/irq_handler_chain.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_bit_ops/bit_ops_aeabi.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_divider/divider.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_divider/divider.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_aeabi.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_aeabi.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_v1_rom_shim.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_aeabi.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_aeabi.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_v1_rom_shim.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_int64_ops/pico_int64_ops_aeabi.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_mem_ops/mem_ops_aeabi.S.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_standard_link/crt0.S" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_standard_link/crt0.S.obj"
  )
set(CMAKE_ASM_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_ASM
  "CFG_TUSB_DEBUG=1"
  "CFG_TUSB_MCU=OPT_MCU_RP2040"
  "CFG_TUSB_OS=OPT_OS_PICO"
  "LIB_PICO_BIT_OPS=1"
  "LIB_PICO_BIT_OPS_PICO=1"
  "LIB_PICO_DIVIDER=1"
  "LIB_PICO_DIVIDER_HARDWARE=1"
  "LIB_PICO_DOUBLE=1"
  "LIB_PICO_DOUBLE_PICO=1"
  "LIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1"
  "LIB_PICO_FLOAT=1"
  "LIB_PICO_FLOAT_PICO=1"
  "LIB_PICO_INT64_OPS=1"
  "LIB_PICO_INT64_OPS_PICO=1"
  "LIB_PICO_MALLOC=1"
  "LIB_PICO_MEM_OPS=1"
  "LIB_PICO_MEM_OPS_PICO=1"
  "LIB_PICO_PLATFORM=1"
  "LIB_PICO_PRINTF=1"
  "LIB_PICO_PRINTF_PICO=1"
  "LIB_PICO_RUNTIME=1"
  "LIB_PICO_STANDARD_LINK=1"
  "LIB_PICO_STDIO=1"
  "LIB_PICO_STDIO_USB=1"
  "LIB_PICO_STDLIB=1"
  "LIB_PICO_SYNC=1"
  "LIB_PICO_SYNC_CORE=1"
  "LIB_PICO_SYNC_CRITICAL_SECTION=1"
  "LIB_PICO_SYNC_MUTEX=1"
  "LIB_PICO_SYNC_SEM=1"
  "LIB_PICO_TIME=1"
  "LIB_PICO_UNIQUE_ID=1"
  "LIB_PICO_UTIL=1"
  "PICO_BOARD=\"pico\""
  "PICO_BUILD=1"
  "PICO_CMAKE_BUILD_TYPE=\"Debug\""
  "PICO_COPY_TO_RAM=0"
  "PICO_CXX_ENABLE_EXCEPTIONS=1"
  "PICO_NO_FLASH=0"
  "PICO_NO_HARDWARE=0"
  "PICO_ON_DEVICE=1"
  "PICO_PROGRAM_NAME=\"main\""
  "PICO_PROGRAM_VERSION_STRING=\"0.1\""
  "PICO_TARGET_NAME=\"main\""
  "PICO_USE_BLOCKED_RAM=0"
  )

# The include file search paths:
set(CMAKE_ASM_TARGET_INCLUDE_PATH
  "../inc"
  "/home/kozha/pico/pico-sdk/src/common/pico_stdlib/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_gpio/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_base/include"
  "generated/pico_base"
  "/home/kozha/pico/pico-sdk/src/boards/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_platform/include"
  "/home/kozha/pico/pico-sdk/src/rp2040/hardware_regs/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_base/include"
  "/home/kozha/pico/pico-sdk/src/rp2040/hardware_structs/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_claim/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_sync/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_uart/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_divider/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_time/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_timer/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_util/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_runtime/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_clocks/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_resets/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pll/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_vreg/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_watchdog/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_xosc/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_printf/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_bootrom/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_bit_ops/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_divider/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_int64_ops/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_malloc/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/boot_stage2/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_binary_info/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/common"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/hw"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_unique_id/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_flash/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_usb_reset_interface/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pwm/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_i2c/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_spi/include"
  )
set(CMAKE_DEPENDS_CHECK_C
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/audio/audio_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/audio/audio_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/cdc/cdc_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/dfu/dfu_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/dfu/dfu_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/dfu/dfu_rt_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/dfu/dfu_rt_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/hid/hid_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/hid/hid_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/midi/midi_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/midi/midi_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/msc/msc_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/msc/msc_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/net/ecm_rndis_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/net/ecm_rndis_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/usbtmc/usbtmc_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/usbtmc/usbtmc_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/vendor/vendor_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/video/video_device.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/class/video/video_device.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/common/tusb_fifo.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/common/tusb_fifo.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/device/usbd.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/device/usbd.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/device/usbd_control.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/device/usbd_control.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c.obj"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/tusb.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/lib/tinyusb/src/tusb.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/critical_section.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_sync/critical_section.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/lock_core.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_sync/lock_core.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/mutex.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_sync/mutex.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/sem.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_sync/sem.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_time/time.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_time/time.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_time/timeout_helper.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_time/timeout_helper.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_util/datetime.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_util/datetime.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_util/pheap.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_util/pheap.c.obj"
  "/home/kozha/pico/pico-sdk/src/common/pico_util/queue.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/common/pico_util/queue.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_claim/claim.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_claim/claim.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_clocks/clocks.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_clocks/clocks.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_flash/flash.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_flash/flash.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_gpio/gpio.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_gpio/gpio.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_i2c/i2c.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_i2c/i2c.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/irq.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/irq.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pll/pll.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pll/pll.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_spi/spi.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_spi/spi.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_sync/sync.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_sync/sync.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_timer/timer.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_timer/timer.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_uart/uart.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_uart/uart.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_vreg/vreg.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_vreg/vreg.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_watchdog/watchdog.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_xosc/xosc.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/hardware_xosc/xosc.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_bootrom/bootrom.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_init_rom.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_init_rom.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_math.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/double_math.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/rp2040_usb_device_enumeration.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/rp2040_usb_device_enumeration.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_init_rom.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_init_rom.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_math.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/float_math.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_malloc/pico_malloc.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_platform/platform.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_platform/platform.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_printf/printf.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_printf/printf.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_runtime/runtime.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_runtime/runtime.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_standard_link/binary_info.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio/stdio.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio/stdio.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/reset_interface.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/reset_interface.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb_descriptors.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdlib/stdlib.c.obj"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_unique_id/unique_id.c" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_unique_id/unique_id.c.obj"
  )
set(CMAKE_C_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_C
  "CFG_TUSB_DEBUG=1"
  "CFG_TUSB_MCU=OPT_MCU_RP2040"
  "CFG_TUSB_OS=OPT_OS_PICO"
  "LIB_PICO_BIT_OPS=1"
  "LIB_PICO_BIT_OPS_PICO=1"
  "LIB_PICO_DIVIDER=1"
  "LIB_PICO_DIVIDER_HARDWARE=1"
  "LIB_PICO_DOUBLE=1"
  "LIB_PICO_DOUBLE_PICO=1"
  "LIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1"
  "LIB_PICO_FLOAT=1"
  "LIB_PICO_FLOAT_PICO=1"
  "LIB_PICO_INT64_OPS=1"
  "LIB_PICO_INT64_OPS_PICO=1"
  "LIB_PICO_MALLOC=1"
  "LIB_PICO_MEM_OPS=1"
  "LIB_PICO_MEM_OPS_PICO=1"
  "LIB_PICO_PLATFORM=1"
  "LIB_PICO_PRINTF=1"
  "LIB_PICO_PRINTF_PICO=1"
  "LIB_PICO_RUNTIME=1"
  "LIB_PICO_STANDARD_LINK=1"
  "LIB_PICO_STDIO=1"
  "LIB_PICO_STDIO_USB=1"
  "LIB_PICO_STDLIB=1"
  "LIB_PICO_SYNC=1"
  "LIB_PICO_SYNC_CORE=1"
  "LIB_PICO_SYNC_CRITICAL_SECTION=1"
  "LIB_PICO_SYNC_MUTEX=1"
  "LIB_PICO_SYNC_SEM=1"
  "LIB_PICO_TIME=1"
  "LIB_PICO_UNIQUE_ID=1"
  "LIB_PICO_UTIL=1"
  "PICO_BOARD=\"pico\""
  "PICO_BUILD=1"
  "PICO_CMAKE_BUILD_TYPE=\"Debug\""
  "PICO_COPY_TO_RAM=0"
  "PICO_CXX_ENABLE_EXCEPTIONS=1"
  "PICO_NO_FLASH=0"
  "PICO_NO_HARDWARE=0"
  "PICO_ON_DEVICE=1"
  "PICO_PROGRAM_NAME=\"main\""
  "PICO_PROGRAM_VERSION_STRING=\"0.1\""
  "PICO_TARGET_NAME=\"main\""
  "PICO_USE_BLOCKED_RAM=0"
  )

# The include file search paths:
set(CMAKE_C_TARGET_INCLUDE_PATH
  "../inc"
  "/home/kozha/pico/pico-sdk/src/common/pico_stdlib/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_gpio/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_base/include"
  "generated/pico_base"
  "/home/kozha/pico/pico-sdk/src/boards/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_platform/include"
  "/home/kozha/pico/pico-sdk/src/rp2040/hardware_regs/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_base/include"
  "/home/kozha/pico/pico-sdk/src/rp2040/hardware_structs/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_claim/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_sync/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_uart/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_divider/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_time/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_timer/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_util/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_runtime/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_clocks/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_resets/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pll/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_vreg/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_watchdog/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_xosc/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_printf/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_bootrom/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_bit_ops/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_divider/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_int64_ops/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_malloc/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/boot_stage2/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_binary_info/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/common"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/hw"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_unique_id/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_flash/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_usb_reset_interface/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pwm/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_i2c/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_spi/include"
  )
set(CMAKE_DEPENDS_CHECK_CXX
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/home/kozha/pico/pico-sdk/src/rp2_common/pico_standard_link/new_delete.cpp.obj"
  "/home/kozha/pico/Projects/QuadrupedV2/src/MPU6050.cpp" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/src/MPU6050.cpp.obj"
  "/home/kozha/pico/Projects/QuadrupedV2/src/MYSERVO.cpp" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/src/MYSERVO.cpp.obj"
  "/home/kozha/pico/Projects/QuadrupedV2/src/NRF24.cpp" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/src/NRF24.cpp.obj"
  "/home/kozha/pico/Projects/QuadrupedV2/src/main.cpp" "/home/kozha/pico/Projects/QuadrupedV2/build/CMakeFiles/main.dir/src/main.cpp.obj"
  )
set(CMAKE_CXX_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_CXX
  "CFG_TUSB_DEBUG=1"
  "CFG_TUSB_MCU=OPT_MCU_RP2040"
  "CFG_TUSB_OS=OPT_OS_PICO"
  "LIB_PICO_BIT_OPS=1"
  "LIB_PICO_BIT_OPS_PICO=1"
  "LIB_PICO_DIVIDER=1"
  "LIB_PICO_DIVIDER_HARDWARE=1"
  "LIB_PICO_DOUBLE=1"
  "LIB_PICO_DOUBLE_PICO=1"
  "LIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1"
  "LIB_PICO_FLOAT=1"
  "LIB_PICO_FLOAT_PICO=1"
  "LIB_PICO_INT64_OPS=1"
  "LIB_PICO_INT64_OPS_PICO=1"
  "LIB_PICO_MALLOC=1"
  "LIB_PICO_MEM_OPS=1"
  "LIB_PICO_MEM_OPS_PICO=1"
  "LIB_PICO_PLATFORM=1"
  "LIB_PICO_PRINTF=1"
  "LIB_PICO_PRINTF_PICO=1"
  "LIB_PICO_RUNTIME=1"
  "LIB_PICO_STANDARD_LINK=1"
  "LIB_PICO_STDIO=1"
  "LIB_PICO_STDIO_USB=1"
  "LIB_PICO_STDLIB=1"
  "LIB_PICO_SYNC=1"
  "LIB_PICO_SYNC_CORE=1"
  "LIB_PICO_SYNC_CRITICAL_SECTION=1"
  "LIB_PICO_SYNC_MUTEX=1"
  "LIB_PICO_SYNC_SEM=1"
  "LIB_PICO_TIME=1"
  "LIB_PICO_UNIQUE_ID=1"
  "LIB_PICO_UTIL=1"
  "PICO_BOARD=\"pico\""
  "PICO_BUILD=1"
  "PICO_CMAKE_BUILD_TYPE=\"Debug\""
  "PICO_COPY_TO_RAM=0"
  "PICO_CXX_ENABLE_EXCEPTIONS=1"
  "PICO_NO_FLASH=0"
  "PICO_NO_HARDWARE=0"
  "PICO_ON_DEVICE=1"
  "PICO_PROGRAM_NAME=\"main\""
  "PICO_PROGRAM_VERSION_STRING=\"0.1\""
  "PICO_TARGET_NAME=\"main\""
  "PICO_USE_BLOCKED_RAM=0"
  )

# The include file search paths:
set(CMAKE_CXX_TARGET_INCLUDE_PATH
  "../inc"
  "/home/kozha/pico/pico-sdk/src/common/pico_stdlib/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_gpio/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_base/include"
  "generated/pico_base"
  "/home/kozha/pico/pico-sdk/src/boards/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_platform/include"
  "/home/kozha/pico/pico-sdk/src/rp2040/hardware_regs/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_base/include"
  "/home/kozha/pico/pico-sdk/src/rp2040/hardware_structs/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_claim/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_sync/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_uart/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_divider/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_time/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_timer/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_sync/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_util/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_runtime/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_clocks/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_irq/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_resets/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pll/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_vreg/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_watchdog/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_xosc/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_printf/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_bootrom/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_bit_ops/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_divider/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_double/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_int64_ops/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_float/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_malloc/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/boot_stage2/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_binary_info/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_stdio_usb/include"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/src/common"
  "/home/kozha/pico/pico-sdk/lib/tinyusb/hw"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/pico_unique_id/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_flash/include"
  "/home/kozha/pico/pico-sdk/src/common/pico_usb_reset_interface/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_pwm/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_i2c/include"
  "/home/kozha/pico/pico-sdk/src/rp2_common/hardware_spi/include"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
