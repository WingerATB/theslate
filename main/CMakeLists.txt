# The bench console is optional and lives outside this tree, in dev/, which the
# published repository does not carry.
#
# Keyed on the directory existing, NOT on CONFIG_CAMLINK_BENCH: REQUIRES is read
# during early expansion, before Kconfig has been processed, so CONFIG_ symbols
# are simply not defined here yet. Asking whether the source exists is a
# question plain CMake can answer at that point. Whether the console is actually
# compiled in is still the config's decision -- bench.c is empty without it.
set(BENCH_REQ "")
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../dev/components/bench")
    set(BENCH_REQ "bench")
endif()

# esp_driver_usb_serial_jtag is still deliberately absent: nothing here calls
# the driver, and the component does not exist for a target without the
# peripheral, so requiring it would break the original ESP32 build.
#
# What app_main DOES use is the register-level header, hal/usb_serial_jtag_ll.h,
# to ask whether a USB host is sending start-of-frame packets. That lives in
# "hal", which every target has, and the one call site is behind
# SOC_USB_SERIAL_JTAG_SUPPORTED so a part without the peripheral compiles the
# stub instead.
idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES camlink msp dji_link duml ui webcfg board
                                nvs_flash bt hal esp_driver_gpio ${BENCH_REQ})
