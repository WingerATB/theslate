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

idf_component_register(SRCS "app_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES camlink msp dji_link duml ui webcfg usbcli nvs_flash bt
                                esp_driver_usb_serial_jtag ${BENCH_REQ})
