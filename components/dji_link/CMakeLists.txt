# DJI R SDK protocol stack, vendored from dji-sdk/Osmo-GPS-Controller-Demo.
#
# Stripped: gps_logic, key_logic, light_logic, test/ and the GPS push command.
# (gps_logic.c also would not compile for esp32c3 -- it uses LP_UART_NUM_0,
# which only exists on the esp32c6 the upstream demo targets.)
#
# Local modifications are marked "SLATE PATCH" / "SLATE ADDITION" in the
# sources, so the derived work is separable from DJI's original by reading it.
idf_component_register(
    SRCS "ble/ble.c"
         "ble/cam_scan.c"
         "data/data.c"
         "protocol/dji_protocol_parser.c"
         "protocol/dji_protocol_data_processor.c"
         "protocol/dji_protocol_data_descriptors.c"
         "protocol/dji_protocol_data_structures.c"
         "logic/connect_logic.c"
         "logic/command_logic.c"
         "logic/status_logic.c"
         "logic/enums_logic.c"
         "utils/crc/custom_crc16.c"
         "utils/crc/custom_crc32.c"
    INCLUDE_DIRS "ble" "data" "protocol" "logic" "utils/crc"
    REQUIRES bt esp_timer board
    PRIV_REQUIRES nvs_flash)
