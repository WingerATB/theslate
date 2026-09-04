idf_component_register(SRCS "camlink.c" "osd_format.c" "camlink_cfg.c"
                       INCLUDE_DIRS "include"
                       # dji_link and bt are for camlink_ble_apply_tx_level(), which turns the radio
                       # down while armed. They resolved transitively through duml before being
                       # declared, which works right up until duml stops needing them.
                       REQUIRES msp duml dji_link board bt esp_timer nvs_flash)
