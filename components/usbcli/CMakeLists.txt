idf_component_register(SRCS "usbcli.c"
                       INCLUDE_DIRS "include"
                       REQUIRES camlink duml dji_link gopro msp webcfg
                       PRIV_REQUIRES hal esp_timer esp_app_format)
