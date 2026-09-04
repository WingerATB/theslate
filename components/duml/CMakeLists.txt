idf_component_register(SRCS "duml.c" "duml_cam.c"
                       INCLUDE_DIRS "include"
                       REQUIRES dji_link gopro esp_timer nvs_flash bt)
