# The GoPro session. Depends on the shared BLE transport and on the vendor
# facts, and on nothing DJI -- no DUML or R SDK code is on this path.
idf_component_register(SRCS "gopro_cam.c"
                       INCLUDE_DIRS "include"
                       REQUIRES dji_link camvendor bt esp_timer)
