# Depends on nothing in this tree on purpose: everything else depends on it, so
# a cycle here would be a cycle everywhere. bt is for esp_bt.h's power enum.
idf_component_register(SRCS "board.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt)
