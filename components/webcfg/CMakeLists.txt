idf_component_register(
    SRCS "webcfg.c" "url_decode.c"
    INCLUDE_DIRS "include" "."
    REQUIRES esp_wifi esp_http_server esp_netif esp_event nvs_flash app_update
             camlink duml msp)

# Gzip the settings page at build time rather than committing a compressed
# blob: the source stays plain HTML, so it is readable and diffable, while what
# ships is the compressed form embedded in the firmware image. One binary, one
# flash step, and no way for the page and the firmware to drift apart.
set(WEB_SRC "${COMPONENT_DIR}/web/index.html")
set(WEB_GZ  "${CMAKE_CURRENT_BINARY_DIR}/index.html.gz")

add_custom_command(
    OUTPUT  "${WEB_GZ}"
    COMMAND gzip -9 -n -c "${WEB_SRC}" > "${WEB_GZ}"
    DEPENDS "${WEB_SRC}"
    COMMENT "Compressing settings page"
    VERBATIM)
add_custom_target(webcfg_page DEPENDS "${WEB_GZ}")
add_dependencies(${COMPONENT_LIB} webcfg_page)

target_add_binary_data(${COMPONENT_LIB} "${WEB_GZ}" BINARY)
