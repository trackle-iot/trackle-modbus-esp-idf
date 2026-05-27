# This CMakeLists.txt is only a ESP-IDF component declaration
set(LIBLIGHTMODBUS_REPO_DIR "${CMAKE_CURRENT_LIST_DIR}/repo")

# Copy config file to liblightmodbus include directory
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/src/esp.config.h"
    "${LIBLIGHTMODBUS_REPO_DIR}/include/lightmodbus/esp.config.h"
    COPYONLY
)

# Register component
idf_component_register(
    SRCS "src/impl.c" "src/trackle_modbus_rtu.c" "src/trackle_modbus_tcp.c" "src/modbus_hal_esp32.c"
    INCLUDE_DIRS "." "${LIBLIGHTMODBUS_REPO_DIR}/include"
    REQUIRES driver trackle-library-esp-idf
)

# Compatibility with esp-idf 5.0 by suppressing error
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)

# Inform liblightmodbus to use ESP config header
idf_build_set_property(COMPILE_DEFINITIONS "-DLIGHTMODBUS_USE_CONFIG_FILE" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DLIGHTMODBUS_CONFIG_FILE=\"esp.config.h\"" APPEND)