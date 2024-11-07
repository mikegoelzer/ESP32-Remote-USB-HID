set(requires "")
if(${IDF_TARGET} STREQUAL "linux")
    list(APPEND requires esp_stubs esp-tls esp_http_server protocol_examples_common nvs_flash)
endif()
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
idf_component_register(SRCS "main.c" "httpd.c" "globals.c" "tinyusb_hid.c" "../hid_device/usb_descriptors.c" "../managed_components/espressif__tinyusb/src/class/hid/hid_device.c"
                    INCLUDE_DIRS "." "../hid_device"
                    REQUIRES ${requires})
cmake_policy(SET CMP0079 NEW)
target_link_libraries(${tusb_lib} PRIVATE ${COMPONENT_LIB})