set(srcs
    "JsonAPIManager.c"   
)

set(includes "inc/"
"${IDF_PATH}/components/esp_http_server/src/"
"${IDF_PATH}/components/esp_http_server/src/port/esp32/"
)

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    REQUIRES "DeviceRegistry" "esp_http_server" "json" "Utils"
)
