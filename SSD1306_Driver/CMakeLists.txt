set(srcs
 "SSD1306_Driver.c"
)

set(includes "inc/")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    REQUIRES "driver" "PeripheralManager" "Utils"
)
