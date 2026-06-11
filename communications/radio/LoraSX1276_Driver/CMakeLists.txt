set(srcs
    "Lora_SX1276_Driver.c"
)

set(includes "inc/")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    REQUIRES "driver" "Utils" "PeripheralManager"
)
