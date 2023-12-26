idf_component_register(SRCS "puzzle.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver twai_can sound gpio_prop shift_reg nfc)