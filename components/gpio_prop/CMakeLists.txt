if(CONFIG_ENABLE_GPIO)
    set(srcs gpio_prop.c)
else()
    set(srcs gpio_prop-dummy.c)
endif()

idf_component_register(SRCS "gpio_prop.c" "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES puzzle shift_reg sound twai_can)