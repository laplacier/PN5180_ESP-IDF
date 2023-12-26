idf_component_register(SRCS "iso15693.c" "pn5180.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver puzzle twai_can)