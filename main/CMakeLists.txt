idf_component_register(SRCS "initialisation.c" "FRAM_module.c" "test.c" "PCF8523_module.c" "E22_module.c" "PCA9536_module.c" "main.c" "C3_module.c"
                      INCLUDE_DIRS "."
                      REQUIRES driver esp_timer)
