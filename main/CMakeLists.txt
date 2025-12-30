idf_component_register(SRCS "mesure_task.c" "mcu_alive_twdt.c" "system_queues.c" "system_state.c" "pca_mgr_task.c" "initialisation.c" "FRAM_module.c" "test.c" "PCF8523_module.c" "E22_module.c" "PCA9536_module.c" "main.c" "C3_module.c"
                      INCLUDE_DIRS "."
                      REQUIRES esp_timer freertos driver)
