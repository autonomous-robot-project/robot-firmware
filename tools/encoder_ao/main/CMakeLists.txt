idf_component_register(SRCS "main.cpp"
                            "../../motor_characterization/main/edge_reporter.cpp"
                       INCLUDE_DIRS "." "../../motor_characterization/main"
                       REQUIRES controller driver esp_adc esp_timer vfs)
