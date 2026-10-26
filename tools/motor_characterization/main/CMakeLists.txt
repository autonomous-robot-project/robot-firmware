idf_component_register(SRCS "main.cpp"
                            "duty_schedule.cpp"
                            "edge_reporter.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES controller driver esp_timer vfs)
