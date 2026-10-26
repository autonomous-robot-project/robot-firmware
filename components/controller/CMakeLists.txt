idf_component_register(SRCS "src/controller.cpp"
                            "src/instrumentation/expiry_meter.cpp"
                            "src/actuation/driver_input.cpp"
                            "src/actuation/motor_driver.cpp"
                            "src/supervision/task_watchdog.cpp"
                            "src/supervision/watchdog_test_trigger.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       PRIV_INCLUDE_DIRS "private_include"
                       PRIV_REQUIRES esp_timer)
