idf_component_register(SRCS "src/agent.cpp"
                            "src/link.cpp"
                            "src/message_conversion.cpp"
                            "src/session.cpp"
                            "src/uart_transport.cpp"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       PRIV_REQUIRES controller driver micro_ros_espidf_component)
