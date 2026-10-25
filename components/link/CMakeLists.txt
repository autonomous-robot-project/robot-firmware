idf_component_register(SRCS "src/link.cpp" "src/transport.cpp"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       PRIV_REQUIRES driver micro_ros_espidf_component)
