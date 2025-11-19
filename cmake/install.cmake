# Installation Configuration
cmake_minimum_required(VERSION 3.10)


install(FILES   ${CMAKE_SOURCE_DIR}/README.md
                ${CMAKE_SOURCE_DIR}/config/ag_config.json
    DESTINATION ${INSTALL_DIR}
)
install(FILES   ${CMAKE_SOURCE_DIR}/api/lidar_sdk_api.h
                ${CMAKE_SOURCE_DIR}/api/rs_lidar_sdk_api.h
    DESTINATION ${INSTALL_DIR}/include
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${INSTALL_DIR}/lib
    ARCHIVE DESTINATION ${INSTALL_DIR}/lib
    RUNTIME DESTINATION ${INSTALL_DIR}/bin
)

if(COMPILE_ARM_VERSION)
    install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/arm_test
        ARCHIVE DESTINATION ${CMAKE_SOURCE_DIR}/arm_test
    )
    install(FILES ${CMAKE_SOURCE_DIR}/config/ag_config.json
    DESTINATION ${CMAKE_SOURCE_DIR}/arm_test
)
endif()