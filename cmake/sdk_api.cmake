# Build AlgoSDK API
cmake_minimum_required(VERSION 3.10)

add_library(${PROJECT_NAME} ${SDK_API_TYPE} ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/api
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/3rdparty
    ${CMAKE_SOURCE_DIR}/core/utils
    ${CMAKE_SOURCE_DIR}/core/decoder/include
)

target_link_libraries(${PROJECT_NAME}
    rs_logger
    yaml-cpp
    pthread
)