# Build AlgoSDK API
cmake_minimum_required(VERSION 3.20)
find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BUILD_SHARED_LIBS ON)

add_library(${PROJECT_NAME} ${SDK_API_TYPE} ${SOURCES})

add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/trail_programm)
add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/denoise_programm)
add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/groundfit_programm)
add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/upsample_programm)
add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/stray_programm)
add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/spray_programm)
add_subdirectory(${CMAKE_SOURCE_DIR}/core/algo/high_programm)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/api
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/core/algo
    ${CMAKE_SOURCE_DIR}/core/algo/trail_programm
    ${CMAKE_SOURCE_DIR}/core/algo/trail_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/core/algo/denoise_programm
    ${CMAKE_SOURCE_DIR}/core/algo/denoise_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/core/algo/groundfit_programm
    ${CMAKE_SOURCE_DIR}/core/algo/groundfit_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/core/algo/upsample_programm
    ${CMAKE_SOURCE_DIR}/core/algo/upsample_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/core/algo/stray_programm
    ${CMAKE_SOURCE_DIR}/core/algo/stray_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/core/algo/spray_programm
    ${CMAKE_SOURCE_DIR}/core/algo/spray_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/core/algo/high_programm
    ${CMAKE_SOURCE_DIR}/core/algo/high_programm/host_cpp_api
    ${CMAKE_SOURCE_DIR}/3rdparty
    ${CMAKE_SOURCE_DIR}/3rdparty/dimw/include
    ${CMAKE_SOURCE_DIR}/3rdparty/rs_logger/lib/spdlog/include
    ${CMAKE_SOURCE_DIR}/core/utils
    ${CMAKE_SOURCE_DIR}/core/decoder
)

target_link_libraries(${PROJECT_NAME}
    rs_logger
    pthread
    trail_api
    denoise_api
    groundfit_api
    upsample_api
    stray_api
    spray_api
    highcalc_api
)