cmake_minimum_required(VERSION 3.20)

find_package(pva-sdk 2.5 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(demo)

message("=============================================================")
message("-- Ready to compile demos")
message("=============================================================")


if(COMPILE_ARM_VERSION)
    add_executable(rs_demo_online demo_online.cpp rs_msop_parse.cpp ${CMAKE_SOURCE_DIR}/core/utils/sync_queue.h)
    target_link_libraries(rs_demo_online  "-Wl,-unresolved-symbols=ignore-in-shared-libs"
        lidar_robosense_SDK
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw/lib/libsensor_ipcclient.so
    )

    target_include_directories(rs_demo_online
        PRIVATE
        ${CMAKE_SOURCE_DIR}/api
        ${CMAKE_SOURCE_DIR}/core/utils
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw/include/
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw/include/CameraAccess
    )
    install(TARGETS rs_demo_online  DESTINATION ${CMAKE_SOURCE_DIR}/arm_test)

    add_executable(rs_demo_offline demo_offline.cpp rs_msop_parse.cpp ${CMAKE_SOURCE_DIR}/core/utils/sync_queue.h)
    target_link_libraries(rs_demo_offline  "-Wl,-unresolved-symbols=ignore-in-shared-libs"
        lidar_robosense_SDK
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw/lib/libsensor_ipcclient.so
    )

    target_include_directories(rs_demo_offline
        PRIVATE
        ${CMAKE_SOURCE_DIR}/api
        ${CMAKE_SOURCE_DIR}/core/utils
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw/include/
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw/include/CameraAccess
    )
    install(TARGETS rs_demo_offline  DESTINATION ${CMAKE_SOURCE_DIR}/arm_test)
else()
    add_executable(rs_demo_offline demo_offline.cpp rs_msop_parse.cpp ${CMAKE_SOURCE_DIR}/core/utils/sync_queue.h)
    target_link_libraries(rs_demo_offline PRIVATE
        lidar_robosense_SDK
    )
endif()