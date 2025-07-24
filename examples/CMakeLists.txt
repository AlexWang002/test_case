cmake_minimum_required(VERSION 3.10)

project(demo)

message("=============================================================")
message("-- Ready to compile demos")
message("=============================================================")


if(COMPILE_ARM_VERSION)
    add_executable(demo_online_x2b demo_online_x2b.cpp ${CMAKE_SOURCE_DIR}/core/utils/sync_queue.h)
    target_link_libraries(demo_online_x2b  "-Wl,-unresolved-symbols=ignore-in-shared-libs"
        lidar_robosense_SDK
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw_x2b/lib/libsensor_access.so
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw_x2b/lib/libsensor_ipcclient.so
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw_x2b/lib/libsensor-nvstream.so
    )

    target_include_directories(demo_online_x2b
        PRIVATE
        ${CMAKE_SOURCE_DIR}/api
        ${CMAKE_SOURCE_DIR}/core/utils
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw_x2b/include/
        ${CMAKE_SOURCE_DIR}/3rdparty/dimw_x2b/include/sensorAccess
    )
    install(TARGETS demo_online_x2b  DESTINATION ${CMAKE_SOURCE_DIR}/arm_test)

    add_executable(rs_demo_online demo_online.cpp ${CMAKE_SOURCE_DIR}/core/utils/sync_queue.h)
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
else()
    add_executable(rs_demo_offline demo_offline.cpp)
    target_link_libraries(rs_demo_offline PRIVATE
        lidar_robosense_SDK
    )
endif()