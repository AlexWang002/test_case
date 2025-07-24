# Cmake modules
include(CheckCXXCompilerFlag)

# 检查编译器是否支持 -fPIC 标志
check_cxx_compiler_flag(-fPIC SUPPORT_FPIC)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
