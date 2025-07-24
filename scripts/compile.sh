#!/bin/bash

# 显示帮助信息
function show_help {
  echo "Usage: \$0 [OPTION]"
  echo "Build and install the project."
  echo ""
  echo "Options:"
  echo "  -c , --clean         Perform a clean build (delete build directory contents)"
  echo "  -i , --install       Install the project after build)"
  echo "  -p, --platform       Specify platform [pc|arm] (default: pc)"
  echo "  --build-dir DIR     Specify the build directory (default: build)"
  echo "  --cmake-arg ARG     Pass additional arguments to CMake"
  echo "  -h, --help          Show this help message"
  echo ""
  echo "If no options are provided, an incremental build will be performed."
}

# 默认参数
CLEAN=false
BUILD_DIR="build"
PLATFORM="pc"
CMAKE_ARGS=""
INSTALL=false

# 解析命令行参数
while [ "$#" -gt 0 ]; do
  case $1 in
    -c|--clean)
      CLEAN=true
      shift
      ;;
    -i|--install)
      INSTALL=true
      shift
      ;;
    --build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    -p|--platform)
      if [ -z "$2" ]; then
        echo "Error: --platform requires a platform argument [pc|arm]"
        show_help
        exit 1
      fi
      if [ "$2" != "pc" ] && [ "$2" != "arm" ]; then
        echo "Error: Unknown platform '$2'. Supported platforms: pc, arm"
        show_help
        exit 1
      fi
      PLATFORM="$2"
      shift 2
      ;;
    --cmake-arg)
      CMAKE_ARGS="$CMAKE_ARGS $2"
      shift 2
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      echo "Unknown argument: \$1"
      show_help
      exit 1
      ;;
  esac
done

# 根据平台设置CMake参数
case "$PLATFORM" in
  "pc")
    CMAKE_PLATFORM_ARGS=""
    ;;
  "arm")
    CMAKE_PLATFORM_ARGS="-DCMAKE_C_COMPILER=/usr/bin/aarch64-linux-gnu-gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++-9 -DCOMPILE_ARM_VERSION=ON"
    BUILD_DIR="build_arm"
    ;;
  *)
    echo "Error: Unsupported platform '$PLATFORM'"
    show_help
    exit 1
    ;;
esac

# 创建或进入 build 目录
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR" || { echo "Failed to enter build directory: $BUILD_DIR"; exit 1; }

# 如果需要重新编译，删除 build 目录中的内容
if $CLEAN; then
  echo "Performing clean build (deleting contents of the build directory)..."
  /bin/rm -rf ./* || { echo "Failed to clean build directory"; exit 1; }
fi

echo "Building and installing the project for platform '$PLATFORM'..."
# 运行 cmake 配置
echo "Running cmake..."
cmake .. $CMAKE_ARGS $CMAKE_PLATFORM_ARGS || { echo "CMake failed"; exit 1; }

# 使用 make 编译
echo "Compiling the project..."
make -j8 || { echo "Make failed"; exit 1; }

if $INSTALL; then
# 安装编译结果
echo "Installing the project..."
make install || { echo "Make install failed"; exit 1; }
echo "Build and installation completed successfully!"
fi

cd ..
