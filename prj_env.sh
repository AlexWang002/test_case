#!/bin/bash

PRJ_DIR=$(pwd)
echo "root_path: $PRJ_DIR"

PRJ_NO="0632"

CMAKE_FILE="$PRJ_DIR/control/build.cmake"

LOG_DIR="$PRJ_DIR/make_logs"

RESULT_DIR="$PRJ_DIR/make_results"

BUILD_DIR="$PRJ_DIR/build"

RELEASE_DIR="$PRJ_DIR/release/$ARCH"

WARN_LOG="$LOG_DIR/war.log"

CMAKE_LOG="$LOG_DIR/cmake.log"

MAKE_LOG="$LOG_DIR/make.log"

INSTALL_LOG="$LOG_DIR/install.log"

STATIC_DIR="$PRJ_DIR/static_results"

ARCH="pc"

set_prj_no() {
    PRJ_NO="0632"
}

set_arch() {
    ARCH=$(echo "$1" | tr '[:upper:]' '[:lower:]')

    if [ "$ARCH" == "pc" ] || [ "$ARCH" == "arm" ]; then  # Check if $2 is right
        echo "Current platform is $ARCH"
    else
        ARCH=""
        echo "Invalid platform $1, Input platform format: 'pc' or 'arm'"
        exit 1
    fi
}

set_cmake() {
    if [ $# -eq 2 ]; then
        set_prj_no "$1"
        set_arch "$2"
    fi

    # Update the PRJ_NO in the build.cmake
    sed -i "s/^set(custom .*/set(custom $PRJ_NO)/" "$CMAKE_FILE"
    sed -i "s/^add_definitions(-DCUSTOM_.*/add_definitions(-DCUSTOM_$PRJ_NO)/" "$CMAKE_FILE"
    echo "set(custom $PRJ_NO)"

    # Convert arch type to lower case ',,'
    sed -i "s/^set(platform .*/set(platform ${ARCH,,})/" "$CMAKE_FILE"
    # Convert arch type to upper case '^^'
    sed -i "s/^add_definitions(-DPLATFORM_.*/add_definitions(-DPLATFORM_${ARCH^^})/" "$CMAKE_FILE"
    echo "set(platform $ARCH)"

    RELEASE_DIR="$PRJ_DIR/release/$ARCH"
}

# Create new directory
create_dir() {
    echo "$1"
    if [ -d "$1" ]; then
        rm -rf "$1"
    fi
    mkdir -p "$1"
    ls -la "$1"
}

# Add directory if not exist
add_dir() {
    echo "$1"
    if [ ! -d "$1" ]; then
        # echo "dir $1 already exists"
        mkdir -p "$1"
    # else
        # echo "dir $1 already exists"
    fi
}

set_imposter_param() {
    export IMPOSTER_MODULES_IN_WORKING_DIR=1
    export IMPOSTER_PATH_ARGUMENT_RELATIVE_TO_WORKING_DIR_OPTION_INTRODUCERS=-isystem:-I
    export IMPOSTER_COMPILER_ARG1=1
}

set_pclp_path() {
    export PCLP="$1/pclp64_linux"
    export PCLP_CONFIG_PATH="$1/config"
    export PCLP_LNT_PATH="$1/lnt"
}
