#!/bin/bash

echo 'Beginning static code scanning using PC-Lint...'
echo '######## 1. Check project info ########'
# if [ $# -ne 2 ]; then
#     echo "ERROR: Input error."
#     exit 1
# fi

CUR_DIR=$(cd "$(dirname "$0")" || exit 1; pwd)
echo "Current directory: $CUR_DIR"
source "$CUR_DIR"/../../prj_env.sh
echo "$USER"

# if [ "$ARCH" != "pc" ]; then
#     echo "ERROR: Only support pc."
#     exit 1
# fi

# Create or add directory.
add_dir "$STATIC_DIR"
add_dir "$RELEASE_DIR"
# create_dir "$BUILD_DIR"
cd "$BUILD_DIR" || exit 1

echo "path: $(pwd)"

# Config the Imposter parameters.
set_imposter_param
# Set the path of PCLP.
set_pclp_path "/home/$USER/pclp"

# Cmake to build the compile commands file.
# cmake -DCMAKE_TOOLCHAIN_FILE="$CMAKE_FILE" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$PRJ_DIR"

# Generate the lnt file and header file for gcc/g++ compiler.
python3.8 "$PCLP_CONFIG_PATH/pclp_config.py" \
  --compiler=gcc \
  --compiler-bin="$(which gcc)" \
  --compilation-db=./compile_commands.json \
  --config-output-lnt-file=co-g++.lnt \
  --config-output-header-file=co-g++.h \
  --compiler-cpp-option="-std=c++14" \
  --generate-compiler-config

# Generate the lnt file for this cmake project.
python3.8 "$PCLP_CONFIG_PATH/pclp_config.py" \
  --compiler=gcc \
  --compilation-db=./compile_commands.json \
  --config-output-lnt-file=project_origin.lnt \
  --generate-project-config

# Set the Pc-lint rules and ouput rules.
# echo "path is : ==========> $PCLP_LNT_PATH/au-autosar19.lnt"
cp "$PCLP_LNT_PATH/env-html.js" ./
cp "$PCLP_LNT_PATH/env-xml.lnt" ./
cp "$PCLP_LNT_PATH/env-html.lnt" ./
cp "$PCLP_LNT_PATH/au-autosar19.lnt" ./
# cp "$PCLP_LNT_PATH/au-misra-cpp.lnt" ./
cp "$PRJ_DIR"/scripts/pclp/*.py ./

python3.8 ./generate-lnt.py \
    "$BUILD_DIR"/project_origin.lnt \
    /usr/include \
    "$PRJ_NO"

cat ./std.lnt

echo "$PCLP"
# Use the Pc-lint to static test the project.
$PCLP -max_threads=4 ./std.lnt

sed -i "s#${PRJ_DIR}#$(basename "$PRJ_DIR")#g" "$BUILD_DIR"/xml_results_"$PRJ_NO".xml
awk '!seen[$0]++' "$BUILD_DIR"/xml_results_"$PRJ_NO".xml > temp.xml
python3.8 ./generate-reports.py \
    --input-xml "$BUILD_DIR"/temp.xml \
    --output-text "$BUILD_DIR"/text_results.txt \
    --output-html "$BUILD_DIR"/html_results.html \
    --output-excel "$BUILD_DIR"/excel_results.xls
    # --input-xml "$BUILD_DIR"/xml_results_"$PRJ_NO".xml \
    # --output-html "$BUILD_DIR"/html_results.html

# # Save the result into release folder.
mv ./*xml "$STATIC_DIR"
mv ./*txt "$STATIC_DIR"
mv ./*html "$STATIC_DIR"
mv ./*xls "$STATIC_DIR"
mv "$STATIC_DIR" "$RELEASE_DIR"