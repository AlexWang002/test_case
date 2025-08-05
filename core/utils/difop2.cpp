/*******************************************************************************
 * \addtogroup utils
 * \{
 * \file difop2.cpp
 * \brief
 * \version 0.1
 * \date 2025-07-24
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-24 | Init version |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <array>
#include <vector>
#include <unistd.h>
#include <chrono>
#include "rs_new_logger.h"
#include "crc32.h"
#include "yaml_manager.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "difop2.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

namespace robosense::lidar::utils {

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/
std::array<char, DIFOP_LEN - 4> difop2_bin_data;
std::array<uint8_t, DIFOP_LEN> difop2_mipi_data;
// std::chrono::steady_clock::time_point last_call_time = std::chrono::steady_clock::now();

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/
const std::string BIN_PATH {"/osdata/dimwcfg/sensor/lidar/middle_lidar_inner_para.bin"};
const std::string JSON_PATH {"/osdata/dimwcfg/sensor/lidar/middle_lidar_inner_para.json"};

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
bool readBinData(const std::string& file_path) {
    if (0 != access(file_path.c_str(), F_OK)) {
        LogError("File {} not exist", file_path);
        return false;
    }
    if (0 != access(file_path.c_str(), R_OK)) {
        LogError("File {} exist but not readable", file_path);
        return false;
    }
    // Open the bin file
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        LogError("Failed to open file: {}", file_path);
        return false;
    }
    // Get the size of bin file
    file.seekg(0, std::ios::end);
    // std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Read file content to memory
    // std::vector<char> buffer(file_size);
    if (!file.read(difop2_bin_data.data(), DIFOP_LEN - 4)) {
        LogError("Failed to read file: {}", file_path);
        file.close();
        return false;
    }
    // Close file
    file.close();

    return true;
}

void printHex(const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
void compareDifop2() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(yaml::demo_test_param.print_difop2_cycle));

        if (!yaml::demo_test_param.data_valid || !yaml::demo_test_param.enable_print_difop2) {
            LogError("Difop2 data is not valid or not enable print difop2");
            continue;
        }
        if (! readBinData(BIN_PATH)) {
            LogError("Failed to read bin file: {}", BIN_PATH);
            printHex(difop2_mipi_data.data(), DIFOP_LEN);
            continue;
        }
        uint8_t* difop2_bin =  reinterpret_cast<uint8_t*>(difop2_bin_data.data());
        uint32_t calculate_crc32 {0};
        uint32_t expected_crc32 {0};

        if (crc32::CrcErrorCode::SUCCESS == crc32::read_json_file_crc32(JSON_PATH, expected_crc32)) {
            calculate_crc32 = crc32::calculate_crc32(difop2_bin, DIFOP_LEN - 4);
            bool result = (calculate_crc32 == expected_crc32);

            if (result)
                LogInfo("verify difop2 crc32 result: {}", result);
            else {
                LogError("verify difop2 crc32 result: {}, crc in json: {}, calculated crc: {}", result, expected_crc32, calculate_crc32);
            }
        }
        if (0 != memcmp(difop2_mipi_data.data(), difop2_bin, DIFOP_LEN - 4)) {
            LogError("Difop2 data do not match, difop2 data form mipi: \n");
            std::cerr << "difop2_mipi: ";
            printHex(difop2_mipi_data.data(), DIFOP_LEN);
            std::cerr << "difop2_bin: ";
            LogError("Difop2 data do not match, difop2 data form bin: \n");
            printHex(difop2_bin, DIFOP_LEN - 4);
        } else {
            LogInfo("Difop2 mipi match");
            std::cout << "difop2 data are matched, difop2 data in bin: \n";
            printHex(difop2_bin, DIFOP_LEN - 4);
        }
    }
}

/******************************************************************************/
/*          Definition of public functions of classes or templates             */
/******************************************************************************/

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/

} // namespace robosense::lidar::utils

/* \}  utils */