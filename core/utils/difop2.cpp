/*******************************************************************************
 * \addtogroup utils
 * \{
 * \file difop2.cpp
 * \brief Defines functions of process DIFOP2 data.
 * \version 0.2
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-24 | Init version |
 * | 0.2 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "crc32.h"
#include "difop2.h"
#include "rs_new_logger.h"
#include "json_reader.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::utils {

/******************************************************************************/
/*                 Declaration or Definition of local variables               */
/******************************************************************************/
const std::string kBinPath {"/osdata/satelite/lidar/middle_lidar_inner_para.bin"};
const std::string kJsonPath {"/osdata/satelite/lidar/lidar_sn_inner_para_crc.json"};

std::array<char, kDifopLen> difop2_bin_data;
std::array<uint8_t, kDifopLen> difop2_mipi_data;

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Print hex data.
 * \param[in] data The data to be printed.
 * \param[in] size The size of data.
 */
void printHex(const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
/**
 * \brief Compare difop2 data.
 */
void compareDifop2() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(json_reader::test_param.print_difop2_cycle));

        if ((!json_reader::test_param.data_valid) || (!json_reader::test_param.enable_print_difop2)) {
            LogTrace("Difop2 data is not valid or not enable print difop2");
            continue;
        }
        if (!readBinData(kBinPath)) {
            LogError("Failed to read bin file: {}", kBinPath);
            printHex(difop2_mipi_data.data(), kDifopLen);
            continue;
        }
        uint8_t* difop2_bin = reinterpret_cast<uint8_t*>(difop2_bin_data.data());
        uint32_t calculate_crc32 {0U};
        uint32_t expected_crc32 {0U};

        if (crc32::CrcErrorCode::SUCCESS == crc32::read_json_file_crc32(kJsonPath, expected_crc32)) {
            calculate_crc32 = crc32::calculate_crc32(difop2_bin, kDifopLen - 4);
            bool result = (calculate_crc32 == expected_crc32);

            if (result)
                LogInfo("verify difop2 crc32 result: {}", result);
            else {
                LogError("verify difop2 crc32 result: {}, crc in json: {}, calculated crc: {}", result, expected_crc32,
                         calculate_crc32);
            }
        }
        if (0 != memcmp(difop2_mipi_data.data(), difop2_bin, kDifopLen - 4)) {
            LogError("Difop2 data do not match, difop2 data form mipi: \n");
            std::cerr << "difop2_mipi: ";
            printHex(difop2_mipi_data.data(), kDifopLen);
            std::cerr << "difop2_bin: ";
            LogError("Difop2 data do not match, difop2 data form bin: \n");
            printHex(difop2_bin, kDifopLen - 4);
        } else {
            LogInfo("Difop2 mipi match");
            std::cout << "difop2 data are matched, difop2 data in bin: \n";
            printHex(difop2_bin, kDifopLen - 4);
        }
    }
}

/**
 * \brief Read bin data from bin file.
 * \param[in] file_path The path of bin file.
 * \return true or false.
 * \retval true: Bin data reading succeeded.
 * \retval false: Bin data reading failed.
 */
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
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (file_size != kDifopLen - 4) {
        LogError("File size {} not equal to difop2 size {}", file_size, kDifopLen - 4);
        file.close();
        return false;
    }
    // Read file content to memory
    if (!file.read(difop2_bin_data.data(), file_size)) {
        LogError("Failed to read file: {}", file_path);
        file.close();
        return false;
    }
    // Close file
    file.close();

    return true;
}

} // namespace robosense::lidar::utils

/* \}  utils */
