/*******************************************************************************
 * \addtogroup utils
 * \{
 * \file crc32.cpp
 * \brief
 * \version 0.1
 * \date 2025-07-13
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-13 | Init version, add bin file reader and crc32 calculator |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include "rs_new_logger.h"
#include "json.hpp"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "crc32.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
namespace robosense::lidar::crc32 {

using json = nlohmann::json;

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
bool fileExist(const std::string& file_path) {
    if (0 != access(file_path.c_str(), F_OK)) {
        LogError("File {} not exist", file_path);
        return false;
    }
    if (0 != access(file_path.c_str(), R_OK)) {
        LogError("File {} exist but not readable", file_path);
        return false;
    }

    return true;
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
CrcErrorCode calculate_bin_file_crc32(const std::string& file_path, uint32_t& result) {
    if (0 != file_path.compare(file_path.size() - 4, 4, ".bin")) {
        LogError("Input file path: {} not end with .bin", file_path);
        return CrcErrorCode::FILE_NOT_BIN;
    }
    if (!fileExist(file_path)) {
        return CrcErrorCode::FILE_NOT_EXIST;
    }
    // Open the bin file
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        LogError("Failed to open file: {}", file_path);
        return CrcErrorCode::FILE_OPEN_ERROR;
    }
    // Get the size of bin file
    file.seekg(0, std::ios::end);
    std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Read file content to memory
    std::vector<char> buffer(file_size);
    if (!file.read(buffer.data(), file_size)) {
        LogError("Failed to read file: {}", file_path);
        file.close();
        return CrcErrorCode::FILE_READ_ERROR;
    }
    // Close file
    file.close();
    result = calculate_crc32(buffer.data(), file_size);
    LogTrace("crc32 of bin file: 0x{:08x}", result);

    return CrcErrorCode::SUCCESS;
}

CrcErrorCode read_json_file_crc32(const std::string& file_path, uint32_t& result) {
    if (0 != file_path.compare(file_path.size() - 5, 5, ".json")) {
        LogError("Input file path: {} not end with .json", file_path);
        return CrcErrorCode::FILE_NOT_JSON;
    }
    if (!fileExist(file_path)) {
        return CrcErrorCode::FILE_NOT_EXIST;
    }
    std::ifstream json_file(file_path);
    if (!json_file.is_open()) {
        LogError("Failed to open json file: {}", file_path);
        return CrcErrorCode::FILE_OPEN_ERROR;
    }

    try {
        json json_data = json::parse(json_file);

        if (!json_data.contains("middle_lidar_crc")) {
            LogError("File: {} does not contain 'middle_lidar_crc'", file_path);
            return CrcErrorCode::FILE_READ_ERROR;
        }

        result = std::stoul(json_data["middle_lidar_crc"].get<std::string>(),
                                nullptr, 16);
        LogTrace("expected crc32 in json file: 0x{:08x}", result);
    } catch (const json::parse_error& e) {
        LogError("Failed to parse JSON file: {}", file_path);
        return CrcErrorCode::FILE_READ_ERROR;
    } catch (const json::type_error& e) {
        LogError("JSON file 'middle_lidar_crc' field is not uint32: {}", file_path);
        return CrcErrorCode::FILE_READ_ERROR;
    } catch (const std::exception& e) {
        LogError("Unknown error: {}", e.what());
        return CrcErrorCode::FILE_READ_ERROR;
    }

    return CrcErrorCode::SUCCESS;
}

bool verify_crc32(const std::string& bin_file_path,
                const std::string& json_file_path) {
    uint32_t calculate_crc32 {0};
    uint32_t expected_crc32 {0};

    if ((CrcErrorCode::SUCCESS != calculate_bin_file_crc32(bin_file_path, calculate_crc32)) ||
        (CrcErrorCode::SUCCESS != read_json_file_crc32(json_file_path, expected_crc32))) {
        return false;
    }

    return (calculate_crc32 == expected_crc32);
}
/******************************************************************************/
/*          Definition of public functions of classes or templates             */
/******************************************************************************/

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/

} // namespace robosense::lidar::crc32

/* \}  utils */