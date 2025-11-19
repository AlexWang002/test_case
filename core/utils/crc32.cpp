/*******************************************************************************
 * \addtogroup utils
 * \{
 * \file crc32.cpp
 * \brief CRC32 calculator
 * \version 0.2
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-07-13 | Init version, add bin file reader and crc32 calculator |
 * | 0.2 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "crc32.h"
#include "json.hpp"
#include "rs_new_logger.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::crc32 {

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using json = nlohmann::json;

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Check if a file exists
 * \param[in] file_path The file path
 * \retval true The file exists
 * \retval false The file does not exist
 */
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
/**
 * \brief Calculate the CRC32 of a bin file
 * \param[in] file_path The file path
 * \param[out] result The CRC32 result
 * \retval CrcErrorCode::SUCCESS The calculation is successful
 * \retval CrcErrorCode::FILE_NOT_BIN The file path is not end with .bin
 * \retval CrcErrorCode::FILE_NOT_EXIST The file does not exist
 * \retval CrcErrorCode::FILE_OPEN_ERROR The file open error
 * \retval CrcErrorCode::FILE_READ_ERROR The file read error
 */
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
    (void)file.seekg(0, std::ios::end);
    std::streamsize file_size = file.tellg();
    (void)file.seekg(0, std::ios::beg);

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

/**
 * \brief Read the CRC32 from a json file
 * \param[in] file_path The file path
 * \param[out] result The CRC32 result
 * \retval CrcErrorCode::SUCCESS The calculation is successful
 * \retval CrcErrorCode::FILE_NOT_JSON The file path is not end with .json
 * \retval CrcErrorCode::FILE_NOT_EXIST The file does not exist
 * \retval CrcErrorCode::FILE_OPEN_ERROR The file open error
 * \retval CrcErrorCode::FILE_READ_ERROR The file read error
 */
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

        result = std::stoul(json_data["middle_lidar_crc"].get<std::string>(), nullptr, 16);
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

/**
 * \brief Verify the CRC32 of a bin file
 * \param[in] bin_file_path The bin file path
 * \param[in] json_file_path The json file path
 * \retval true The CRC32 is verified
 * \retval false The CRC32 is not verified
 */
bool verify_crc32(const std::string& bin_file_path, const std::string& json_file_path) {
    uint32_t calculate_crc32 {0U};
    uint32_t expected_crc32 {0U};
    bool result {false};

    if ((CrcErrorCode::SUCCESS != calculate_bin_file_crc32(bin_file_path, calculate_crc32)) ||
        (CrcErrorCode::SUCCESS != read_json_file_crc32(json_file_path, expected_crc32))) {
        return false;
    }

    if (calculate_crc32 == expected_crc32) {
        LogInfo("Bin file verify crc32 success");
        result = true;
    } else {
        LogError("Bin file verify crc32 failed");
        LogError("Calculate crc32: 0x{:08x}, Expected crc32: 0x{:08x}", calculate_crc32, expected_crc32);
    }

    return result;
}

} // namespace robosense::lidar::crc32

/* \}  utils */