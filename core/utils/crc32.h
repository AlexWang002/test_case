/*******************************************************************************
 * \addtogroup CRC32
 * \{
 * \headerfile crc32.h "crc32.h"
 * \brief Defines the CRC32 calculate functions.
 * \version 1.1
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-06-05 | Init version |
 * | 1.0 | 2025-07-07 | Refactor file format, fix static warning |
 * | 1.1 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/
#ifndef I_CRC32_H
#define I_CRC32_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <iostream>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#if defined(__arm__) || defined(__aarch64__)
#include <arm_acle.h> // ARM 内置函数头文件
#endif

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::crc32 {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
/**
 * \enum CrcErrorCode
 * \brief Error code of CRC32 calculation.
 * \var CrcErrorCode::SUCCESS
 * \var CrcErrorCode::FILE_NOT_EXIST
 * \var CrcErrorCode::FILE_NOT_READABLE
 * \var CrcErrorCode::FILE_NOT_BIN
 * \var CrcErrorCode::FILE_NOT_JSON
 * \var CrcErrorCode::FILE_OPEN_ERROR
 * \var CrcErrorCode::FILE_READ_ERROR
 */
enum class CrcErrorCode : uint8_t {
    SUCCESS = 0,
    FILE_NOT_EXIST = 1,
    FILE_NOT_READABLE = 2,
    FILE_NOT_BIN = 3,
    FILE_NOT_JSON = 4,
    FILE_OPEN_ERROR = 4,
    FILE_READ_ERROR = 5,
};

/******************************************************************************/
/*           Declaration and definition of exported constant data             */
/******************************************************************************/
static constexpr uint32_t kEndValue{0xFFFFFFFFU};
static constexpr uint32_t kStartValue{0xFFFFFFFFU};
static constexpr uint32_t kPolynomial{0xEDB88320U};   // reverse: 0x04C11DB7

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
#if defined(__arm__) || defined(__aarch64__)

/**
 * \brief Calculate CRC32 value of a data block using ARM CRC32 instruction set.
 * \param[in] in_buf The pointer to the data block.
 * \param[in] length The length of the data block.
 * \return The CRC32 value.
 */
inline uint32_t calculate_crc32(const void *const in_buf,
                                const size_t& length) noexcept {
    if (nullptr == in_buf || 0 == length) {
        return kEndValue;
    }

    const uint8_t *data = static_cast<const uint8_t *>(in_buf);
    const uint8_t *end = data + length;
    uint32_t crc = kStartValue;
    uint64_t u64_val;
    uint32_t u32_val;
    uint16_t u16_val;
    // 处理8字节数据
    while (data + 8 <= end) {
        memcpy(&u64_val, data, sizeof(uint64_t)); // 确保安全读取
        crc = __crc32d(crc, u64_val);             // 使用 __crc32d
        data += 8;
    }
    // 处理4字节数据
    while (data + 4 <= end) {
        memcpy(&u32_val, data, sizeof(uint32_t));
        crc = __crc32w(crc, u32_val); // 使用 __crc32w
        data += 4;
    }
    // 处理2字节数据
    while (data + 2 <= end) {
        memcpy(&u16_val, data, sizeof(uint16_t));
        crc = __crc32h(crc, u16_val); // 使用 __crc32h
        data += 2;
    }
    // 处理1字节数据
    while (data < end) {
        crc = __crc32b(crc, *data); // 使用 __crc32b
        ++data;
    }

    return (crc ^ kEndValue);
}

#else

/**
 * \brief Calculate CRC32 value of a single byte.
 * \param[in] f_data_r The byte to be calculated.
 * \return The CRC32 value of the byte.
 */
inline uint32_t calculate_byte(const int32_t& f_data_r) noexcept {
    uint32_t ulCRC = f_data_r & 0xFFU;

    for (int8_t i = 0; i < 8; ++i) {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ kPolynomial;
        else
            ulCRC >>= 1;
    }

    return ulCRC;
}

/**
 * \brief Calculate CRC32 value of a data block.
 * \param[in] in_buf The pointer to the data block.
 * \param[in] length The length of the data block.
 * \return The CRC32 value.
 */
inline uint32_t calculate_crc32(const void *const in_buf,
                                const size_t& length) noexcept {
    if (nullptr == in_buf || 0 == length) {
        return kEndValue;
    }

    // 初始化 CRC32 为 0xFFFFFFFF
    uint32_t crc = kStartValue; // 标准 CRC32 初始化值
    const uint8_t *data = static_cast<const uint8_t *>(in_buf);
    const uint8_t *end = data + length;

    while (data < end) {
        uint32_t ulCrcDark = (crc >> 8) & 0x00FFFFFFU;
        uint32_t ulCrcWhite = calculate_byte(
                            static_cast<int32_t>((crc ^ (*data)) & 0xFFU));
        crc = ulCrcDark ^ ulCrcWhite;
        data++;
    }

    // 最终 CRC32 为 crc ^ 0xFFFFFFFF
    crc = crc ^ kEndValue; // 标准 CRC32 最终异或
    return crc;
}

#endif

/**
 * \brief Read CRC32 value from a JSON file.
 * \param[in] file_path The path of the JSON file.
 * \param[out] result The CRC32 value.
 * \return The error code.
 */
CrcErrorCode read_json_file_crc32(const std::string& file_path,
                                    uint32_t& result);

/**
 * \brief Calculate CRC32 value of a binary file.
 * \param[in] file_path The path of the binary file.
 * \param[out] result The CRC32 value.
 * \return The error code.
 */
CrcErrorCode calculate_bin_file_crc32(const std::string& file_path,
                                    uint32_t& result);

/**
 * \brief Calculate CRC32 value of a string.
 * \param[in] str_data The string.
 * \return The CRC32 value.
 */
inline uint32_t calculate_crc32(const std::string& str_data) noexcept {
    return calculate_crc32(str_data.data(), str_data.size());
}

/**
 * \brief Calculate CRC32 value of a vector of int32_t.
 * \param[in] vector The vector.
 * \return The CRC32 value.
 */
inline uint32_t calculate_crc32(const std::vector<int32_t>& vector) noexcept {
    const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(vector.data());
    return calculate_crc32(byte_ptr, vector.size() * sizeof(int32_t));
}

/**
 * \brief Calculate CRC32 value of a vector of uint32_t.
 * \param[in] vector The vector.
 * \return The CRC32 value.
 */
inline uint32_t calculate_crc32(const std::vector<uint32_t>& vector) noexcept {
    const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(vector.data());
    return calculate_crc32(byte_ptr, vector.size() * sizeof(uint32_t));
}

/**
 * \brief Calculate CRC32 value of a vector of uint8_t.
 * \param[in] vector The vector.
 * \return The CRC32 value.
 */
inline uint32_t calculate_crc32(const std::vector<uint8_t>& vector) noexcept {
    const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(vector.data());
    return calculate_crc32(byte_ptr, vector.size());
}

/**
 * \brief Verify CRC32 value of a binary file.
 * \param[in] bin_file_path The path of the binary file.
 * \param[in] json_file_path The path of the JSON file.
 * \return The error code.
 */
bool verify_crc32(const std::string& bin_file_path,
                const std::string& json_file_path);

/**
 * \brief Verify CRC32 value of a data block.
 * \param[in] in_buf The pointer to the data block.
 * \param[in] length The length of the data block.
 * \param[in] crc32 The CRC32 value.
 * \return The error code.
 */
inline bool verify_crc32(const void *const in_buf, const size_t& length,
                        const uint32_t& crc32) noexcept {
    return (crc32 == calculate_crc32(in_buf, length));
}

/**
 * \brief Verify CRC32 value of a vector of uint8_t.
 * \param[in] vector The vector.
 * \param[in] crc32 The CRC32 value.
 * \return The error code.
 */
inline bool verify_crc32(const std::vector<uint8_t>& vector,
                        const uint32_t& crc32) noexcept {
    return (crc32 == calculate_crc32(vector));
}

} // namespace robosense::lidar::crc32

/** \} CRC32 */
#endif /* I_CRC32_H */