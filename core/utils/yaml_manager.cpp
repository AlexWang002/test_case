/*******************************************************************************
 * \addtogroup YmlMng
 * \{
 * \file yaml_manager.cpp
 * \brief
 * \version 0.2
 * \date 2025-06-26
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-05-22 | Init version |
 * | 0.2 | 2025-06-26 | Add algorithm switch yaml file parse |
 *
 ******************************************************************************/

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cstring>
#include <unistd.h>
#include <iomanip>
#include "rs_new_logger.h"
#include "crc32.h"

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "yaml_manager.h"

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

namespace robosense::lidar::yaml {

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/

/******************************************************************************/
/*            Declaration of exported variables or constant data              */
/******************************************************************************/
Difop2 difop2_inner_param;
AlgoSwitch algo_switch_param;
TestParam demo_test_param;
std::vector<ThreadConfig> thread_params;

/******************************************************************************/
/*                       Definition of local variables                        */
/******************************************************************************/

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
inline uint16_t swapEndianU16(uint16_t value) noexcept {
    return (((value >> 8) & 0xFFU) | ((value & 0xFFU) << 8));
}
/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
bool readAlgoYaml(const std::string& path) {
    AlgoYaml algo_yaml_obj(path);
    ErrorCode result = algo_yaml_obj.parseYamlFile();
    if (ErrorCode::SUCCESS != result) {
        return false;
    }

    return true;
}

/******************************************************************************/
/*          Definition of public functions of classes or templates             */
/******************************************************************************/
ErrorCode DifopYaml::parseYamlFile() {
    ErrorCode result;

    try {
        result = parseDifop2Data();

        if (ErrorCode::SUCCESS != result) {
            return result;
        }
        result = checkYamlData();

        if (ErrorCode::SUCCESS == result) {
            difop2_inner_param = difop2_param_;
        }
    }  catch (const YAML::BadFile& e) {
        LogError("Error opening file: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (const YAML::ParserException& e) {
        LogError("YAML parsing error: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (const std::exception& e) {
        LogError("Standard error: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (...) {
        LogError("Unknown error occurred");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }

    return result;
}

void DifopYaml::getYamlConfigParam(Difop2& difop_param) {
    difop_param = difop2_param_;
}

ErrorCode AlgoYaml::parseYamlFile() {
    ErrorCode result;

    try {
        result = parseAlgoSwitchData();

        if (ErrorCode::SUCCESS == result) {
            algo_switch_param = algo_param_;
            algo_switch_param.data_valid = true;
        } else {
            algo_switch_param.data_valid = false;
        }
        result = parseTestParam();

        if (ErrorCode::SUCCESS == result) {
            demo_test_param = test_param_;
            demo_test_param.data_valid = true;
        } else {
            demo_test_param.data_valid = false;
        }
    }  catch (const YAML::BadFile& e) {
        LogError("Error opening file: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (const YAML::ParserException& e) {
        LogError("YAML parsing error: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (const std::exception& e) {
        LogError("Standard error: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (...) {
        LogError("Unknown error occurred");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }

    return result;
}

void AlgoYaml::getYamlConfigParam(AlgoSwitch& algo_switch_param) {
    algo_switch_param = algo_param_;
}

//Assign thread params in a class to global variables
void ThreadConfigYaml::getYamlConfigParam(std::vector<ThreadConfig>& thread_params) {
    thread_params = thread_params_;
}

// parse thread yaml file to thread_params
ErrorCode ThreadConfigYaml::parseYamlFile() {
    ErrorCode result;
    try {
        result = parseThreadConfig();
        if (ErrorCode::SUCCESS == result) {
            thread_params = thread_params_;
        }
    }  catch (const YAML::BadFile& e) {
        LogError("Error opening file: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (const YAML::ParserException& e) {
        LogError("YAML parsing error: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (const std::exception& e) {
        LogError("Standard error: {}", e.what());
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    } catch (...) {
        LogError("Unknown error occurred");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }

    return result;
}


/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/
ErrorCode DifopYaml::parseDifop2Data() {
    std::string file {input_yaml_path_};

    if((file.size() < 6) || (0 != file.compare(file.size() - 4, 4, "yaml"))) {
        file += "middle_lidar_inner_para.yaml";
        LogInfo("Yaml file path: {}", file);
    }
    if (0 != access(file.c_str(), F_OK)) {
        LogError("custom yaml file is not existed! input_base_path_ = {}", input_yaml_path_.c_str());
        return ErrorCode::YAML_FILE_NOT_EXIST;
    }
    YAML::Node node = YAML::LoadFile(file);

    if ((!node.IsMap()) || (!node["difop_2"]) || (!node["CRC32"])) {
        LogError("feild 'difop_2' or 'CRC32' NOT in yaml file.");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }
    difop2_param_ = node["difop_2"].as<Difop2>();
    crc32_ = node["CRC32"].as<uint32_t>();

    return ErrorCode::SUCCESS;
}

void DifopYaml::swapDifopDataBigEndian(Difop2& difop) {
    uint16_t temp_pitch_0 {difop.surface_pitch_offset[0]};
    uint16_t temp_pitch_1 {difop.surface_pitch_offset[1]};
    uint16_t temp_roll {difop.roll_offset};
    uint16_t temp_length {difop.data_length};
    uint16_t temp_counter {difop.counter};

    difop.surface_pitch_offset[0] = swapEndianU16(temp_pitch_0);
    difop.surface_pitch_offset[1] = swapEndianU16(temp_pitch_1);
    difop.roll_offset = swapEndianU16(temp_roll);
    difop.data_length = swapEndianU16(temp_length);
    difop.counter = swapEndianU16(temp_counter);
}

ErrorCode DifopYaml::checkYamlData() {
    size_t total_size = sizeof(Difop2) - 2 * sizeof(bool);
    std::vector<uint8_t> all_bytes(total_size);
    Difop2 temp_difop {difop2_param_};

    swapDifopDataBigEndian(temp_difop);
    std::memcpy(all_bytes.data(), &temp_difop.info_header[0], total_size);

    if (!crc32::verify_crc32(all_bytes, crc32_)) {
        LogError("CRC32 value verify fault!");
        difop2_param_.data_valid = false;
        return ErrorCode::CRC32_VERIFY_FAULT;
    }
    difop2_param_.data_valid = true;

    return ErrorCode::SUCCESS;
}

ErrorCode AlgoYaml::parseAlgoSwitchData() {
    std::string file {input_yaml_path_};

    if((file.size() < 6) || (0 != file.compare(file.size() - 4, 4, "yaml"))) {
        file += "algorithm_switch.yaml";
        LogInfo("Yaml file path: {}", file);
    }
    if (0 != access(file.c_str(), F_OK)) {
        LogError("custom yaml file is not existed! input_base_path_ = {}", input_yaml_path_.c_str());
        return ErrorCode::YAML_FILE_NOT_EXIST;
    }
    YAML::Node node = YAML::LoadFile(file);

    if ((!node.IsMap()) || (!node["Algorithm_Switch"])) {
        LogError("Feild 'Algorithm_Switch' NOT in yaml file.");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }

    algo_param_ = node["Algorithm_Switch"].as<AlgoSwitch>();

    return ErrorCode::SUCCESS;
}

ErrorCode AlgoYaml::parseTestParam() {
    std::string file {input_yaml_path_};

    if((file.size() < 6) || (0 != file.compare(file.size() - 4, 4, "yaml"))) {
        file += "algorithm_switch.yaml";
        LogInfo("Yaml file path: {}", file);
    }
    if (0 != access(file.c_str(), F_OK)) {
        LogError("custom yaml file is not existed! input_base_path_ = {}", input_yaml_path_.c_str());
        return ErrorCode::YAML_FILE_NOT_EXIST;
    }
    YAML::Node node = YAML::LoadFile(file);

    if ((!node.IsMap()) || (!node["Test_Param"])) {
        LogError("Feild 'Test_Param' NOT in yaml file.");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }

    test_param_ = node["Test_Param"].as<TestParam>();

    return ErrorCode::SUCCESS;
}

// parse thread yaml file to thread_params
ErrorCode ThreadConfigYaml::parseThreadConfig() {
    std::string file {input_yaml_path_};

    if((file.size() < 6) || (0 != file.compare(file.size() - 4, 4, "yaml"))) {
        file += "thread_params.yaml";
        LogInfo("Yaml file path: {}", file);
    }
    if (0 != access(file.c_str(), F_OK)) {
        LogError("custom yaml file is not existed! input_base_path_ = {}", input_yaml_path_.c_str());
        return ErrorCode::YAML_FILE_NOT_EXIST;
    }
    YAML::Node node = YAML::LoadFile(file);

    if ((!node.IsMap()) || (!node["threads"])) {
        LogError("Field 'threads' NOT in yaml file.");
        return ErrorCode::PARSE_YAML_NODE_FAILED;
    }

    thread_params_ = node.as<std::vector<robosense::lidar::yaml::ThreadConfig>>();

    return ErrorCode::SUCCESS;
}

} // namespace robosense::lidar


namespace YAML {

/// @brief 将十六进制字符串转换为 uint8_t 数组
/// @param hex_str
/// @return
std::vector<uint8_t> hexStringToByteArray(const std::string& hex_str) {
    std::string processed_str = hex_str;
    // Remove the space in the hex-string.
    processed_str.erase(std::remove_if(processed_str.begin(),
                        processed_str.end(), ::isspace), processed_str.end());

    if (processed_str.length() % 2 != 0) {
        LogError("Hex string must have an even length.");
        throw std::invalid_argument("Hex string must have an even length.");
    }

    std::vector<uint8_t> byteArray;
    size_t start_index = (0 == processed_str.compare(0, 2, "0x")) ? 2 : 0;

    // Cast 2 characters to uint8_t, except "0x"
    for (size_t i {start_index}; i < processed_str.length(); i += 2) {
        std::string byteStr = processed_str.substr(i, 2);
        uint8_t temp = static_cast<uint8_t>(
                        std::stoul(byteStr, nullptr, 16) & 0xFF);
        byteArray.push_back(temp);
    }

    return byteArray;
}

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
template<>
struct convert<robosense::lidar::yaml::Difop2> {
    static bool decode(const Node& node, robosense::lidar::yaml::Difop2& rhs) {
        if ((!node.IsMap()) || (!node["InfoHdr"]) || (!node["Res0"]) ||
            (!node["SurfaceCnt"]) || (!node["VcselPixelCnt"]) ||
            (!node["VcselCnt"]) || (!node["VcselYawOffset"]) ||
            (!node["PixelPitch"]) || (!node["PitchOffset"]) ||
            (!node["RollOffset"]) || (!node["Res1"]) || (!node["DataLength"]) ||
            (!node["Counter"]) || (!node["DataID"])) {
            return false;
        }
        try {   // decode the data of InfoHdr
            std::string hex_str = node["InfoHdr"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.info_header.size()) {
                throw std::length_error("Infor header vector size does not match array size.");
                return false;
            }
            std::copy(hex_vector.begin(), hex_vector.end(), rhs.info_header.begin());
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of Res0
            std::string hex_str = node["Res0"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.reserve_data_0.size()) {
                throw std::length_error("Reserve data 0 vector size does not match array size.");
                return false;
            }
            std::copy(hex_vector.begin(), hex_vector.end(), rhs.reserve_data_0.begin());
        } catch (const std::exception& e) {
            return false;
        }
        try {   // decode the data of SurfaceCnt
            rhs.surface_count = node["SurfaceCnt"].as<uint8_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of VcselPixelCnt
            rhs.vcsel_pixel_count = node["VcselPixelCnt"].as<uint8_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of VcselCnt
            rhs.vcsel_count = node["VcselCnt"].as<uint8_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of VcselYawOffset
            std::string hex_str = node["VcselYawOffset"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.vcsel_yaw_offset.size()) {
                throw std::length_error("Vcsel yaw offset vector size does not match array size.");
                return false;
            }
            std::copy(hex_vector.begin(), hex_vector.end(), rhs.vcsel_yaw_offset.begin());
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of PixelPitch
            std::string hex_str = node["PixelPitch"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.pixel_pitch.size()) {
                throw std::length_error("Pixel pitch vector size does not match array size.");
                return false;
            }
            std::copy(hex_vector.begin(), hex_vector.end(), rhs.pixel_pitch.begin());
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of PitchOffset
            std::string hex_str = node["PitchOffset"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.surface_pitch_offset.size() * sizeof(rhs.surface_pitch_offset[0])) {
                throw std::length_error("Pitch offset vector size does not match array size.");
                return false;
            }

            // 拷贝数据
            for (size_t i {0}; i < rhs.surface_pitch_offset.size(); ++i) {
                // 将每两个字节组合成一个 uint16_t 元素
                rhs.surface_pitch_offset[i] = (static_cast<uint16_t>(hex_vector[i * 2]) << 8) | hex_vector[i * 2 + 1];
            }
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }

        try {   // decode the data of RollOffset
            rhs.roll_offset = node["RollOffset"].as<uint16_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of Res1
            std::string hex_str = node["Res1"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.reserve_data_1.size()) {
                throw std::length_error("Reserve data 1 vector size does not match array size.");
                return false;
            }
            std::copy(hex_vector.begin(), hex_vector.end(), rhs.reserve_data_1.begin());
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of DataLength
            rhs.data_length = node["DataLength"].as<uint16_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of Counter
            rhs.counter = node["Counter"].as<uint16_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of DataID
            std::string hex_str = node["DataID"].as<std::string>();
            std::vector<uint8_t> hex_vector = hexStringToByteArray(hex_str);

            if (hex_vector.size() != rhs.data_id.size()) {
                throw std::length_error("Data ID vector size does not match array size.");
                return false;
            }
            std::copy(hex_vector.begin(), hex_vector.end(), rhs.data_id.begin());
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }

        return true;
    }
};

template<>
struct convert<robosense::lidar::yaml::AlgoSwitch> {
    static bool decode(const Node& node, robosense::lidar::yaml::AlgoSwitch& rhs) {
        if ((!node.IsMap()) ||
            (!node["enable_trail_remove"]) ||
            (!node["enable_denoise"]) ||
            (!node["enable_high_ref_cross"]) ||
            (!node["enable_filter"]) ||
            (!node["enable_hor_dist_smooth"]) ||
            (!node["enable_hor_ref_smooth"]) ||
            (!node["enable_ver_dist_smooth"]) ||
            (!node["enable_ver_ref_smooth"])) {
            return false;
        }
        try {   // decode the data of enable_trail_remove
            rhs.enable_trail_remove = node["enable_trail_remove"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_denoise
            rhs.enable_denoise = node["enable_denoise"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_high_ref_cross
            rhs.enable_high_ref_cross = node["enable_high_ref_cross"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_filter
            rhs.enable_filter = node["enable_filter"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_hor_dist_smooth
            rhs.enable_hor_dist_smooth = node["enable_hor_dist_smooth"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_hor_ref_smooth
            rhs.enable_hor_ref_smooth = node["enable_hor_ref_smooth"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_ver_dist_smooth
            rhs.enable_ver_dist_smooth = node["enable_ver_dist_smooth"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_ver_ref_smooth
            rhs.enable_ver_ref_smooth = node["enable_ver_ref_smooth"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        rhs.data_valid = true;

        return true;
    }
};

template<>
struct convert<robosense::lidar::yaml::TestParam> {
    static bool decode(const Node& node, robosense::lidar::yaml::TestParam& rhs) {
        if ((!node.IsMap()) ||
            (!node["enable_save_pcd"]) ||
            (!node["save_path"])) {
            return false;
        }
        try {   // decode the data of enable_save_pcd
            rhs.enable_save_pcd = node["enable_save_pcd"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of save_path
            rhs.save_path = node["save_path"].as<std::string>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_cpu_monitor
            rhs.enable_cpu_monitor = node["enable_cpu_monitor"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_print_difop2
            rhs.enable_print_difop2 = node["enable_print_difop2"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_mipi_crc_check
            rhs.enable_mipi_crc_check = node["enable_mipi_crc_check"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of enable_difop2_crc_check
            rhs.enable_difop2_crc_check = node["enable_difop2_crc_check"].as<bool>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of print_difop2_cycle
            rhs.print_difop2_cycle = node["print_difop2_cycle"].as<uint32_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of cpu_monitor_cycle
            rhs.cpu_monitor_cycle = node["cpu_monitor_cycle"].as<uint32_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of difop2_crc_check_cycle
            rhs.difop2_crc_check_cycle = node["difop2_crc_check_cycle"].as<uint32_t>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        try {   // decode the data of cpu_threshold
            rhs.cpu_threshold = node["cpu_threshold"].as<double>();
        } catch (const std::exception& e) {
            LogError(e.what());
            return false;
        }
        rhs.data_valid = true;

        return true;
    }
};

template<>
struct convert<std::vector<robosense::lidar::yaml::ThreadConfig>> {
    static bool decode(const Node& node, std::vector<robosense::lidar::yaml::ThreadConfig>& rhs) {
        using robosense::lidar::yaml::ThreadConfig;

        if (!node.IsMap() || !node["threads"] || !node["threads"].IsSequence()) {
            std::cerr << "YAML format error: 'threads' field missing or not a sequence." << std::endl;
            return false;
        }

        try {
            for (const auto& item : node["threads"]) {
                ThreadConfig cfg;

                if (!item["id"] || !item["policy"] || !item["priority"] || !item["cpu_affinity"]) {
                    std::cerr << "Incomplete thread config detected." << std::endl;
                    return false;
                }

                cfg.id = item["id"].as<uint16_t>();
                cfg.policy = item["policy"].as<std::string>();
                cfg.priority = item["priority"].as<int>();
                cfg.cpu_affinity = item["cpu_affinity"].as<std::vector<int>>();

                rhs.emplace_back(std::move(cfg));
            }
        } catch (const std::exception& e) {
            std::cerr << "YAML decode error: " << e.what() << std::endl;
            return false;
        }

        return true;
    }
};


} // namespace robosense::lidar::yaml

/* \}  YmlMng */