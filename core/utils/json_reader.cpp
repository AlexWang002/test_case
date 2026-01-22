/*******************************************************************************
 * \addtogroup utils
 * \{
 * \file json_reader.cpp
 * \brief
 * \version 0.1
 * \date 2025-09-26
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-09-26 | Init version |
 *
 ******************************************************************************/

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <fstream>
#include <iostream>
#include <unistd.h>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "json_reader.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::json_reader {

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using nlohmann::json;

/******************************************************************************/
/*                   Definition of exported constant data                     */
/******************************************************************************/

/******************************************************************************/
/*                     Definition of local constant data                      */
/******************************************************************************/

/******************************************************************************/
/*              Definition of local types (enum, struct, union)               */
/******************************************************************************/

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
AlgoSwitch algo_switch;
TestParam test_param;

/******************************************************************************/
/*                 Declaration or Definition of local variables               */
/******************************************************************************/

/******************************************************************************/
/*                       Definition of local functions                        */
/******************************************************************************/
/**
 * \brief Check if a file exists
 * \param[in] file_path The file path
 * \retval true The file exists
 * \retval false The file does not exist
 */
bool checkFileExist(const std::string& file_path) {
    if (0 != access(file_path.c_str(), F_OK)) {
        std::cout << "File " << file_path << " not exist" << std::endl;
        return false;
    }
    if (0 != access(file_path.c_str(), R_OK)) {
        std::cout << "File " << file_path << " exist but not readable" << std::endl;
        return false;
    }

    return true;
}

inline std::string boolToString(const bool& value) {
    return value ? "true" : "false";
}

/******************************************************************************/
/*                      Definition of exported functions                      */
/******************************************************************************/
bool readJsonFile(const std::string& file, json& json_obj) {
    if (!checkFileExist(file)) {
        std::cout << "Read JSON file " << file << " failed" << std::endl;
        return false;
    }
    std::ifstream file_stream(file);
    file_stream >> json_obj;
    return true;
}

std::vector<DataVariant> parseJsonFile(const std::string& file) {
    std::vector<DataVariant> result;
    json json_obj;

    if (!readJsonFile(file, json_obj)) {
        std::cout << "Parse JSON file " << file << " failed" << std::endl;
        return result;
    }

    try {
        if (!json_obj.contains("RobosenseLidar")) {
            throw std::runtime_error("JSON key 'RobosenseLidar' not found");
            return result;
        }
        json target = json_obj["RobosenseLidar"];
        DataVariant mipi_crc = getJsonData<bool>(target, "ENABLE_MIPI_CRC");
        DataVariant param_path = getJsonData<std::string>(target, "INNER_PARAM_PATH");
        DataVariant log_path = getJsonData<std::string>(target, "LOG_CONFIG_FILE");
        DataVariant fault_save = getJsonData<bool>(target, "PARSE_INNER_PARAM_BIN");
        DataVariant fault_path = getJsonData<std::string>(target, "LOG_CONFIG_FILE");
        DataVariant parse_inner_param_bin = getJsonData<bool>(target, "PARSE_INNER_PARAM_BIN");
        DataVariant delay_stat = getJsonData<bool>(target, "DELAY_STAT");
        DataVariant vpu_auth = getJsonData<bool>(target, "VPU_AUTH");
        DataVariant threads = getJsonData<std::vector<thread::ThreadConfig>>(target, "THREADS");
        DataVariant algo_param = getJsonData<AlgoSwitch>(target, "ALGORITHM");
        algo_switch = getJsonData<AlgoSwitch>(target, "ALGORITHM");
        result.push_back(mipi_crc);
        result.push_back(param_path);
        result.push_back(log_path);
        result.push_back(delay_stat);
        result.push_back(vpu_auth);
        result.push_back(parse_inner_param_bin);
        result.push_back(threads);
        result.push_back(algo_param);
        result.push_back(fault_save);
        result.push_back(fault_path);

        if (target.contains("TEST_PARAM")) {
            DataVariant param = getJsonData<TestParam>(target, "TEST_PARAM");
            test_param = getJsonData<TestParam>(target, "TEST_PARAM");
            result.push_back(param);
        }
    } catch (const json::exception& e) {
        throw std::runtime_error(std::string("JSON parsing error: ") + e.what());
    } catch (const std::runtime_error& e) {
        std::cout << "JSON parsing middle error: " << e.what() << std::endl;
        throw;
    } catch (...) {
        throw std::runtime_error("Unknown error when parse JSON file");
    }

    return result;
}

/******************************************************************************/
/*          Definition of public functions of classes or templates            */
/******************************************************************************/
template <>
std::vector<thread::ThreadConfig>
getJsonData<std::vector<thread::ThreadConfig>>(const json& json_obj, const std::string& key) {
    std::vector<thread::ThreadConfig> configs;

    try {
        if (!json_obj.contains(key)) {
            throw std::runtime_error("JSON key '" + key + "' not found");
            return configs;
        }
        json target = json_obj[key];

        for (json::const_iterator it = target.begin(); it != target.end(); ++it) {
            const json& item = *it;
            thread::ThreadConfig config;

            config.id = getJsonData<uint16_t>(item, "id");
            config.policy = getJsonData<std::string>(item, "policy");
            config.priority = getJsonData<int>(item, "priority");
            config.cpu_affinity = getJsonData<std::vector<int>>(item, "cpu_affinity");
            configs.push_back(config);
        }
    } catch (const json::exception& e) {
        throw std::runtime_error("JSON parsing error for key '" + key + "': " + e.what());
    } catch (...) {
        throw std::runtime_error("Unknown error when get JSON data for key '" + key + "'");
    }

    return configs;
}

template <>
AlgoSwitch getJsonData<AlgoSwitch>(const json& json_obj, const std::string& key) {
    AlgoSwitch config;

    try {
        if (!json_obj.contains(key)) {
            throw std::runtime_error("JSON key '" + key + "' not found");
            return config;
        }
        json target = json_obj[key];
        config.enable_trail_remove = getJsonData<bool>(target, "ENABLE_TRAIL_REMOVE");
        config.enable_denoise = getJsonData<bool>(target, "ENABLE_DENOISE");
        config.enable_stray = getJsonData<bool>(target, "ENABLE_STRAY");
        config.enable_spray = getJsonData<bool>(target, "ENABLE_SPRAY");
        config.enable_delete = getJsonData<bool>(target, "ENABLE_SPRAY_DELETE");
        config.data_valid = true;
    } catch (const json::exception& e) {
        throw std::runtime_error("JSON parsing error for key '" + key + "': " + e.what());
    } catch (...) {
        throw std::runtime_error("Unknown error when get JSON data for key '" + key + "'");
    }

    return config;
}

template <>
TestParam getJsonData<TestParam>(const json& json_obj, const std::string& key) {
    TestParam config;

    try {
        if (!json_obj.contains(key)) {
            throw std::runtime_error("JSON key '" + key + "' not found");
            return config;
        }
        json target = json_obj[key];

        config.enable_save_pcd = getJsonData<bool>(target, "ENABLE_SAVE_PCD");
        config.enable_cpu_monitor = getJsonData<bool>(target, "ENABLE_CPU_MONITOR");
        config.enable_print_difop2 = getJsonData<bool>(target, "ENABLE_PRINT_DIFOP2");
        config.enable_mipi_crc = getJsonData<bool>(target, "ENABLE_MIPI_CRC_CHECK");
        config.enable_difop2_crc = getJsonData<bool>(target, "ENABLE_DIFOP2_CRC_CHECK");
        config.print_difop2_cycle = getJsonData<uint32_t>(target, "PRINT_DIFOP2_CYCLE");
        config.cpu_monitor_cycle = getJsonData<uint32_t>(target, "CPU_MONITOR_CYCLE");
        config.difop2_crc_cycle = getJsonData<uint32_t>(target, "DIFOP2_CRC_CHECK_CYCLE");
        config.cpu_threshold = getJsonData<double>(target, "CPU_THRESHOLD");
        config.save_path = getJsonData<std::string>(target, "SAVE_PATH");
        config.data_valid = true;
    } catch (const json::exception& e) {
        throw std::runtime_error("JSON parsing error for key '" + key + "': " + e.what());
    } catch (...) {
        throw std::runtime_error("Unknown error when get JSON data for key '" + key + "'");
    }

    return config;
}

std::string DataVariantVisitor::operator()(const bool& value) const {
    return "Bool value: " + boolToString(value);
}

std::string DataVariantVisitor::operator()(const std::string& value) const {
    return "String value: " + value;
}

std::string DataVariantVisitor::operator()(const std::vector<thread::ThreadConfig>& value) const {
    std::string result = "===== Thread Configs =====\n";

    for (const auto& thread : value) {
        result += "Thread ID      : " + std::to_string(thread.id) + "\n" + "  Policy       : " + thread.policy + "\n" +
                  "  Priority     : " + std::to_string(thread.priority) + "\n" + "  CPU Affinity : [";
        for (const auto& cpu : thread.cpu_affinity) {
            result += std::to_string(cpu) + ", ";
        }
        result += "]\n";
    }
    result += "===== Total thread configs count: " + std::to_string(value.size()) + " =====\n";

    return result;
}

std::string DataVariantVisitor::operator()(const AlgoSwitch& value) const {
    std::string result = "===== Algorithm Switch =====\n";
    result += "Enable trail remove : " + boolToString(value.enable_trail_remove) + "\n" +
              "Enable de-noise     : " + boolToString(value.enable_denoise) + "\n" +
              "Enable stray        : " + boolToString(value.enable_stray) + "\n" +
              "Enable spray        : " + boolToString(value.enable_spray) + "\n" +
              "Enable delete       : " + boolToString(value.enable_delete) + "\n";

    return result;
}

std::string DataVariantVisitor::operator()(const TestParam& value) const {
    std::string result = "===== Test Parameters =====\n";
    std::ostringstream oss;

    oss << std::fixed << std::setprecision(2) << value.cpu_threshold;
    result += "Enable save PCD     : " + boolToString(value.enable_save_pcd) + "\n";
    result += "Enable CPU monitor  : " + boolToString(value.enable_cpu_monitor) + "\n";
    result += "Enable print Difop2 : " + boolToString(value.enable_print_difop2) + "\n";
    result += "Enable MIPI CRC     : " + boolToString(value.enable_mipi_crc) + "\n";
    result += "Enable Difop2 CRC   : " + boolToString(value.enable_difop2_crc) + "\n";
    result += "Difop2 print cycle  : " + std::to_string(value.print_difop2_cycle) + "ms \n";
    result += "Monitor CPU cycle   : " + std::to_string(value.cpu_monitor_cycle) + "ms \n";
    result += "Difop2 CRC cycle    : " + std::to_string(value.difop2_crc_cycle) + "ms \n";
    result += "CPU usage threshold : " + oss.str() + "% \n";
    result += "PCD data save path  : " + value.save_path + "\n";

    return result;
}

/******************************************************************************/
/*         Definition of private functions of classes or templates            */
/******************************************************************************/

} // namespace robosense::lidar::json_reader

/* \}  utils */
