/*******************************************************************************
 * \addtogroup utils
 * \{
 * \headerfile json_reader.h "json_reader.h"
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
#ifndef I_JSON_READER_H
#define I_JSON_READER_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <json.hpp>
#include <mpark/variant.hpp>
#include <vector>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "thread_config.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::json_reader {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
/**
 * \struct AlgoSwitch
 * \brief This struct defines the algorithm switch.
 */
struct AlgoSwitch {
    bool enable_trail_remove {false};
    bool enable_denoise      {false};
    bool enable_stray        {false};
    bool enable_spray        {false};
    bool enable_delete       {false};
    bool data_valid          {false};
};

/**
 * \struct TestParam
 * \brief This struct defines the test param.
 */
struct TestParam {
    bool enable_save_pcd        {false};
    bool enable_cpu_monitor     {false};
    bool enable_print_difop2    {false};
    bool enable_mipi_crc        {false};
    bool enable_difop2_crc      {false};
    uint32_t print_difop2_cycle {1000U};
    uint32_t cpu_monitor_cycle  {1000U};
    uint32_t difop2_crc_cycle   {1000U};
    double cpu_threshold        {500.0};
    std::string save_path       {"./"};
    bool data_valid             {false};
};

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/
using DataVariant = mpark::variant<bool, std::string, std::vector<thread::ThreadConfig>, AlgoSwitch, TestParam>;

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern AlgoSwitch algo_switch;
extern TestParam test_param;

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
template <class T>
T getJsonData(const nlohmann::json& json, const std::string& key) {
    try {
        if (json.contains(key)) {
            return json[key].get<T>();
        } else {
            throw std::runtime_error("JSON key '" + key + "' not found");
        }
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("JSON parsing error for key '" + key + "': " + e.what());
    } catch (...) {
        throw std::runtime_error("Unknown error when get JSON data for key '" + key + "'");
    }
}

template <>
std::vector<thread::ThreadConfig>
getJsonData<std::vector<thread::ThreadConfig>>(const nlohmann::json& json, const std::string& key);

template <>
AlgoSwitch getJsonData<AlgoSwitch>(const nlohmann::json& json, const std::string& key);

template <>
TestParam getJsonData<TestParam>(const nlohmann::json& json, const std::string& key);

class DataVariantVisitor {
  public:
    std::string operator()(const bool& value) const;
    std::string operator()(const std::string& value) const;
    std::string operator()(const std::vector<thread::ThreadConfig>& value) const;
    std::string operator()(const AlgoSwitch& value) const;
    std::string operator()(const TestParam& value) const;
};

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
bool readJsonFile(const std::string& file, nlohmann::json& json_obj);

std::vector<DataVariant> parseJsonFile(const std::string& file);

} // namespace robosense::lidar::json_reader

/** \} utils */
#endif /* I_JSON_READER_H */
