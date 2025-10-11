/*******************************************************************************
 * \addtogroup YmlMng
 * \{
 * \headerfile yaml_manager.h "yaml_manager.h"
 * \brief This file contains the yaml prase module common class.
 * \version 0.3
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-05-22 | Init version |
 * | 0.2 | 2025-06-26 | Add algorithm switch struct |
 * | 0.3 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/
#ifndef I_YAML_MANAGER_H
#define I_YAML_MANAGER_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cstdint>
#include <string>
#include <array>
#include <map>
#include <vector>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "yaml-cpp/yaml.h"
#include "difop2.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar::yaml {

/**
 * \struct ThreadConfig
 * \brief This struct defines the thread config.
 */
struct ThreadConfig {
    uint16_t id{0};
    std::string policy{"SCHED_OTHER"};
    int priority{0};
    std::vector<int> cpu_affinity{1};
};

/**
 * \struct AlgoSwitch
 * \brief This struct defines the algorithm switch.
 */
struct AlgoSwitch { // 算法开关
    bool enable_trail_remove{false};
    bool enable_denoise{false};
    bool enable_high_ref_cross{false};
    bool enable_filter{false};
    bool enable_hor_dist_smooth{false};
    bool enable_hor_ref_smooth{false};
    bool enable_ver_dist_smooth{false};
    bool enable_ver_ref_smooth{false};
    bool enable_stray{false};
    bool enable_spray{false};
    bool data_valid{false};
};

/**
 * \struct TestParam
 * \brief This struct defines the test param.
 */
struct TestParam {
    bool enable_save_pcd{false};
    std::string save_path;
    bool enable_cpu_monitor{false};
    bool enable_print_difop2{false};
    bool enable_mipi_crc_check{false};
    bool enable_difop2_crc_check{false};
    uint32_t print_difop2_cycle{std::numeric_limits<uint32_t>::max()};
    uint32_t cpu_monitor_cycle{std::numeric_limits<uint32_t>::max()};
    uint32_t difop2_crc_check_cycle{std::numeric_limits<uint32_t>::max()};
    double cpu_threshold{100.0};
    bool data_valid{false};
};

/**
 * \enum ErrorCode
 * \brief This enum defines the error code.
 * \var ErrorCode::SUCCESS
 * \var ErrorCode::YAML_FILE_NOT_EXIST
 * \var ErrorCode::PARSE_YAML_NODE_FAILED
 * \var ErrorCode::CRC32_VERIFY_FAULT
 */
enum class ErrorCode : int8_t {
    SUCCESS = 0,
    YAML_FILE_NOT_EXIST = -1,
    PARSE_YAML_NODE_FAILED = -2,
    CRC32_VERIFY_FAULT = -3,
};

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
/**
 * \class YamlAbstract
 * \brief Abstract class of yaml.
 */
template <typename DataType>
class YamlAbstract{
  public:
    YamlAbstract() = default;

    explicit YamlAbstract(const std::string& input_path) noexcept :
        input_yaml_path_(input_path) {
    }

    explicit YamlAbstract(std::string&& input_path) noexcept :
        input_yaml_path_(std::move(input_path)) {
    }

    ~YamlAbstract() = default;

    void setFilePath(const std::string& input_path) noexcept {
        input_yaml_path_ = input_path;
    }

    virtual ErrorCode parseYamlFile() {};
    virtual void getYamlConfigParam(DataType& param) {};
  protected:
    std::string input_yaml_path_;
};

/**
 * \class DifopYaml
 * \brief This class defines the difop yaml.
 */
class DifopYaml : YamlAbstract<utils::Difop2> {
  public:
    using YamlAbstract<utils::Difop2>::YamlAbstract;

    ErrorCode parseYamlFile() override;
    void getYamlConfigParam(utils::Difop2& difop_param) override;

  private:
    ErrorCode parseDifop2Data();
    ErrorCode checkYamlData();
    void swapDifopDataBigEndian(utils::Difop2& difop);

  private:
    utils::Difop2 difop2_param_;
    uint32_t crc32_{0};
};

/**
 * \class AlgoYaml
 * \brief This class defines the algo yaml.
 */
class AlgoYaml : YamlAbstract<AlgoSwitch> {
  public:
    using YamlAbstract<AlgoSwitch>::YamlAbstract;

    ErrorCode parseYamlFile() override;
    void getYamlConfigParam(AlgoSwitch& algo_switch_param) override;

  private:
    ErrorCode parseAlgoSwitchData();
    ErrorCode parseTestParam();

  private:
    AlgoSwitch algo_param_;
    TestParam test_param_;
};

/**
 * \class ThreadConfigYaml
 * \brief This class defines the thread config yaml.
 */
class ThreadConfigYaml : YamlAbstract<std::vector<ThreadConfig>> {
  public:
    using YamlAbstract<std::vector<ThreadConfig>>::YamlAbstract;

    ErrorCode parseYamlFile() override;
    void getYamlConfigParam(std::vector<ThreadConfig>& thread_params) override;

  private:
    ErrorCode parseThreadConfig();

  private:
    std::vector<ThreadConfig> thread_params_;
};

/******************************************************************************/
/*                     Declaration of exported variables                      */
/******************************************************************************/
extern utils::Difop2 difop2_inner_param;
extern AlgoSwitch algo_switch_param;
extern TestParam demo_test_param;
extern std::vector<ThreadConfig> thread_params;

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern bool readAlgoYaml(const std::string& path);

} // namespace robosense::lidar::yaml

/** \} YmlMng */
#endif /* I_YAML_MANAGER_H */