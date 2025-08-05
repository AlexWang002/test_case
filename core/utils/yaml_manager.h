/*******************************************************************************
 * \addtogroup YmlMng
 * \{
 * \headerfile yaml_manager.h
 * \brief This file contains the yaml prase module common class.
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
 * | 0.2 | 2025-06-26 | Add algorithm switch struct |
 *
 ******************************************************************************/
#ifndef I_YAML_MANAGER_H
#define I_YAML_MANAGER_H

/******************************************************************************/
/*                         Include dependant headers                          */
/******************************************************************************/
#include <cstdint>
#include <string>
#include <array>
#include <map>
#include <vector>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include <yaml-cpp/yaml.h>

/******************************************************************************/
/*                  Using namespace, type or template alias                   */
/******************************************************************************/

namespace robosense::lidar::yaml {

/******************************************************************************/
/*        Definition of exported types (typedef, enum, struct, union)         */
/******************************************************************************/
struct Difop2 {
    std::array<uint8_t, 4> info_header;             // 0~4       4 Bytes
    std::array<uint8_t, 63> reserve_data_0;         // 4~67     63 Bytes
    uint8_t surface_count;                          // 67~68     1 Byte
    uint8_t vcsel_pixel_count;                      // 68~69     1 Byte
    uint8_t vcsel_count;                            // 69~70     1 Byte
    std::array<uint8_t, 24> vcsel_yaw_offset;       // 70~94    24 Bytes
    std::array<uint8_t, 384> pixel_pitch;           // 94~478  384 Bytes
    std::array<uint16_t, 2> surface_pitch_offset;   // 478~482   4 Bytes
    uint16_t roll_offset;                           // 482~484   2 Bytes
    std::array<uint8_t, 4> reserve_data_1;          // 484~488   4 Bytes
    uint16_t data_length;                           // 488~490   2 Bytes
    uint16_t counter;                               // 490~492   2 Bytes
    std::array<uint8_t, 4> data_id;                 // 492~496   4 Bytes

    bool data_valid {false};
};

struct ThreadConfig {
    uint16_t id = 0;
    std::string policy = "SCHED_OTHER";
    int priority = 0;
    std::vector<int> cpu_affinity = {0};
};

struct AlgoSwitch { // 算法开关
    bool enable_trail_remove {false};
    bool enable_denoise {false};
    bool enable_high_ref_cross {false};
    bool enable_filter {false};
    bool enable_hor_dist_smooth {false};
    bool enable_hor_ref_smooth {false};
    bool enable_ver_dist_smooth {false};
    bool enable_ver_ref_smooth {false};

    bool data_valid {false};
};

struct TestParam {
    bool enable_save_pcd {false};
    std::string save_path;
    bool enable_cpu_monitor {false};
    bool enable_print_difop2 {false};
    bool enable_mipi_crc_check {false};
    bool enable_difop2_crc_check {false};
    uint32_t print_difop2_cycle {std::numeric_limits<uint32_t>::max()};
    uint32_t cpu_monitor_cycle {std::numeric_limits<uint32_t>::max()};
    uint32_t difop2_crc_check_cycle {std::numeric_limits<uint32_t>::max()};
    double cpu_threshold {100.0};

    bool data_valid {false};
};

enum class ErrorCode : int8_t {
    SUCCESS = 0,
    YAML_FILE_NOT_EXIST = -1,
    PARSE_YAML_NODE_FAILED = -2,
    CRC32_VERIFY_FAULT = -3,
};

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
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

    virtual ErrorCode parseYamlFile() = 0;
    virtual void getYamlConfigParam(DataType& param) = 0;
  protected:
    std::string input_yaml_path_;
};

class DifopYaml : YamlAbstract<Difop2> {
  public:
    using YamlAbstract<Difop2>::YamlAbstract;

    ErrorCode parseYamlFile() override;
    void getYamlConfigParam(Difop2& difop_param) override;

  private:
    ErrorCode parseDifop2Data();
    ErrorCode checkYamlData();
    void swapDifopDataBigEndian(Difop2& difop);

  private:
    Difop2 difop2_param_;
    uint32_t crc32_ {0};
};

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
extern Difop2 difop2_inner_param;
extern AlgoSwitch algo_switch_param;
extern TestParam demo_test_param;
extern std::vector<ThreadConfig> thread_params;

/******************************************************************************/
/*                   Declaration of exported constant data                    */
/******************************************************************************/

/******************************************************************************/
/*                Declaration of exported function prototypes                 */
/******************************************************************************/
extern bool readAlgoYaml(const std::string& path);

} // namespace robosense::lidar::yaml

/** \} YmlMng */
#endif /* I_YAML_MANAGER_H */