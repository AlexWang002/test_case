#ifndef IMAGE_COMMON_H
#define IMAGE_COMMON_H

#include <iostream>
#include "nvscibuf.h"
#include "nvscisync.h"

namespace dimw {
namespace satradar_access {

typedef struct {
    uint32_t sensor;
    uint32_t total_obj;
    uint32_t obj_index;
    uint32_t frame_index;
    uint64_t frame_uuid;
    NvSciBufObj obj;
    uint64_t frame_timestamp;
} BufFrameObj;

/* IRD亮度信息结构体 */
typedef struct {
    int32_t roi_stats_valid = 0;   // defalt 0; 1:valid
    float frame_brightness = 0.0;  // 帧亮度信息
    void *data_ptr = nullptr;      // float 类型数据连续存放
    uint32_t data_len = 0;         // 数据大小，字节数
    uint32_t region_num = 0;       // 感兴趣区域数量, 4
    uint32_t block_per_region = 0; // 每个感兴趣区域划分块的数量 32*32  4*4
    uint32_t value_per_block = 0;  // 每个分块值的数量 4 RGGB信息
    uint32_t frame_index = 0;
} CameraFrameRoiStatsData;

enum SensorName {
    CAMERA_FRONT_RESERVE = 0x00,
    CAMERA_REAR,
    CAMERA_FRONT_WIDE,
    CAMERA_FRONT_NARROW,
    CAMERA_SIDE_LEFT_FRONT,
    CAMERA_SIDE_RIGHT_FRONT,
    CAMERA_SIDE_RIGHT_REAR,
    CAMERA_SIDE_LEFT_REAR,
    CAMERA_SURROUND_FRONT,
    CAMERA_SURROUND_RIGHT,
    CAMERA_SURROUND_REAR,
    CAMERA_SURROUND_LEFT,
    CAMERA_DMS,
    CAMERA_RESERVE1,
    CAMERA_RESERVE2,
    CAMERA_RESERVE3
};

typedef enum {
    CLIENT_SUCCESS = 0,
    CLIENT_INIT_FAILD = -1,
    CLIENT_SYNC_ERROR = -2,
    CLIENT_PARAM_ERROR = -3,
    CLIENT_OPRATE_ERROR = -4,
    CLIENT_BUF_ERROR = -5,
    CLIENT_REMOTE_EXIT = -6,
    CLIENT_TIME_OUT = -7,
    CLIENT_SERVER_REMOTE_ERR = -101,
} ClientErrorCode;

}
}

#endif