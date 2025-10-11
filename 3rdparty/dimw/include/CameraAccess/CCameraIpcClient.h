#ifndef CAMERA_IPC_CLIENT_HPP
#define CAMERA_IPC_CLIENT_HPP
#include "ImageCommon.hpp"
#include <vector>

namespace dimw {
namespace cameraipcclient {

class CCameraIpcClient {
public:
    CCameraIpcClient(AppType appType = AppType::CAM_IRC_CONSUMER);
    ~CCameraIpcClient() = default;

    /**
     * @brief 初始化
     * 
     * @param mask [in] mask ：每个bit标识SensorId，每个bit定义见枚举类型SensorName
     * @param isRawEnable [in] isRawEnable ：使能raw功能标志位，使能为true，不使能为false，默认为false；
     * @return 0：success ，-1：faild 
     */
    int Init(uint16_t &mask, bool isRawEnable = false);

    /**
     * \brief：使能数据同步操作，成功或者超时后返回
     * \return :0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    int Start(void);

    /**
     * \brief：获取一帧camera数据，和 ReleaseFrame 配对使用;
     * \param[in] sensor ：指定要获取图像的摄像头通道
     * \param[out] frameobj ：获取到一帧数据信息，NvSciBufObj obj 中包camera 真实的camera数据，请勿修改frameobj
     * 中任何参数的值 通过NvSciBufObjGetConstCpuPtr可获取内存地址 \return :0：success ;<0:失败，根据 ClientErrorCode
     * 定义查看具体原因。
     */
    int GetFrame(SensorName sensor, BufFrameObj *frameobj);

    /**
     * \brief：释放一帧camera数据 和 GetFrame 配对使用
     * \param[in] obj ：释放 sensor 和 pip 指定camera 数据（使用GetFrame 中获取到 frameobj 对象）
     * \return :0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    int ReleaseFrame(const BufFrameObj *frameobj);

    /**
     * \brief：stop
     */
    void Stop(void);
    /**
     * \brief：去初始化
     */
    void DeInit(void);

    /**
     * \brief：根据 index 获取frame 的分块亮度信息。
     * \param[in] frameobj
     * \param[out] data 扩展信息指针
     * \return :0：success ;<0:失败，根据 ClientErrorCode 定义查看具体原因。
     */
    int GetFrameRoiStatsData(const BufFrameObj *frameobj, CameraFrameRoiStatsData *data);

    /**
     * @brief 保存图像至文件
     * 
     * @param obj 
     * @return int 
     */
    int WriteImageToFile(BufFrameObj *obj);

    /**
     * @brief 保存RAW图像至文件
     * 
     * @param obj 
     * @return int 
     */
    int WriteRawImageToFile(BufFrameObj *obj);

    /**
     * @brief 设置某个模组的fsync偏差，groupId：0前视，1周视，2环视，3雷达
     * 
     * @param groupId 
     * @param delayTimeMs 
     * @return ClientErrorCode 
     */
    int setFsyncDeltaMs(GroupId groupId, uint64_t delayTimeMs);

    /**
     * @brief 获取所有传感器的类型（相机+雷达）
     * 
     * @param allSensorTypeInfo 
     * @return ClientErrorCode 
     */
    int getAllSensorTypeInfo(std::vector<SensorTypeInfo>& allSensorTypeInfo);

private:
    AppType m_appType;
};

}
}

#endif