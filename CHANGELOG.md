# Changelog

## [3.00.01] - 2025-07-18

### Update

- 增加线程配置yaml，使用初始化接口的参数
- 用线程配置yaml来管理不同线程的调度策略，优先级，绑核属性

### Change

- 将getRSLidarSdkInterface扩展为C
- SDK版本号更新为v3.00.01


## [3.00.00] - 2025-07-15


### Update

- 更新适配API为V1.0.12功能开发
- 增加地面拟合算法 V1.0.0
- 增加 mirror_id 功能适配开发

### Change

- SDK版本号更新为v3.00.00
- 日志路径变更为/applog/lidar
- 从 difop2 bin 文件读取内参
- 更新上采样算法 V1.0.1

### Fixed

- 修复mipi_data队列溢出异常


## [2.01.01] - 2025-06-30

### Update

- 适配API/SDKV1.0.11 功能开发
- 适配雷达点云/状态格式 V5.0 版本功能开发

## [2.1.0] - 2025-06-27

### Update

- API/SDK 版本更新 V1.0.10
- 雷达点云格式、雷达状态格式使用 V4.0 版本

### Fixed

- 解决客户阻塞 50ms 数据错乱问题，新开线程单独发送 MSOP
- 修复时间同步后时间戳解析异常问题

### Changed

- 增加 yaml 配置、解析算法开关和 pcd 存储配置的功能
- 点云 buffer 锁优化，降低资源消耗
- DID 接口读 SDK 版本号，使用 DID：0x1112U，数据使用大端
- 优化毛刺问题
- 增加开发去噪算法 高反算法

## [2.0.0] - 2025-06-20

### Fixed

- 解决因点云算法解析正反扫描 bug 导致 PCD 存储有效点变少的问题
- 将回调指针函数句柄由引用修改为拷贝，解决客户上层释放问题
- 修复 mipi 数据队列超限后未调用 releaseAdc 问题

### Changed

- 更新 lidar_sdk_api.h 头文件版本为 1.0.9
- 将 injectAdc 接口输入的指针从拷贝改为直接保存原始指针
- 增加 IIC 安全下电时序功能
- 优化算法 CPU 负载

## [1.0.2] - 2025-06-10

### Changed

- 编译选项从 Release 改为 Debug

## [1.0.1] - 2025-06-09

### Changed

- 适配在线雷达。
- 多线程分配优化，数据结构优化。

### Feat

- 优化 Stop 函数延时。

## [0.1.0] - 2025-06-05

### Changed

- 屏蔽 deviceinfo 解析。

### Fixed

- 修复 Stop 函数阻塞问题。

## [0.0.8] - 2025-05-30

### Added

- 无。

### Changed

- 更新 mipi 数据格式。

### Fixed

- 无。

### Removed

- 无。

## [0.0.7] - 2025-05-28

### Added

- 无。

### Changed

- 更新 lidar_sdk_api.h 头文件版本为 1.0.7。

### Fixed

- 无。

### Removed

- 无。

## [0.0.6] - 2025-05-23

### Added

- 无。

### Changed

- 算法处理改为多核模式。

### Fixed

- 无。

### Removed

- 无。

## [0.0.4] - 2025-05-22

### Added

- 无。

### Changed

- 更新 lidar_sdk_api.h 头文件版本为 1.0.5。
- 更新 demo 示例，增加点云解析实例。

### Fixed

- 修复无命名空间的问题。

### Removed

- 无。

## [0.0.3] - 2025-05-15

### Added

- 初始版本发布。

### Changed

- 无。

### Fixed

- 无。

### Removed

- 无。
