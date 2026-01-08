# Changelog

## [3.02.00] - 2026-1-5

### Update
- 关闭CRC算法
- 增加readDID接口功能
- 优化拖点算法
- 优化杂散算法
- 优化雨雾算法
- 算法线程减少为1个，降低cpu负载
- 更新json文件线程配置
- json文件ENABLE_DELETE字段改为ENABLE_SPRAY_DELETE字段
- 降低receiveCloud互斥锁使用频率
- 减少数据拷贝次数

### Fix

- 修复杂散算法帧间信息失效问题
- 修复遮挡上报故障逻辑
- 修复雨雾标记功能逻辑问题

## [3.01.01] - 2025-12-18

### Update
- 增加内参bin文件解析开关 PARSE_INNER_PARAM_BIN ，默认打开（true）：
  - true  开启：SDK会解析内参bin文件，获取内参数据。
  - false 关闭：SDK会解析MIPI-Difop2, 获取内参数据。

## [3.01.00] - 2025-12-15

### Update
- 优化高度计算函数，减少函数耗时
- 增加PVA 任务提交与等待时间日志
- 增加CPU绑核OTHER策略的nice 值设定功能
- 适配删除MIPI填充数据消影区雷达固件


## [3.00.00] - 2025-12-10

### Update

- 增加json配置VPU使能功能，字段’VPU_AUTH’写true:开启VPU使能；写false:关闭VPU使能
- 增加PVA算法各阶段时延日志打印
- 增加PVA算法任务超时限制
- 优化高度计算算法PVA耗时
- 优化杂散算法CPU部分耗时

### Change

- ag_config.json中算法线程的调度策略调整至FIFO
- 将deviceinfo相关业务移至点云业务后，不计入handlemipi总耗时
- runDeviceInfoCallback回调频率调整至10Hz
- SDK版本号更新为VP3.00.00

### Fix

- 修复帧序号乱序问题
- 解决10Hz偶发中间件getFrame报错问题

## [2.01.01] - 2025-12-02

### Update

- json配置延时打印功能，字段’DELAY_STAT’写true:开启日志打印延时；写false:关闭日志打印延时

### Change

- SDK版本号更新为VP2.01.01

## [2.01.00] - 2025-11-28

### Update

- 优化雨雾算法逻辑
- 去噪同步模型修改
- 高度计算移植到PVA
- PVA算法绑核至VPU1
- 增加PVA异常日志信息
- 修复帧序号乱序问题
- 修复MSOP解析pitch补偿问题

### Change

- SDK版本号更新为VP2.01.00

## [2.00.00] - 2025-11-18

### Update

- 雨雾算法移植到PVA
- 优化拖点算法逻辑
- 上采样算法新增attr输出
- SDK功能与CPU V4.01.04同步

### Change

- SDK版本号更新为VP2.00.00

## [1.03.00] - 2025-10-18

### Update

- 杂散算法部分移植到PVA & 进行向量化优化
- 分辨率重建算法PVA向量化优化
- 拖点算法PVA处理耗时优化
- 新增功能安全功能
- 新增CPU指针转换为GPU指针功能
- 删除yaml文件
- SDK API更新至v1.0.18

### Change

- SDK版本号更新为VP1.03.00

## [1.02.00] - 2025-09-29

### Update

- 优化PVA任务提交负载
- 二维去噪算法VPU向量化优化
- 拖点算法VPU向量化优化
- 雨雾杂散算法流水式处理
- 算法开关客户可配置

### Change

- SDK版本号更新为VP1.02.00

## [1.01.00] - 2025-09-12

### Update

- 二维去噪算法移植到PVA进行处理
- 拖点算法移植到PVA进行处理
- 分辨率重建算法移植到PVA进行处理
- offline demo适配mipi数据回灌
- offline/online demo支持输出pcd文件

### Change

- CPU版本滤波抽取算法适配PVA版SDK
- CPU版本地面拟合算法适配PVA版SDK
- CPU版本雨雾噪点算法适配PVA版SDK
- CPU版本雨雾膨胀算法适配PVA版SDK
- SDK版本号更新为VP1.01.00
