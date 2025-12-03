# Changelog

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
