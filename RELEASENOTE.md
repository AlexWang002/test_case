# RELEASENOTE

## [3.00.00] - 2025-12-10

1. 增加json配置VPU使能功能，字段’VPU_AUTH’写true:开启VPU使能；写false:关闭VPU使能
2. 增加PVA算法各阶段时延日志打印
3. 增加PVA算法任务超时限制
4. 优化高度计算算法耗时
5. 优化杂散算法耗时
6. ag_config.json中算法线程的调度策略调整至FIFO
7. 将deviceinfo相关业务移至点云业务后，不计入handlemipi总耗时中
8. runDeviceInfoCallback回调频率调整至10Hz