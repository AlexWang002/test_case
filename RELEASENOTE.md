# RELEASENOTE

## [2.01.00] - 2025-11-28

### 更新

1. 增加PVA异常日志信息打印
2. 新增pitch补偿功能（需要客户进行适配）
3. 增加json配置标记切换功能, 字段‘ENABLE_DELETE’ 写true： 删点； 写false： 输出标记
4. 适配10HZ点云版本，device info依旧为60HZ

### 优化

1. 优化雨雾、去噪算法，相对VP2.0版本减少算法延时8ms