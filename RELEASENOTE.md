# RELEASENOTE

## [3.02.00] - 2026-1-5

1. 关闭CRC算法
2. 增加readDID接口功能
3. 优化拖点算法，解决近距离墙面边缘拖点问题
4. 算法线程减少为1个，降低cpu负载
5. json文件线程配置减少为3个
6. json文件ENABLE_DELETE字段改为ENABLE_SPRAY_DELETE字段
7. 降低互斥锁使用频率