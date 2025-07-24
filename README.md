# 激光雷达 SDK 示例程序说明

## 概述
本示例程序展示了如何使用激光雷达 SDK 进行数据处理和点云生成。程序模拟了从底软接口获取数据，并通过 SDK 进行处理，最终输出点云数据。

## 编译环境要求
- C++11 或更高版本的编译器（如 g++ 4.8 及以上）
- CMake 3.10 及以上（可选，用于简化编译过程）
- 激光雷达 SDK 库文件（liblidar_sdk.so）

## 编译步骤

```bash
mkdir build
cd build
cmake ..
make
```

## 库文件处理
当编译平台和运行平台不一致时，确保 SDK 库文件（liblidar_sdk.so）对程序可见：

**方法一**：
- 将可执行文件和库文件放在同一目录下

**方法二**：
- 通过环境变量指定库文件搜索路径：
    ```bash
    export LD_LIBRARY_PATH=/path/to/sdk/lib:$LD_LIBRARY_PATH
    ```
- 永久生效配置（添加到 `~/.bashrc`）：
    ```bash
    echo 'export LD_LIBRARY_PATH=/path/to/sdk/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
    source ~/.bashrc
    ```

## 运行程序

### 准备数据
- 默认程序会从指定数据目录下读取 10 个帧的数据文件（`frame_0.txt` 到 `frame_9.txt`）
- 可通过命令行参数指定数据路径：

### 运行命令
```bash
./demo -path /path/to/your/data/
```
