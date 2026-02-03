# -*- coding: utf-8 -*-
# @file deviceInfoParse.py
# @brief Parse the device info

import ctypes
import argparse
import os
import glob

class LidarDeviceInfoCtypes(ctypes.Structure):
    """
    使用 ctypes 定义的雷达设备信息结构体，用于与 C++ 代码交互
    对应 C++ 中的 LidarDeviceInfo 结构体，保持相同的内存布局
    """

    _fields_ = [
        ("firmware_version", ctypes.c_uint16),                      # 固件版本号
        ("sdk_version", ctypes.c_uint16),                          # SDK版本号
        ("motor_speed", ctypes.c_uint16),                          # 电机转速，单位rpm
        ("return_mode", ctypes.c_uint8),                           # 回波模式设置
        ("padding_1", ctypes.c_uint8),                # 字节对齐
        ("timestamp", ctypes.c_uint64),                            # 微秒级时间
        ("lidar_operation_state", ctypes.c_uint8),                 # 雷达运行状态
        ("lidar_fault_state", ctypes.c_uint8),                     # 雷达故障状态
        ("sdk_total_fault_number", ctypes.c_uint8),                # SDK故障码总数
        ("padding_2", ctypes.c_uint8 * 5),            # 字节对齐
        ("sdk_fault_code_position", ctypes.c_uint64),              # SDK故障码在队列中的位置
        ("supplier_internal_fault_id", ctypes.c_uint16),           # 供应商内部故障ID
        ("supplier_internal_fault_indicate", ctypes.c_uint8 * 12), # 供应商故障详细指示信息
        ("supplier_internal_fault_number", ctypes.c_uint8),        # 供应商内部故障总数
        ("supplier_internal_fault_position", ctypes.c_uint8),      # 供应商内部故障在队列中的位置
        ("time_sync_mode", ctypes.c_uint8),                        # 时间同步模式
        ("time_sync_status", ctypes.c_uint8),                      # 时间同步状态
        ("padding_3", ctypes.c_uint8 * 6),            # 字节对齐
        ("time_offset", ctypes.c_uint64),                          # 时间偏差
        ("lidar_product_sn", ctypes.c_uint8 * 25),                 # 产品序列号
        ("manufacture", ctypes.c_uint8),                           # 制造商
        ("model", ctypes.c_uint8),                                 # 型号
        ("reserved", ctypes.c_uint8 * 50),                         # 预留字段
        ("padding_4", ctypes.c_uint8 * 3),            # 字节对齐
    ]

    def to_dict(self):
        """将结构体转换为字典，便于处理"""
        return {
            "firmware_version": self.firmware_version,
            "sdk_version": self.sdk_version,
            "motor_speed": self.motor_speed,
            "return_mode": self.return_mode,
            "timestamp": self.timestamp,
            "lidar_operation_state": self.lidar_operation_state,
            "lidar_fault_state": self.lidar_fault_state,
            "sdk_total_fault_number": self.sdk_total_fault_number,
            "sdk_fault_code_position": self.sdk_fault_code_position,
            "supplier_internal_fault_id": self.supplier_internal_fault_id,
            "supplier_internal_fault_indicate": bytes(self.supplier_internal_fault_indicate),
            "supplier_internal_fault_number": self.supplier_internal_fault_number,
            "supplier_internal_fault_position": self.supplier_internal_fault_position,
            "time_sync_mode": self.time_sync_mode,
            "time_sync_status": self.time_sync_status,
            "time_offset": self.time_offset,
            "lidar_product_sn": bytes(self.lidar_product_sn),
            "manufacture": self.manufacture,
            "model": self.model,
            "reserved": bytes(self.reserved),
        }

    def __str__(self):
        """返回对象的字符串表示"""
        data = self.to_dict()
        return (
            f"LidarDeviceInfoCtypes(\n"
            f"  firmware_version: {data['firmware_version']}\n"
            f"  sdk_version: {data['sdk_version']}\n"
            f"  motor_speed: {data['motor_speed']} rpm\n"
            f"  return_mode: {data['return_mode']}\n"
            f"  timestamp: {data['timestamp']} us\n"
            f"  lidar_operation_state: {data['lidar_operation_state']:#04x}\n"
            f"  lidar_fault_state: {data['lidar_fault_state']:#04x}\n"
            f"  sdk_total_fault_number: {data['sdk_total_fault_number']}\n"
            f"  sdk_fault_code_position: {data['sdk_fault_code_position']:#018x}\n"
            f"  supplier_internal_fault_id: {data['supplier_internal_fault_id']}\n"
            f"  supplier_internal_fault_indicate: {data['supplier_internal_fault_indicate'].hex()}\n"
            f"  supplier_internal_fault_number: {data['supplier_internal_fault_number']}\n"
            f"  supplier_internal_fault_position: {data['supplier_internal_fault_position']}\n"
            f"  time_sync_mode: {data['time_sync_mode']}\n"
            f"  time_sync_status: {data['time_sync_status']}\n"
            f"  time_offset: {data['time_offset']}\n"
            f"  lidar_product_sn: {data['lidar_product_sn'].decode('ascii', errors='replace')}\n"
            f"  manufacture: {data['manufacture']}\n"
            f"  model: {data['model']}\n"
            f"  reserved: {data['reserved'].hex()}\n"
            f")"
        )

    def from_str(self, string: str):
        """从字符串解析结构体"""

        string = string.replace("\n", "").replace(" ", "")  # 移除字符串中的换行符和空格
        byte_data = bytes.fromhex(string)                   # 将十六进制字符串转换为字节数组

        if len(byte_data) != ctypes.sizeof(self):           # 检查字节数组长度是否与结构体大小匹配
            raise ValueError(f"字节数组长度({len(byte_data)})与结构体大小({ctypes.sizeof(self)})不匹配")
        ctypes.memmove(ctypes.byref(self), byte_data, ctypes.sizeof(self))  # 使用ctypes将字节数组加载到结构体中

        return self

    @classmethod
    def from_file(cls, file_path: str):
        """从文件解析结构体"""

        with open(file_path, 'r') as f:
            lines = f.readlines()   # 打开文件并读取所有行

        if len(lines) < 2:          # 检查文件是否有足够的行数
            raise ValueError("文件格式不正确，至少需要包含时间戳行和一行十六进制数据")
        hex_str = ''.join(line.strip() for line in lines[1:])        # 跳过第一行时间戳，拼接剩余行的十六进制字符串
        instance = cls()            # 创建结构体实例并解析

        return instance.from_str(hex_str)


def process_path(path):
    """处理单个路径，如果是文件则直接处理，如果是文件夹则遍历所有log文件"""

    files_to_process = []

    if os.path.isfile(path):
        if path.lower().endswith('.log'):                   # 检查文件扩展名是否为.log
            files_to_process.append(path)
        else:
            print(f"警告: 文件 {path} 不是.log文件，将被跳过")
    elif os.path.isdir(path):
        log_files = glob.glob(os.path.join(path, '*.log'))  # 遍历文件夹下所有.log文件

        if log_files:
            files_to_process.extend(log_files)
        else:
            print(f"警告: 文件夹 {path} 中没有找到.log文件")
    else:
        print(f"错误: 路径 {path} 不存在")

    return files_to_process


if __name__ == "__main__":
    """测试代码"""

    parser = argparse.ArgumentParser(description='解析雷达设备信息日志文件')      # 创建命令行参数解析器
    parser.add_argument('paths', nargs='*', help='要解析的.log文件路径或包含.log文件的文件夹路径')    # 添加位置参数，支持多个输入路径
    args = parser.parse_args()  # 解析命令行参数

    if len(args.paths) == 0:    # 如果没有提供命令行参数，使用默认测试数据
        print("\n" + "="*50)
        print("使用默认测试数据:")
        example = LidarDeviceInfoCtypes()
        hex_str = """0529fa756f00010083093adf75430600010201000000000004000000000000000071000000000000
                     0000000000000000010100000000000090c000000000000030303134383234383138373935323138
                     5336313530303130300301007100000000000000ff0001ff3df4ff3df0ff3ef0ff35d4f537c4f537
                     c40000000000000000000000000000000000000000000000"""
        example.from_str(hex_str)
        print("解析后的结构体:")
        print(example)
    else :      # 收集所有需要处理的文件
        all_files = []

        for path in args.paths:
            all_files.extend(process_path(path))

        if not all_files:   # 如果没有找到任何文件，显示帮助信息
            print("错误: 没有找到任何.log文件")
            parser.print_help()
        else:               # 处理每个文件
            print(f"共找到 {len(all_files)} 个.log文件，开始解析...\n")

            for file_path in all_files:
                print(f"=== 解析文件: {file_path} ===")

                try:
                    device_info = LidarDeviceInfoCtypes.from_file(file_path)
                    print(device_info)
                    print("解析成功!\n")
                except Exception as e:
                    print(f"解析失败: {e}\n")
