import re
import csv

with open('rs_lidar_sdk.log', 'r') as file:
    log_content = file.read()

pattern = r"(\d{2}-\d{2}-\d{2}_\d{2}:\d{2}:\d{2}\.\d{3}).*Current=([\d.]+), Average=([\d.]+), Frames=(\d+)"
matches = re.findall(pattern, log_content)

# 准备CSV数据，第一列是时间
csv_data = [["Time", "Current FPS", "Average FPS", "Frames"]]
# 将匹配到的数据添加到CSV数据中
for match in matches:
    # 转换数据类型，时间保持字符串，其他转为适当的数值类型
    time_str, current, average, frames = match
    csv_data.append([
        time_str, 
        float(current), 
        float(average), 
        int(frames)
    ])

# 写入CSV文件
with open('fps_with_time_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(csv_data)

print("数据已成功写入 fps_with_time_data.csv 文件，包含时间信息")