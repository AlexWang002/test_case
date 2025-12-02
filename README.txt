域控端运行PVA版本rs_demo_online及SDK相关步骤：
1. 执行指令：
    sudo echo 0 > /sys/kernel/debug/pva0/vpu_app_authentication
2. 执行指令，启动demo：
    PVA_GEN=GEN2 ./rs_demo_online

域控端监测VPU负载步骤：
1. 执行指令：
    echo Y | sudo tee /sys/kernel/debug/pva0/stats_enabled
2. 执行指令：
    echo 2 | sudo tee /sys/kernel/debug/pva0/profiling_level
3. 执行指令，监测VPU负载，每0.1s打印一次：
    while true; do sudo cat /sys/kernel/debug/pva0/vpu_stats | tail -n 2 | xargs ; sleep 0.1; done

说明：
1. 开启VPU负载监测后，终端左侧输出为VPU0负载，右侧为VPU1负载。例：1721 2437表示此时VP0负载为17.21%, VPU1负载为24.37%
2. 不可同时在多个终端下运行多条负载监测指令，该情况下输出的VPU负载值不准确