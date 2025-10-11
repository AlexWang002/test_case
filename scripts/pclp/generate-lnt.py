# coding: utf-8
# !/usr/bin/python
import os, sys
from shutil import copyfile

pass_files_path = ["/third_party/", "/rs_driver/", "/common/Debug/"]

def proj_lnt_modify(origin_file, file_path):
    new_file = file_path + 'project.lnt'
    fp_new = open(new_file, 'w+')
    fp_origin = open(origin_file, 'r')

    # 第一行数据直接写入
    line = fp_origin.readline()
    fp_new.write(line)

    # 读取剩下数据，处理重复内容
    file_data = fp_origin.read()
    fp_origin.close()

    # 源文件列表
    source_file_list = []
    repeat_file = 0         # 是否是重复文件

    data_block_list = file_data.split("-env_pop", -1)
    for block in data_block_list:
        # 将每个block按行拆分，并提取源文件行
        line_list = block.split("\n", -1)
        if len(line_list) >= 3:
            for line in line_list:
                if ".cpp" in line:
                    source_file = line

            # 剔除三方库代码
            if ("/eol_core/" in source_file):
                pass
            else:
                # 查询是否是重复文件
                for file in source_file_list:
                    if source_file in file:
                        repeat_file = 1

                # 将非重复文件配置写入
                if repeat_file == 0:
                    source_file_list.append(source_file)
                    fp_new.write(block)
                    fp_new.write("-env_pop")

                repeat_file = 0

    fp_new.write("\n")
    fp_new.close()
    # os.remove(origin_file)

def rule_lnt_generate(file_path, compiler_path):
    rule_lnt = file_path + 'rules.lnt'
    if os.path.exists(rule_lnt):
        os.remove(rule_lnt)
    proj_file = file_path + 'project.lnt'
    # 读取工程配置文件
    fp_proj = open(proj_file, 'r')
    proj_data = fp_proj.read()
    fp_proj.close()

    header_list = []
    repeated_flag = 0

    proj_list = proj_data.split('\n', -1)
    for line in proj_list:
        if "-i\"" in line:    # 头文件目录
            if ("/eol_core/" in line):
                for header_file in header_list:
                    if line in header_file: # repeated content
                        repeated_flag = 1

                if repeated_flag == 0:
                    header_list.append(line)
                repeated_flag = 0

    fp_rule = open(rule_lnt, 'w+')

    # 屏蔽PCLint Plus自身规则集相关告警相关
    fp_rule.write("-e*\n")

    # 仅报告error和warning级别
    fp_rule.write("-w2\n")

    # 包含MISRA-C++-2008, AUTOSRA-19规则集，并屏蔽部分规则
    # fp_rule.write("\n{0}/au-misra-cpp.lnt\n".format(file_path))
    fp_rule.write("{0}/au-autosar19.lnt\n".format(file_path))

    # 包含其他规则
    # fp_rule.write("\n+e9070  /* function '__name__' is recursive */\n+elib(9070)\n-append(9070,[CERT C Recommendation MEM05-C])\n")
    # fp_rule.write("\n/* Do not use input functions to convert character data if they cannot handle all possible inputs */\n")
    # fp_rule.write("+e586   /* use of deprecated entity */\n+elib(586)\n")

    fp_rule.write("-efile(*, */3rdparty/dimw/*)\n")
    fp_rule.write("-efile(*, */3rdparty/rs_logger/*)\n")
    fp_rule.write("-efile(*, */3rdparty/yaml-cpp/*)\n")
    fp_rule.write("-efile(*, */3rdparty/json.hpp)\n")

    fp_rule.write("\n-efile(*, /usr/lib/*)\n")
    fp_rule.write("-efile(*, /usr/lib/*/*)\n")
    fp_rule.write("-efile(*, /usr/include/*)\n")
    fp_rule.write("-efile(*, /usr/include/*/*)\n")

    fp_rule.write("-efile(*, */core/algo/*)\n")

    # 剔除部分 supplemental/info/note 类型的警告规则
    supplemental_no = [831, 891, 893, 894, 897]
    info_no = [1714, 1735, 1752, 1762, 1764, 714, 715, 757, 758, 765,
               774,  801,  818,  829,  838,  843, 850, 886]
    note_no = [ 1909, 1912, 1914, 1924, 1928, 1938, 1946, 1954, 3903, 9001,
                9003, 9005, 9007, 9008, 9010, 9011, 9012, 9013, 9016, 9022,
                9023, 9024, 9026, 904,  9042, 9049, 9070, 9071, 9072, 9073,
                9079, 9084, 9090, 9093, 910,  9104, 9105, 9106, 9111, 9113,
                9114, 9115, 9116, 9117, 9118, 9119, 9120, 9122, 9123, 9124,
                9125, 9126, 9128, 9130, 9131, 9132, 9133, 9138, 9141, 9144,
                9145, 9146, 9147, 9148, 9150, 9153, 9158, 9165, 9169, 9173,
                9175, 9176, 9177, 9183, 9215, 9252, 9272, 940,  9416, 9418,
                9419, 9421, 9422, 9424, 9428, 9436, 9437, 9439, 944,  9441,
                9444, 9445, 9449, 9456, 948,  952,  953,  967,  970,  9110,
                9006, 9172, 946,  9121, 9136, 9134, 1715]

    for pass_rule_list in [supplemental_no, info_no, note_no]:
        for rule_no in pass_rule_list:
            fp_rule.write("\n-efile({0}, *)".format(rule_no))
        fp_rule.write("\n\n")

    # 剔除无需告警头文件（源文件已通过proj.lnt剔除）
    for line in header_list:
        fp_rule.write("\n-file(*, {0}/*)\n".format(line[3:-1]))

    fp_rule.close()

def std_lnt_generate(file_path):
    std_lnt = file_path + 'std.lnt'
    fp_std = open(std_lnt, 'w+')
    fp_std.write("rules.lnt\noutput.lnt\nco-g++.lnt\nproject.lnt\n")
    fp_std.close()

def output_lnt_generate(file_path, proj):
    file = file_path + 'output.lnt'
    fp = open(file, 'w+')
    fp.write("-os={0}xml_results_{1}.xml\n".format(file_path, proj))
    fp.write("env-xml.lnt\n")
    fp.close()

if __name__ == '__main__':
    if os.path.exists(sys.argv[1]):
        file_path = sys.argv[1][0:-18]
        proj_lnt_modify(sys.argv[1], file_path)
        rule_lnt_generate(file_path, sys.argv[2])
        output_lnt_generate(file_path, sys.argv[3])
        std_lnt_generate(file_path)
    else:
        print("ERROR: No project lnt file!!!")