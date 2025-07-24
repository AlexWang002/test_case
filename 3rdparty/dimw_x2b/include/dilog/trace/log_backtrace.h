/**
 * @file log_backtrace.cpp
 * @brief  
 * @details
 * @author shi.dongdong (shi.dongdong@byd.com)
 * @version 3.5.0
 * @date 2022/04/28 14:11:02
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */
#ifndef LOG_BACK_TRACE_H
#define LOG_BACK_TRACE_H

#include <iostream>
#include <string>
#include <vector>
namespace dimw
{
    namespace dilog
    {
        /**
         * @brief 获取当前的调用栈
         * @param[out] bt 保存调用栈
         * @param[in] size 最多返回层数
         * @param[in] skip 跳过栈顶的层数
         */
        void backtrace(std::vector<std::string> &bt, int16_t size = 64, int16_t skip = 1);

        /**
         * @brief 获取当前函数栈信息，以字符串格式返回
         * @param[in] size 栈的最大层数
         * @param[in] skip 跳过栈顶的层数
         * @param[in] prefix 栈信息前输出的内容
         */
        std::string backtraceToString(int16_t size = 64, int16_t skip = 2, const std::string &prefix = "");
    }
}

#endif