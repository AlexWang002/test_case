/**
 * @file log_trace_method.h
 * @brief  
 * @details
 * @author shi.dongdong (shi.dongdong@byd.com)
 * @version 3.5.0
 * @date 2022/04/28 14:10:50
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */
#ifndef LOG_TRACE_METHOD_H
#define LOG_TRACE_METHOD_H
#include <iostream>

namespace dimw
{
    namespace dilog
    {
        class TraceMethod
        {
        public:
            TraceMethod(const char *keywords, const char *file);
            ~TraceMethod();

        private:
            const char *m_file;
            const char *m_function;
        };
    }
}
#endif