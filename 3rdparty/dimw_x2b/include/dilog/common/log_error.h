/**
 * @file log_appender.h
 * @brief 定义日志过滤条件和日志过滤逻辑处理类
 * @author shi.dongdong shi.dongdong@byd.com
 * @version 3.5.0
 * @date 2022/03/03
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */

#ifndef LOG_ERROR_H
#define LOG_ERROR_H

#include <iostream>
#include <cstring>

namespace dimw
{
    namespace dilog
    {
/**
 * @brief 日志客户端，打印日志宏定义，输出控制台
 * @param fmt 可扩展参数
 * @return void
 * @note   该函数打印为日志模块正常打印，该函数不区分 release 版本和 debug 版本
 */
/*LDRA_NOANALYSIS*/
#define MTFILE(x) strrchr(x, '/') ? strrchr(x, '/') + 1 : x
#define LTLOG(fmt, ...)                                              \
    printf("%s:%d " fmt, MTFILE(__FILE__), __LINE__, ##__VA_ARGS__); \
    printf("\n")
        /*LDRA_ANALYSIS*/
        class LogError
        {
        public:
            /*LDRA_NOANALYSIS*/
            /**
             * @brief 日志错误码枚举
             */
            enum Code
            {
                Success = 0U,          // APP 初始化错误码范围 1 - 50
                ErrAppIdCheckFailed,   /*"AppId check failed, please check!"*/
                ErrAppIdRepeatRegged,  /*"AppId forbid to regging repeated, please check!"*/
                ErrCtxRegSequence,     /*"CTX ensure to reg before AppId, please check!"*/
                ErrCtxCheckFailed      /*"CtxId check failed, please check!"*/
            };

            /**
             * @brief 根据日志错误码，获取错误具体原因
             */
            static const char *msg[];
            /*LDRA_ANALYSIS*/
        };
    }
}
#endif
