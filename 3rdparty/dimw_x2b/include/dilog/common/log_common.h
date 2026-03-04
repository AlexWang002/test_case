/**
* @file log_common.h
* @brief 定义日志通用类
* @author shi.dongdong shi.dongdong@byd.com
* @version 3.0.0
* @date 2022/03/03
* @copyright Copyright (c) 2022 比亚迪股份有限公司
*/

#ifndef LOG_COMMON_H
#define LOG_COMMON_H

#include <string>

#define PROXY_BACK_URL "ipc:///tmp/server.url:23450"
#define PROXY_RESET_URL "ipc:///tmp/client.url:23440"
namespace dimw
{
    namespace dilog
    {
        /// 一条日志内容（不包含日志头）的长度最大为 1024 字节
        constexpr uint64_t LOG_LINE_CONTENT_MAX_BYTE{1024U};
        constexpr uint64_t LONG_LINE_UNIT_MAX_BYTE{256U};
        /**
         * @brief 日志级别
        */
        class LogLevel
        {
        public:
            /**
             * @brief 日志级别枚举
             */
            enum Level : uint16_t
            {
                Off = 0,
                Fatal,
                Error,
                Warn,
                Info,
                Debug,
                Verbose
            };

            /**
             * @brief 将日志级别转成文本输出
             * @param[in] level 日志级别
             */
            const static std::string toStr(LogLevel::Level level);
        };

        /**
        * @brief 日志模式类
        */
        class LogMode
        {
        public:
            /**
             * @brief 日志模式枚举
             */
            enum Mode : uint16_t
            {
                Remote = 1,
                File,
                RemoteFile,
                Console,
                RemoteConsole,
                FileConsole,
                RemoteFileConsole
            };

            /**
             * @brief 将日志模式转成文本输出
             * @param[in] mode 日志模式
             */
            const static std::string toStr(LogMode::Mode mode);
        };

        /**
         * @brief     App级涉及File模式存储信息
         * @details
         */
        typedef struct
        {
            uint32_t fileNum;
            uint32_t fileSize;
            bool isTimeStampFormat;
            bool isCompress;
        } AppFileInfo;

        typedef struct
        {
            uint32_t fileNum;
            uint32_t fileSize;
            std::string logFormat;
        } CtxFileInfo;
    }
}

#endif