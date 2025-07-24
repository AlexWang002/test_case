/**
 * @file log_event.h
 * @brief 定义日志事件
 * @author zhao.jiazheng  zhao.jiazheng @byd.com
 * @version 3.5.0
 * @date 2022/03/03
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */

#ifndef LOG_EVENT_H
#define LOG_EVENT_H

#include <sstream>
#include "planetTimeMgr/planetUtcMgr.h"
#include "dataplanetimesync/dataplanetimesync.h"
#include "dilog/logger/logger.h"
#include "dilog/common/log_stream.h"

namespace dimw
{
    namespace dilog
    {
        /**
         * @brief 日志事件
         */
        class LogEvent
        {
        public:
            using ptr = std::shared_ptr<LogEvent>;
            LogEvent() = default;
            LogEvent(const std::shared_ptr<Logger> &logHeader) : m_logger(logHeader) {}
            /**
             * @brief 构造函数
             * @param[in] logger 日志器
             * @param[in] level 日志级别
             * @param[in] file 文件名
             * @param[in] func  函数名
             * @param[in] line 文件行号
             */
            LogEvent(const std::shared_ptr<Logger> &logHeader, const dimw::dilog::LogLevel::Level &level, const char *const file, const char *const func, const int32_t line);

            /**
             * @brief  Destroy the Log Event object
             * @details
             */
            ~LogEvent() { m_ss.str().clear(); };

            /**
             * @brief 返回日志事件入队列时间
             */
            uint64_t getTime() const;

            void setTime();

            /**
             * @brief 返回日志事件入队列时钟时间
             */
            uint64_t getSteadyTime() const;

            void setSteadyTime();

            // /**
            //  * @brief 返回日志事件入队列数据面时间
            //  */
            uint64_t getDataTime() const;

            void setDataTime();

            /**
             * @brief 返回日志事件入队列管理面时间
             */
            uint64_t getManageTime() const;

            void setManageTime();

            /**
             * @brief 返回日志事件写入文件时间
             */
            uint64_t getWriteFileTime() const;

            /**
             * @brief 设置日志事件写入文件时间
             */
            void setWriteFileTime(const uint64_t &time);

            /**
             * @brief 返回连续数
             */
            uint32_t getContinuousNumber() const;

            void setContinuousNumber();

            /**
             * @brief 返回 AppId
             */
            std::string getAppId() const;

            /**
             * @brief 返回 CtxId
             */
            std::string getCtxId() const;

            /**
             * @brief 返回日志级别
             */
            dimw::dilog::LogLevel::Level getLevel() const;

            /**
             * @brief 返回文件名
             */
            const char *getFile() const;

            /**
             * @brief 返回函数名
             */
            const char *getFunction() const;

            /**
             * @brief 返回行号
             */
            int32_t getLine() const;

            /**
             * @brief 返回日志器
             */
            std::shared_ptr<Logger> getLogger() const;

            /**
             * @brief 返回日志内容字符串流
             */
            /*LDRA_NOANALYSIS*/
            LogStream &getSS();

            /*LDRA_ANALYSIS*/

            /**
             * @brief 返回日志内容
             */
            std::string getContent();

            /**
             * @brief 格式化写入日志内容
             */
            void format(const char *const fmt, ...);

            /**
             * @brief 格式化写入日志内容
             */
            void format(const char *const fmt, va_list al);

        private:
            /// 时间戳 : Event 创建时间
            uint64_t m_time;

            /// 时间戳 : Event 创建时数据面时间
            uint64_t m_data_time;

            /// 时间戳 : Event 创建时管理面时间
            uint64_t m_manage_time;

            /// 时钟时间戳 : Event 创建时间
            uint64_t m_steadyTime;

            /// 时间戳 : 文件落盘时间
            uint64_t m_writeFileTime;

            /// 连续数
            uint32_t m_continuous_number = 0U;

            /// appId 应用进程 Id
            std::string m_appId;

            /// ctxId 线程 Id
            std::string m_ctxId;

            /// 日志级别
            dimw::dilog::LogLevel::Level m_level;

            /// 文件名
            const char *m_file = nullptr;

            /// 函数名
            const char *m_function = nullptr;

            /// 行号
            int32_t m_line = 0;

            /// 日志内容流
            LogStream m_ss;

            /// 日志器
            std::shared_ptr<Logger> m_logger;
        };
    }
}
#endif
