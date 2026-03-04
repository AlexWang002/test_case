/**
 * @file log_event_wrap.h
 * @brief  
 * @details
 * @author zhao.jiazheng (zhao.jiazheng@byd.com)
 * @version 1.0.0
 * @date 2023/12/14 19:38:06
 * @copyright Copyright (c) 2023 比亚迪股份有限公司
 */

#ifndef LOG_EVENT_WRAP_H
#define LOG_EVENT_WRAP_H

#include "dilog/event/log_event.h"

namespace dimw
{
    namespace dilog
    {
        /**
         * @brief 日志事件包装器
         */
        class LogEventWrap
        {
        public:
            /**
             * @brief 构造函数
             * @param[in] e 日志事件
             */
            explicit LogEventWrap(std::shared_ptr<LogEvent> e);

            // 移动构造函数
            explicit LogEventWrap(LogEventWrap &&e) = delete;

            // 默认赋值构造函数
            explicit LogEventWrap(const LogEventWrap &e) = delete;

            // 复制赋值操作符
            LogEventWrap &operator=(LogEventWrap const &other) = delete;

            // 移动赋值操作符
            LogEventWrap &operator=(LogEventWrap &&other) = delete;

            /**
             * @brief 析构函数
             */
            ~LogEventWrap();


            /**
             * @brief  获取日志事件
             * @details
             * @return std::shared_ptr<LogEvent>
             */
            std::shared_ptr<LogEvent> getEvent() const
            {
                return m_event;
            }

            /**
             * @brief  获取日志内容流
             * @details
             * @return LogStream&
             */
            LogStream &getSS();

        private:
            /**
             * @brief 日志事件
             */
            std::shared_ptr<LogEvent> m_event;
        };
    }
}
#endif