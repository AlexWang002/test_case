/**
 * @file logger.h
 * @brief  
 * @details
 * @author zhao.jiazheng (zhao.jiazheng@byd.com)
 * @version 1.0.0
 * @date 2023/11/30 13:49:29
 * @copyright Copyright (c) 2023 比亚迪股份有限公司
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <list>
#include "dilog/common/log_mutex.h"
#include "dilog/filter/log_filter.h"

namespace dimw
{
    namespace dilog
    {
        class LogEvent;
        class LogFormatter;
        class LogAppender;

        /**
         * @class Logger
         * @brief 日志器
         */
        class Logger
        {
        public:
            /**
             * @brief  定义自旋锁别名
             * @details
             */
            using MutexType = Spinlock;

            /**
             * @brief  Logger 智能指针
             * @details
             */
            using ptr = std::shared_ptr<Logger>;

            /**
             * @brief 日志级别枚举
             */
            enum Type
            {
                Nomal,   // Ctx logger
                Operate, // 操作日志 : 未来可能存在
                Secure   // 加密日志 : 未来可能存在
            };

            /**
             * @brief  构造 Logger 对象
             * @details
             */
            Logger() = default;
            // 移动构造函数
            explicit Logger(Logger &&e) = delete;

            // 默认赋值构造函数
            explicit Logger(const Logger &e) = delete;

            // 复制赋值操作符
            Logger &operator=(Logger const &other) = delete;

            // 移动赋值操作符
            Logger &operator=(Logger &&other) = delete;

            /**
             * @brief  销毁 Logger 对象
             * @details
             */
            ~Logger();

            /**
             * @brief 构造函数
             * @param[in] ctxId 日志器名称
             * @param[in] ctxDescription 日志器描述
             * @param[in] logLevel 日志器级别
             * @note 该构造函数给 Normal 类型 Logger 使用
             */
            Logger(const std::string &ctxId, const std::string &ctxDescription, const dimw::dilog::LogLevel::Level &logLevel, Type logType = Nomal);

            /**
             * @brief 写日志
             * @param[in] event 日志事件
             */
            void logging(const std::shared_ptr<LogEvent> &event);

            /**
             * @brief 日志过滤
             * @param[in] runTimelevel 运行日志级别
             */
            bool filtering(const dimw::dilog::LogLevel::Level &runTimelevel);

            /**
             * @brief 添加日志目标
             * @param[in] appender 日志目标
             */
            void addAppender(std::shared_ptr<LogAppender> appender);

            /**
             * @brief 删除日志目标
             * @param[in] appender 日志目标
             */
            void delAppender(std::shared_ptr<LogAppender> appender);

            /**
             * @brief 清空日志目标
             */
            void clearAppenders();

            /**
             * @brief 清空过滤目标
             */
            void clearFilters();

            /**
             * @brief 返回线程日志过滤级别
             */
            dimw::dilog::LogLevel::Level getCtxLevel() const
            {
                return m_ctx_level;
            }
            /**
             * @brief  
             * @details
             * @param[in]   level 重置过滤等级
             */
            void resetLevel(dimw::dilog::LogLevel::Level level);
            /**
             * @brief 返回日志名称
             */
            const std::string &getCtxId() const
            {
                return m_ctxId;
            }

            /**
             * @brief 设置日志格式器
             */
            void setFormatter(std::shared_ptr<LogFormatter> val);

            /**
             * @brief 设置日志格式模板
             */
            void setFormatter(const std::string &val);

            /**
             * @brief 获取日志格式器
             */
            std::shared_ptr<LogFormatter> getFormatter();

            /**
             * @brief  添加过滤条件
             * @details
             * @param[in] filter
             */
            void addFilter(const std::shared_ptr<IFilter> &filter);

            /**
             * @brief 删除过滤条件
             * @param[in] filter
             */
            void delFilter(const std::shared_ptr<IFilter> &filter);

            /**
             * @brief 设置 SourceAppId
             * @param[in] appId
             * @note 用于 Client 与 Server 通讯交互
             */
            void setSourceAppId(const std::string &appId)
            {
                m_source_appId = appId;
            }

            /**
             * @brief 返回 SourceAppId
             * @param[in] appId
             * @note 用于 Client 与 Server 通讯交互
             */
            std::string getSourceAppId() const
            {
                return m_source_appId;
            }

            /**
             * @brief 设置 SourceCtxId
             * @param[in] ctxId
             * @note 用于 Client 与 Server 通讯交互
             */
            void setSourceCtxId(const std::string &ctxId)
            {
                m_source_ctxId = ctxId;
            }

            /**
             * @brief 返回 SourceCtxId
             * @note 用于 Client 与 Server 通讯交互
             */
            std::string getSourceCtxId() const
            {
                return m_source_ctxId;
            }

        private:
            /// 日志名称
            std::string m_ctxId;

            /// 日志器描述
            std::string m_ctx_description;

            /// 日志过滤执行级别
            dimw::dilog::LogLevel::Level m_filter_level;

            /// 线程日志过滤级别
            dimw::dilog::LogLevel::Level m_ctx_level;

            /// 锁
            MutexType m_mutex;

            /// 日志过滤设置
            std::list<std::shared_ptr<IFilter>> m_filters;

            /// 输出方向设置
            std::list<std::shared_ptr<LogAppender>> m_appenders;

            /// 日志格式
            std::shared_ptr<LogFormatter> m_formatter;

            /// Server 过滤日志时使用
            std::string m_source_appId;

            /// Server 过滤日志时使用
            std::string m_source_ctxId;
            Type m_log_type = Type::Nomal;
        };
    }
}
#endif