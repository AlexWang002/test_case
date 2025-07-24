/**
 * @file logger_wrap.h
 * @brief 日志器封装类
 * @author shi.dongdong shi.dongdong@byd.com
 * @version 3.5.0
 * @date 2022/03/07
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */

#ifndef LOGGER_WRAP_H
#define LOGGER_WRAP_H

#include "dilog/common/log_singleton.h"
#include "dilog/logger/logger.h"
#include "dilog/utils/log_util.h"


namespace dimw
{
    namespace dilog
    {
        /**
         * @brief 提供各种类型的 Logger; 目前根据需求，提供 Ctx Logger 和 Server Logger。
         */
        class LoggerWrap
        {
        public:
            /**
             * @brief 
             */
            /**
             * @brief  创建普通类型日志器日志器，提供参数校验。
             * @details
             * @param[in] ctxId             ctxID 线程级日志
             * @param[in] ctxDescription    描述
             * @param[in] logLevel          日志等级
             * @param[in] fileNum           文件数量
             * @param[in] fileSize          文件大小
             * @param[in] logformat         日志格式自定义
             * @return Logger::ptr
             */
            Logger::ptr createLogger(const std::string &ctxId, const std::string &ctxDescription,
                                     const dimw::dilog::LogLevel::Level &logLevel = dimw::dilog::LogLevel::Info,
                                     const dimw::dilog::CtxFileInfo &ctxFileInfo = {0U, 0U, "%I{%Y/%m/%d %H:%M:%S.} %M{%Y/%m/%d %H:%M:%S.} %s{%Y/%m/%d %H:%M:%S.} %c %a %C %f %F %L %l [%m]%n"});

        private:
            /// Server 日志器
            Logger::ptr m_server_logger;
        };

        /// 日志器封装类单例模式
        using LogWrap = Singleton<LoggerWrap>;
    }
}
#endif