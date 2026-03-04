/**
 * @file log_interface.h
 * @brief  
 * @details
 * @author zhao.jiazheng (zhao.jiazheng@byd.com)
 * @version 1.0.0
 * @date 2023/12/14 19:52:13
 * @copyright Copyright (c) 2023 比亚迪股份有限公司
 */
#ifndef LOG_INTERFACE_H
#define LOG_INTERFACE_H

#include "dilog/common/log_error.h"
#include "dilog/event/log_event_wrap.h"
#include "dilog/trace/log_trace_method.h"
#include "dilog/trace/log_backtrace.h"

namespace dimw
{
    namespace dilog
    {

        /**
         * @brief                       initLogging 功能函数
         * @details                     注册应用ID，初始化当前应用的日志信息。
         * @param[in]  appId            对 APPID 校验:长度限制在4字节内，且仅由字母、数字、下划线组成(不满足报错并退出)；
         * @param[in]  appDescription   对 APP的描述: 长度限制在256字节，超过256字节做截断处理；
         * @param[in]  logMode          日志输出模式: 远程日志记录(Remote)、文件日志记录(File)、控制台日志打印(Console)三种模式可单选，可组合多选；
         * @param[in]  logLevel         日志输出级别: 包括Off, Verbose, Debug, Info, Warn, Error, Fatal，默认级别为 Info 级别；
         * @param[in]  appFileInfo      日志文件存储方式信息，包含  日志存储路径、 日志存储文件限定数量、日志存储文件大小限定、文件名是否带时间戳、文件是否压缩；
         * @return LogError::Code       初始化标识: Success 表示成功。
         * @note    		            APPID，用于区分不同进程日志，由调用方保证 APPID 的唯一性。
         *                              logFileDir，当日志模式为指定路径存储时，日志文件存储在 logFileDir 路径下，若不指定路径，默认文件存储位置为当前应用进程所在目录。
         *                              File 模式下，日志文件命名为 DiLOG_ + APPID + 时间戳，格式为txt文件, 例如: DiLOG_APPID_20211022_150228_199539.txt。
         */
        LogError::Code
        initLogging(const std::string &appId, const std::string &appDescription,
                    const dimw::dilog::LogMode::Mode &logMode,
                    const dimw::dilog::LogLevel::Level &logLevel = dimw::dilog::LogLevel::Info,
                    const std::string &logFileDir = "/oslog/dimw/file/",
                    const dimw::dilog::AppFileInfo &appFileInfo = {10U, 5U, false, false});
        /**
         * @brief  
         * @details
         * @param[in] appLogLevel       重置App 级 日志过滤等级
         */
        void resetLogLevel(const LogLevel::Level &appLogLevel);

        /**
         * @brief                       createLogger 功能函数
         * @details                     在应用ID初始化完成后，注册线程ID，初始化当前线程日志信息。
         * @param[in] ctxId             模块上下文CTXID，限制在4字节内，同一个应用内唯一; 超出4字节，（超过4字节返回错误）。
         * @param[in] ctxDescription    模块上下文描述信息，限制为256个Byte大小, 超过256Byte做截断处理。
         * @param[in] ctxLogLevel       日志默认输出级别包括 Off, Verbose, Debug, Info, Warn, Error, Fatal; 默认级别为 Info
         * @return createLogger
         * @retval Logger::logger 返回根据当前线程初始化信息创建的日志器指针: 执行成功
         * @retval nullptr        返回空指针 : 执行失败
         * @note                        当 CtxId 定义的 LogLevel 与 AppId 定义的 LogLevel 不一致时，取两者最小值; 注册线程日志打印时，由调用方保证 ctxId 进程内唯一。
         */
        Logger::ptr createLogger(const std::string &ctxId, const std::string &ctxDescription, const dimw::dilog::LogLevel::Level &ctxLogLevel = dimw::dilog::LogLevel::Info,
                                 const dimw::dilog::CtxFileInfo &ctxFileInfo = {0U, 0U, "%I{%Y/%m/%d %H:%M:%S.} %M{%Y/%m/%d %H:%M:%S.} %d{%Y/%m/%d %H:%M:%S.} %s{%Y/%m/%d %H:%M:%S.} %c %a %C %f %F %L %l [%m]%n"});

        /**
         * @brief                       createOperateLogger 功能函数
         * @details                     在应用ID初始化完成后，注册操作ID，目的可更改当前ssessionID 下日志存储路径。
         * @param[in] sessionId         操作日志ID
         * @param[in] filePath          文件路径   
         * @param[in] optLogLevel       过滤条件
         * @return Logger::ptr
         */
        Logger::ptr createOperateLogger(const std::string &sessionId, const std::string &filePath = "/oslog/update/operationlog/",
                                        const dimw::dilog::LogLevel::Level &optLogLevel = dimw::dilog::LogLevel::Verbose, const std::string &logFormat = "%I{%Y/%m/%d %H:%M:%S.} %M{%Y/%m/%d %H:%M:%S.} %d{%Y/%m/%d %H:%M:%S.} %s{%Y/%m/%d %H:%M:%S.} %c %a %C %f %F %L %l [%m]%n");
        /**
         * @brief  
         * @details
         * @param[in] ctxLoggerparameter description
         * @param[in] logLevelparameter description
         * @return true
         * @return false
         */
        bool fiterLog(dimw::dilog::Logger::ptr ctxLogger, dimw::dilog::LogLevel::Level logLevel);
/*LDRA_NOANALYSIS*/
// 写日志接口
/**
 * @brief 使用流式方式将日志级别debug的日志写入到logger
 */
#define DILOG_VERBOSE(logger) DILOG_LEVEL(logger, dimw::dilog::LogLevel::Verbose)

/**
 * @brief 使用流式方式将日志级别debug的日志写入到logger
 */
#define DILOG_DEBUG(logger) DILOG_LEVEL(logger, dimw::dilog::LogLevel::Debug)

/**
 * @brief 使用流式方式将日志级别info的日志写入到logger
 */
#define DILOG_INFO(logger) DILOG_LEVEL(logger, dimw::dilog::LogLevel::Info)

/**
 * @brief 使用流式方式将日志级别warn的日志写入到logger
 */
#define DILOG_WARN(logger) DILOG_LEVEL(logger, dimw::dilog::LogLevel::Warn)

/**
 * @brief 使用流式方式将日志级别error的日志写入到logger
 */
#define DILOG_ERROR(logger) DILOG_LEVEL(logger, dimw::dilog::LogLevel::Error)

/**
 * @brief 使用流式方式将日志级别fatal的日志写入到logger
 */
#define DILOG_FATAL(logger) DILOG_LEVEL(logger, dimw::dilog::LogLevel::Fatal)

/**
 * @brief 使用格式化方式将日志级别debug的日志写入到logger
 */
#define DILOG_FMT_VERBOSE(logger, fmt, ...) DILOG_FMT_LEVEL(logger, dimw::dilog::LogLevel::Verbose, fmt, ##__VA_ARGS__)

/**
 * @brief 使用格式化方式将日志级别debug的日志写入到logger
 */
#define DILOG_FMT_DEBUG(logger, fmt, ...) DILOG_FMT_LEVEL(logger, dimw::dilog::LogLevel::Debug, fmt, ##__VA_ARGS__)

/**
 * @brief 使用格式化方式将日志级别info的日志写入到logger
 */
#define DILOG_FMT_INFO(logger, fmt, ...) DILOG_FMT_LEVEL(logger, dimw::dilog::LogLevel::Info, fmt, ##__VA_ARGS__)

/**
 * @brief 使用格式化方式将日志级别warn的日志写入到logger
 */
#define DILOG_FMT_WARN(logger, fmt, ...) DILOG_FMT_LEVEL(logger, dimw::dilog::LogLevel::Warn, fmt, ##__VA_ARGS__)

/**
 * @brief 使用格式化方式将日志级别error的日志写入到logger
 */
#define DILOG_FMT_ERROR(logger, fmt, ...) DILOG_FMT_LEVEL(logger, dimw::dilog::LogLevel::Error, fmt, ##__VA_ARGS__)

/**
 * @brief 使用格式化方式将日志级别fatal的日志写入到logger
 */
#define DILOG_FMT_FATAL(logger, fmt, ...) DILOG_FMT_LEVEL(logger, dimw::dilog::LogLevel::Fatal, fmt, ##__VA_ARGS__)

/**
 * @brief 使用流式方式将日志级别level的日志写入到logger
 */
#define DILOG_LEVEL(logger, level)                                                                                          \
    if (fiterLog(logger, level))                                                                                            \
    dimw::dilog::LogEventWrap(std::make_shared<dimw::dilog::LogEvent>(logger, level, MTFILE(__FILE__), __func__, __LINE__)) \
        .getSS()
/**
 * @brief 使用格式化方式将日志级别level的日志写入到logger
 */
#define DILOG_FMT_LEVEL(logger, level, fmt, ...)                                                                            \
    if (fiterLog(logger, level))                                                                                            \
    dimw::dilog::LogEventWrap(std::make_shared<dimw::dilog::LogEvent>(logger, level, MTFILE(__FILE__), __func__, __LINE__)) \
        .getEvent()                                                                                                         \
        ->format(fmt, ##__VA_ARGS__)

/**
 * @brief  打印函数进出
 */
#define DILOG_TRACE_METHOD() dimw::dilog::TraceMethod tmp = dimw::dilog::TraceMethod(__FILE__, __func__)

/**
 * @brief  打印函数栈信息
 * @details
 */
#define DILOG_BACKTRACE()                                                                                      \
    {                                                                                                          \
        std::cout << "---------------------------- BackTrace Start ----------------------------" << std::endl; \
        std::cout << backtraceToString() << std::endl;                                                         \
        std::cout << "---------------------------- BackTrace End ----------------------------" << std::endl;   \
    }
        /*LDRA_ANALYSIS*/
    }
}

#endif
