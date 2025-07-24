/**
 * @file log_filter.h
 * @brief  
 * @details
 * @author zhao.jiazheng (zhao.jiazheng@byd.com)
 * @version 1.0.0
 * @date 2023/11/30 20:30:06
 * @copyright Copyright (c) 2023 比亚迪股份有限公司
 */

#ifndef LOG_FILTER_H
#define LOG_FILTER_H


#include <memory>
#include "dilog/common/log_common.h"


namespace dimw
{
    namespace dilog
    {
        /**
         * @struct FilterCondition
         * @brief 过滤条件
         */
        typedef struct
        {
            dimw::dilog::LogLevel::Level loggerLevel;
            dimw::dilog::LogLevel::Level runTimeLevel;

            std::string sourceAppId;
            std::string sourceCtxId;
        } FilterCondition;

        /**
         * @class IFilter
         * @brief 日志过滤接口类
         * @details 具体过滤条件由具体子类实现
         */
        class IFilter
        {
        public:
            using ptr = std::shared_ptr<IFilter>;
            /**
             * @brief 构造函数
             */
            IFilter() = default;
// 移动构造函数
            explicit IFilter(IFilter &&e) = delete;

            // 默认赋值构造函数
            explicit IFilter(const IFilter &e) = delete;

            // 复制赋值操作符
            IFilter &operator=(IFilter const &other) = delete;

            // 移动赋值操作符
            IFilter &operator=(IFilter &&other) = delete;

            /**
             * @brief 析构函数
             */
            virtual ~IFilter() = default;

            /**
             * @brief decide
             * @details 根据日志过滤条件过滤日志
             * @param[in] condition 日志过滤条件
             * @return bool
             *       @retval true 日志可被写入
             *       @retval false 不满足日志过滤条件，日志事件将会被丢弃
             */
            virtual bool decide(const FilterCondition &condition) = 0;
        };
    }
}
#endif