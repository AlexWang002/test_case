/**
 * @file log_stream.h
 * @brief
 * @details
 * @author shi.dongdong (shi.dongdong@byd.com)
 * @version 3.5.0
 * @date 2022/03/17 08:51:09
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */
#ifndef LOG_STREAM_H
#define LOG_STREAM_H
#include <string>
#include <cstring>
#include <cstdint>
namespace dimw
{
    namespace dilog
    {
        /**
         * @brief  日志流重载类
         * @details
         */
        class LogStream
        {
        public:
            // C++内置类型，整形(字符，整形，bool)，浮点型，分别重载输出运算符
            ~LogStream()
            {
                m_str.clear();
            };
            /**
             * @brief operator<< bool
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(bool v);

            /**
             * @brief  重载 signed char / int8_t
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(signed char v); // int8_t

            /**
             * @brief  重载 unsigned char / uint8_t
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(unsigned char v); // uint8_t

            /**
             * @brief  重载 short
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(short v); // int16_t

            /**
             * @brief  重载 unsigned short
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(unsigned short v); // uint16_t

            /**
             * @brief  重载 int32_t
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(int32_t v); // int32_t

            /**
             * @brief  重载 uint32_t
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(uint32_t v); // uint32_t

            /**
             * @brief  重载 long
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(long v);

            /**
             * @brief  重载 unsigned long
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(unsigned long v);

            /**
             * @brief  重载 long long
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(long long v); // int64_t

            /**
             * @brief  重载 unsigned long long
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(unsigned long long v); // uint64_t

            /**
             * @brief  重载 double
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(double v); // double

            /**
             * @brief  重载浮点输出
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(float v); // float

            /**
             * @brief  重载字符串输出
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(const std::string &v); // string

            /**
             * @brief  重载 const char *
             * @details
             * @param[in] str
             * @return LogStream&
             */
            LogStream &operator<<(const char *const str); // char *

            /**
             * @brief  重载指针输出
             * @details
             * @return LogStream&
             */
            LogStream &operator<<(const void *); // ptr

            /**
             * @brief  重载字符输出
             * @details
             * @param[in] v
             * @return LogStream&
             */
            LogStream &operator<<(char v); // char

            /**
             * @brief  返回 LogStream 对象的字符串
             * @details
             * @return std::string
             */
            std::string str()
            {
                return m_str;
            };

        private:
            /**
             * @brief  类型转换为字符串模板
             * @details
             * @tparam  T
             * @return const std::string
             */
            template <typename T>
            const std::string toFormatStr(T);

            /// LogStream 对象的字符串
            std::string m_str;

            /// 已经使用的字符长度。 LogStream 尽可能交付的字节数为 1024字节以内，但不能保证。 需要调用方保证字符串长度限制
            uint32_t m_used_size = 0U;
            // 单元最大长度
            uint32_t m_unit_size = 0U;

            /**
             * @brief  是否能够继续记录消息
             * @details 通过以记录字符串长度是否在 1024 byte 以内
             * @return true
             * @return false
             */
            bool canRecordMsg();
        };
    }
}
#endif