/**
 * @file log_mutex.h
 * @brief
 * @author shi.dongdong (shi.dongdong@byd.com)
 * @version 3.5.0
 * @date 2022/03/07 10:37:12
 * @copyright Copyright (c) 2022 比亚迪股份有限公司
 */

#ifndef MUTEX_H
#define MUTEX_H

#include <memory>
#include <pthread.h>
#include <semaphore.h>
#include <atomic>

namespace dimw
{
    namespace dilog
    {
        /**
         * @brief 对象无法拷贝,赋值
         */
        class Noncopyable
        {
        public:
            /**
             * @brief 默认构造函数
             */
            Noncopyable() = default;

            /**
             * @brief 默认析构函数
             */
            ~Noncopyable() = default;

            /**
             * @brief 拷贝构造函数(禁用)
             */
            Noncopyable(const Noncopyable &) = delete;

            /**
             * @brief 赋值函数(禁用)
             */
            Noncopyable &operator=(const Noncopyable &) = delete;
        };

        /**
         * @brief 信号量
         */
        class Semaphore : Noncopyable
        {
        public:
            /**
             * @brief 构造函数
             * @param[in] count 信号量值的大小
             */
            Semaphore(uint32_t count = 0);

            /**
             * @brief 析构函数
             */
            ~Semaphore();

            /**
             * @brief 获取信号量
             */
            void wait();

            /**
             * @brief 释放信号量
             */
            void notify();

        private:
            sem_t m_semaphore;
        };

        /**
         * @brief 局部锁的模板实现
         */
        template <class T>
        struct ScopedLockImpl
        {
        public:
            /**
             * @brief 构造函数
             * @param[in] mutex Mutex
             */
            ScopedLockImpl(T &mutex)
                : m_mutex(mutex)
            {
                m_mutex.lock();
                m_locked = true;
            }

            /**
             * @brief 析构函数,自动释放锁
             */
            ~ScopedLockImpl()
            {
                unlock();
            }

            /**
             * @brief 加锁
             */
            void lock()
            {
                if (!m_locked)
                {
                    m_mutex.lock();
                    m_locked = true;
                }
            }

            /**
             * @brief 解锁
             */
            void unlock()
            {
                if (m_locked)
                {
                    m_mutex.unlock();
                    m_locked = false;
                }
            }

        private:
            /// mutex
            T &m_mutex;
            /// 是否已上锁
            bool m_locked;
        };

        /**
         * @brief 自旋锁
         */
        class Spinlock : Noncopyable
        {
        public:
            /// 局部锁
            using Lock = ScopedLockImpl<Spinlock>;

            /**
             * @brief 构造函数
             */
            Spinlock()
            {
                pthread_spin_init(&m_mutex, 0);
            }

            /**
             * @brief 析构函数
             */
            ~Spinlock()
            {
                pthread_spin_destroy(&m_mutex);
            }

            /**
             * @brief 上锁
             */
            void lock()
            {
                pthread_spin_lock(&m_mutex);
            }

            /**
             * @brief 解锁
             */
            void unlock()
            {
                pthread_spin_unlock(&m_mutex);
            }

        private:
            /// 自旋锁
            pthread_spinlock_t m_mutex;
        };

        /**
         * @brief 原子锁
         */
        // CAS : Compare And SWAP
        class CASLock : Noncopyable
        {
        public:
            /// 局部锁
            using Lock = ScopedLockImpl<CASLock>;
            /**
             * @brief 构造函数
             */
            CASLock()
            {
                m_mutex.clear();
            }

            /**
             * @brief 析构函数
             */
            ~CASLock()
            {
            }

            /**
             * @brief 上锁
             */
            void lock()
            {
                while (std::atomic_flag_test_and_set_explicit(&m_mutex, std::memory_order_acquire))
                    ;
            }

            /**
             * @brief 解锁
             */
            void unlock()
            {
                std::atomic_flag_clear_explicit(&m_mutex, std::memory_order_release);
            }

        private:
            /// 原子状态
            volatile std::atomic_flag m_mutex;
        };
    }
}
#endif
