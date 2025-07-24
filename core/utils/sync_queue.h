#ifndef ROBSENSE_LIDAR_SYNC_QUEUE_HPP
#define ROBSENSE_LIDAR_SYNC_QUEUE_HPP

#include <mutex>
#include <condition_variable>
#include <vector>
#include <atomic>
#include <utility>
#include <stdexcept>
#include <functional>

namespace robosense
{
namespace lidar
{
template <typename T>
class SyncQueue
{
public:
  using ReleaseFunc = std::function<void(T&&)>;

  explicit SyncQueue(size_t capacity, ReleaseFunc release_func = nullptr)
    : capacity_(capacity > 0 ? capacity : throw std::invalid_argument("Capacity must be greater than 0"))
    , buffer_(capacity_)
    , head_(0)
    , tail_(0)
    , size_(0)
    , stopped_(false)
    , release_func_(std::move(release_func))
  {
  }

  // Delete copy and assignment
  SyncQueue(const SyncQueue&) = delete;
  SyncQueue& operator=(const SyncQueue&) = delete;

  ~SyncQueue()
  {
    stopWait();
    reset();
  }

  // Push with overwrite flag
  size_t push(const T& value, bool& out_overwritten)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    return pushImpl(value, out_overwritten, lock);
  }

  size_t push(T&& value, bool& out_overwritten)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    return pushImpl(std::move(value), out_overwritten, lock);
  }

  // Non-blocking pop
  bool tryPop(T& value)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (stopped_ || size_ == 0)
    {
      return false;
    }

    value = std::move(buffer_[head_]);
    head_ = advance(head_);
    --size_;
    return true;
  }

  // Blocking pop
  bool pop(T& value)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return size_ > 0 || stopped_; });

    if (stopped_)
    {
      return false;
    }

    value = std::move(buffer_[head_]);
    head_ = advance(head_);
    --size_;
    return true;
  }

  // Timed pop
  bool popWait(T& value, uint32_t usec = 1000000)
  {
    std::unique_lock<std::mutex> lock(mtx_);

    if (!cv_.wait_for(lock, std::chrono::microseconds(usec), [this] { return size_ > 0 || stopped_; }))
    {
      return false;  // Timeout
    }

    if (stopped_)
    {
      return false;
    }

    value = std::move(buffer_[head_]);
    head_ = advance(head_);
    --size_;
    return true;
  }

  // Stop all waiting operations
  void stopWait()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!stopped_)
    {
      stopped_ = true;
      cv_.notify_all();
    }
  }

  // Clear the queue
  void clear()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    head_ = 0;
    tail_ = 0;
    size_ = 0;
    stopped_ = false;
    cv_.notify_all();
  }

  // Check if empty
  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return size_ == 0;
  }

  // Get current size
  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return size_;
  }

  // Get capacity
  size_t capacity() const noexcept
  {
    return capacity_;
  }

  // Reset with the predefined release function
  void reset()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (release_func_)
    {
      while (size_ > 0)
      {
        release_func_(std::move(buffer_[head_]));
        head_ = advance(head_);
        --size_;
      }
    }
    else
    {
      head_ = 0;
      tail_ = 0;
      size_ = 0;
    }
  }

private:
  // Helper to advance index with wrap-around
  size_t advance(size_t index) const noexcept
  {
    return (index + 1) % capacity_;
  }

  // Push implementation (simple)
  template <typename U>
  size_t pushImpl(U&& value, bool& out_overwritten, std::unique_lock<std::mutex>& lock)
  {
    if (stopped_)
    {
      out_overwritten = false;
      return 0;
    }

    const bool was_empty = (size_ == 0);
    out_overwritten = (size_ == capacity_);

    if (out_overwritten)
    {
      if (release_func_)
      {
        release_func_(std::move(buffer_[head_]));
      }
      head_ = advance(head_);
      --size_;
    }

    buffer_[tail_] = std::forward<U>(value);
    tail_ = advance(tail_);
    ++size_;

    lock.unlock();

    if (was_empty)
    {
      cv_.notify_one();
    }

    return size_;
  }

  const size_t capacity_;
  std::vector<T> buffer_;
  size_t head_;
  size_t tail_;
  size_t size_;
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  bool stopped_;
  ReleaseFunc release_func_;
};

}  // namespace lidar
}  // namespace robosense

#endif  // ROBOSENSE_LIDAR_SYNC_QUEUE_HPP