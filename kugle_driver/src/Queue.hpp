#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
 
template <typename T>
class Queue
{
 public:
 
  T pop()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    auto item = queue_.front();
    queue_.pop();
    return item;
  }

  bool pop(T& item, float timeout_seconds) {
    std::unique_lock<std::mutex> mlock(mutex_);
    if (queue_.empty()) {
      if (cond_.wait_for(mlock, std::chrono::milliseconds(int64_t(1000*timeout_seconds))) != std::cv_status::timeout) {
        item = queue_.front();
        queue_.pop();
        return true;
      } else {
        return false; // timeout
      }
    } else { // element already available
      item = queue_.front();
      queue_.pop();
      return true;
    }
  }

  void pop(T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = queue_.front();
    queue_.pop();
  }
 
  void push(const T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }
 
  void push(T&& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(std::move(item));
    mlock.unlock();
    cond_.notify_one();
  }

  void clear()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    std::queue<T> empty;
    std::swap( queue_, empty );
    mlock.unlock();
  }
 
 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};
