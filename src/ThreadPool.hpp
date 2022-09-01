#pragma once
#include <cstdint>
#include <thread>
#include <vector>
#include <functional>

namespace sb {

class ThreadPool {
public:
    using Task = std::function<void()>;
    ThreadPool(uint32_t threads = std::thread::hardware_concurrency());
    ~ThreadPool();

    void post_task(Task&& task);

private:
    enum class ThreadState {
        Alive,
        ShouldDie,
        Dead,
    };
    std::vector<ThreadState> states {};
    std::vector<Task> mailboxes {};
    std::vector<std::thread> thread_pool {};
};

}
