#include <ThreadPool.hpp>
#include <thread>

namespace sb {

ThreadPool::ThreadPool(uint32_t threads)
{
    states.reserve(threads);
    mailboxes.reserve(threads);
    thread_pool.reserve(threads);
    for (uint32_t thread_id = 0; thread_id < threads; thread_id++) {
        auto thread_function = [this, thread_id] {
            auto& state = states[thread_id];
            auto& mailbox = mailboxes[thread_id];
            state = ThreadState::Alive;
            while(state != ThreadState::ShouldDie) {
                if (mailbox) {
                    mailbox();
                    mailbox = nullptr;
                }
            }
            state = ThreadState::Dead;
        };
        thread_pool.push_back(std::thread(thread_function));
    }
}

ThreadPool::~ThreadPool()
{
    for (auto& state : states)
        state = ThreadState::ShouldDie;
    for (auto& state : states)
        while(state != ThreadState::Dead);
}

void ThreadPool::post_task(Task&& task)
{
    while (1) {
        for (size_t i = 0; i < mailboxes.size(); i++) {
            if (!mailboxes[i]) {
                mailboxes[i] = task;
                return;
            }
        }
    }
}

}
