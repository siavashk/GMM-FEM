/*
 * thread
 *
 *  Created on: Jan 2, 2015
 *      Author: antonio
 */

#ifndef MAS_CONCURRENCY_THREAD_H_
#define MAS_CONCURRENCY_THREAD_H_

#include "mas/concurrency/queue.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <future>
#include <vector>

namespace mas {
namespace concurrency {

/**
 * A class to automatically clean up after a group of threads,
 * joining any last threads before destruction
 */
template<typename Container>
class thread_group {
private:
    Container& c;
public:
    explicit thread_group(Container& threads_);
    ~thread_group();

};

/**
 * A barrier that moves up, allowing threads to wait
 * or poll until a minimum value is reached.
 */
class rolling_barrier {
private:
    std::vector<size_t> vals;
    std::atomic<bool> terminate;

public:
    rolling_barrier(size_t nthreads);
    rolling_barrier(const rolling_barrier& rb) = delete;
    rolling_barrier& operator=(const rolling_barrier& rb) = delete;

    void set(size_t idx, size_t val);
    bool poll(size_t idx, size_t val);
    bool wait(size_t idx, size_t val);

    ~rolling_barrier(); // force all to stop waiting?
};

class void_function_wrapper {
private:
    struct impl_type {
        virtual void call() = 0;
    };
    template<typename F>
    struct impl_F : impl_type {
        F f;
        impl_F(F&& f_) : f(std::move(f_)){};
        void call();
    };

    std::unique_ptr<impl_type> impl;

public:
    template<typename F>
    void_function_wrapper(F&& f);
    void_function_wrapper(void_function_wrapper&& f);
    void_function_wrapper() = default;
    void_function_wrapper(const void_function_wrapper&)=delete;
    void_function_wrapper(void_function_wrapper&)=delete;

    void_function_wrapper& operator=(void_function_wrapper&& f);
    void_function_wrapper& operator=(const void_function_wrapper&)=delete;

    void operator() ();
};

/**
 * A pool of threads
 */
class thread_pool {
private:
    std::atomic<bool> done;
    threadsafe_queue<void_function_wrapper> workQueue;
    std::vector<std::thread> threads;
    thread_group<decltype(threads)> threadJoiner;

    void doWork();

public:
    thread_pool(const thread_pool&) = delete;
    thread_pool(thread_pool&&) = delete;
    thread_pool& operator = (const thread_pool&) = delete;
    thread_pool& operator = (thread_pool&&) = delete;

    thread_pool(size_t nthreads = 0);
    ~thread_pool();
    void terminate();

    template<typename FunctionType>
    std::future<typename std::result_of<FunctionType()>::type> submit_back(FunctionType f);

    template<typename FunctionType>
    std::future<typename std::result_of<FunctionType()>::type> submit_front(FunctionType f);

    void run_pending_task();
    //    template<typename FunctionType, typename... Args>
    //    std::future<typename std::result_of<FunctionType()>::type> submit(FunctionType&& f, Args&&... args);
};

} // concurrency
} // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
