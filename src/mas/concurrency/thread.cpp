#include "mas/concurrency/thread.h"

namespace mas {
namespace concurrency {

rolling_barrier::rolling_barrier(size_t nthreads) {

    vals = std::vector<size_t>(nthreads, 0);
    terminate.store(false);
}

void rolling_barrier::set(size_t idx, size_t val) {
    // sort and set?
    vals[idx] = val;

//    // potentially notify anyone who is waiting
//    {
//        std::lock_guard < std::mutex > lk(mutex);
//        cv.notify_all();
//    }
}

bool rolling_barrier::poll(size_t idx, size_t val) {
    for (int i = 0; i < vals.size(); i++) {
        if (i != idx && vals[i] <= val) {
            return false;
        }
    }

    return true;
}

bool rolling_barrier::wait(size_t idx, size_t val) {

//    auto barrier_met = [&] {
//        if (terminate.load()) {
//            return true;
//        }
//
//        for (int i=0; i<vals.size(); i++) {
//            if (i != idx && vals[i] <= val) {
//                return false;
//            }
//        }
//        return true;};
//
//    std::unique_lock < std::mutex > lk(mutex);
//    cv.wait(lk, barrier_met);
    while (!poll(idx, val) && !terminate.load()) {
        std::this_thread::yield();
    }

    return !terminate.load();
}

rolling_barrier::~rolling_barrier() {
    terminate.store(true);
    //cv.notify_all();
}

void void_function_wrapper::operator()() {
    impl->call();
}

void_function_wrapper::void_function_wrapper(void_function_wrapper&& f) :
        impl(std::move(f.impl)) {
}

void_function_wrapper& void_function_wrapper::operator=(
        void_function_wrapper&& f) {
    impl = std::move(f.impl);
    return *this;
}

void thread_pool::doWork() {
    while (!done) {
        void_function_wrapper task;
        if (workQueue.pop(task)) {
            task();
        } else {
            std::this_thread::yield();
        }
    }
}

thread_pool::thread_pool(size_t nthreads) :
        done(false), threadJoiner(threads) {
    if (nthreads == 0) {
        nthreads = std::thread::hardware_concurrency();
    }

    try {
        for (size_t i = 0; i < nthreads; i++) {
            threads.push_back(std::thread(&thread_pool::doWork, this));
        }
    } catch (...) {
        done = true;
        throw;
    }
}

thread_pool::~thread_pool() {
    done = true;
}

void thread_pool::terminate() {
    done = true;
}

void thread_pool::run_pending_task() {
    void_function_wrapper task;
    if (workQueue.pop(task)) {
        task();
    } else {
        std::this_thread::yield();
    }
}

}
}
