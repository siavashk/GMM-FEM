/*
 * thread
 *
 *  Created on: Jan 2, 2015
 *      Author: antonio
 */

#ifndef MAS_CONCURRENCY_THREAD_H_
#define MAS_CONCURRENCY_THREAD_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
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
    std::mutex mutex;
    std::condition_variable cv;

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

} // concurrency
} // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
