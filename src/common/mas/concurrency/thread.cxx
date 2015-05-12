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



void thread_pool::doWork() {
	std::unique_ptr<thread_function_wrapper> task;
    while (!done) {
        if (workQueue.pop(task)) {
        	task->invoke();
        } else {
            std::this_thread::yield();
        }
    }
}

thread_pool::thread_pool(int nthreads) :
		done(false), threads(), threadJoiner(threads) {
    if (nthreads <= 0) {
        nthreads = std::max(std::thread::hardware_concurrency(), 2u);
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

bool thread_pool::run_pending_task() {
	if (!done) {
		std::unique_ptr<thread_function_wrapper> task;
		if (workQueue.pop(task)) {
			task->invoke();
			return true;
		} else {
			std::this_thread::yield();
			return false;
		}
	}
	return false; // no task run
}

}
}
