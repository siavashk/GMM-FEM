/*
 * thread
 *
 *  Created on: Jan 2, 2015
 *      Author: antonio
 */

#ifndef MAS_CONCURRENCY_THREAD_H_
#define MAS_CONCURRENCY_THREAD_H_

#include "mas/concurrency/queue.h"
#include "mas/concurrency/function.hpp"

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

	void set(size_t idx, size_t val);bool poll(size_t idx, size_t val);bool wait(
			size_t idx, size_t val);

	~rolling_barrier(); // force all to stop waiting?
};

class thread_function_wrapper {
private:
	struct tfw_base {
		virtual void invoke() = 0;
	};

	template<typename Fn, typename ... Args>
	struct tfw_impl: tfw_base {
		future_function_wrapper<Fn, Args...> f_;
	public:
		tfw_impl(future_function_wrapper<Fn, Args...> && f) :
				f_(std::move(f)) {
		}
		void invoke() {
			f_.invoke();
		}
	};

	std::unique_ptr<tfw_base> f_;
public:
	template<typename Fn, typename ... Args>
	thread_function_wrapper(future_function_wrapper<Fn, Args...> && f) :
			f_(new tfw_impl<Fn,Args...>(std::move(f))) {
	}

	void invoke() {
		f_->invoke();
	}

	thread_function_wrapper(const thread_function_wrapper& f) = delete;
	thread_function_wrapper& operator=(const thread_function_wrapper& f) = delete;

	thread_function_wrapper(thread_function_wrapper&& f) = default;
	thread_function_wrapper& operator=(thread_function_wrapper&& f) = default;
};

/**
 * A pool of threads
 */
class thread_pool {
private:
	threadsafe_queue<std::unique_ptr<thread_function_wrapper>> workQueue;
	std::atomic<bool> done;
	std::vector<std::thread> threads;
	thread_group<decltype(threads)> threadJoiner;

	void doWork();

public:
	thread_pool(const thread_pool&) = delete;
	thread_pool(thread_pool&&) = delete;
	thread_pool& operator =(const thread_pool&) = delete;
	thread_pool& operator =(thread_pool&&) = delete;

	thread_pool(size_t nthreads = 0);
	~thread_pool();
	void terminate();

	template<typename _Fn, typename ... _Args>
	std::future<typename std::result_of<_Fn(_Args...)>::type> submit_back(
			_Fn&& __fn, _Args&&... __args);

	template<typename _Fn, typename ... _Args>
	std::future<typename std::result_of<_Fn(_Args...)>::type> submit_front(
			_Fn&& __fn, _Args&&... __args);

	bool run_pending_task();

};

class simple_thread_pool {
	std::atomic_bool done;
	threadsafe_queue<std::unique_ptr<thread_function_wrapper>> work_queue;
	std::vector<std::thread> threads;
	thread_group<std::vector<std::thread>> joiner;

	void worker_thread() {
		while (!done) {
			std::unique_ptr<thread_function_wrapper> task;
			if (work_queue.pop(task)) {
				task->invoke();
			} else {
				std::this_thread::yield();
			}
		}
	}

public:
	simple_thread_pool() :
			done(false), joiner(threads) {
		unsigned const thread_count = std::thread::hardware_concurrency();
		try {
			for (unsigned i = 0; i < thread_count; ++i) {
				threads.push_back(
						std::thread(&simple_thread_pool::worker_thread, this));
			}
		} catch (...) {
			done = true;
			throw;
		}
	}

	~simple_thread_pool() {
		done = true;
	}

	template<typename Fn, typename ...Args>
	std::future<typename std::result_of<Fn(Args...)>::type> submit_back(
			const Fn& f, Args&&... args) {
		typedef typename std::result_of<Fn(Args...)>::type result_type;

		auto fw = wrap_future(std::ref(f), std::forward<Args>(args)...);
		std::future<result_type> fut = fw.get_future();
		std::unique_ptr<thread_function_wrapper> fp(
				new thread_function_wrapper(std::move(fw)));

		work_queue.push_back(std::move(fp));
		return fut;
	}

	template<typename Fn, typename ...Args>
	std::future<typename std::result_of<Fn(Args...)>::type> submit_back(Fn&& f,
			Args&&... args) {
		typedef typename std::result_of<Fn(Args...)>::type result_type;

		auto fw = wrap_future(std::ref(f), std::forward<Args>(args)...);
		std::future<result_type> fut = fw.get_future();
		std::unique_ptr<thread_function_wrapper> fp(
				new thread_function_wrapper(std::move(fw)));

		work_queue.push_back(std::move(fp));
		return fut;
	}

	void run_pending_task() {
		if (!done) {
			std::unique_ptr<thread_function_wrapper> task;
			if (work_queue.pop(task)) {
				task->invoke();
			} else {
				std::this_thread::yield();
			}
		}
	}
};

} // concurrency
} // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
