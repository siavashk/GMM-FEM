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

// Can't be copied, only moved, so can hopefully be only
// run in a single thread
class thread_function_wrapper {
private:
	std::atomic_flag done;
	struct tfw_base {
		virtual void invoke() = 0;
	};

	template<typename Fn, typename ... Args>
	struct tfw_impl: tfw_base {
		std::unique_ptr<function_wrapper<Fn(Args...)>> f_;
		std::promise<typename function_wrapper_t<Fn, Args...>::result_type> p_;
	public:
		tfw_impl(std::unique_ptr<function_wrapper<Fn(Args...)>>&& f,
				std::promise<
						typename function_wrapper_t<Fn, Args...>::result_type>&& p) :
				f_(std::move(f)), p_(std::move(p)) {
		}
		void invoke() {
			p_.set_value(f_->invoke());
		}
	};

	std::unique_ptr<tfw_base> f_;

public:
	template<typename Fn, typename ... Args>
	thread_function_wrapper(std::unique_ptr<function_wrapper<Fn(Args...)>>&& f,
			std::promise<typename function_wrapper_t<Fn, Args...>::result_type>&& p) :
			done(ATOMIC_FLAG_INIT), f_(
					new tfw_impl<Fn, Args...>(std::move(f), std::move(p))) {
	}

	void invoke() {
		if (!done.test_and_set()) {
			f_->invoke();
		}
	}

	thread_function_wrapper(const thread_function_wrapper& f) = delete;
	thread_function_wrapper& operator=(const thread_function_wrapper& f) = delete;

	thread_function_wrapper(thread_function_wrapper&& f) = default;
	thread_function_wrapper& operator=(thread_function_wrapper&& f) = default;
};

class thread_worker {
	std::atomic<bool> running;
	std::atomic<bool> terminated;
	std::mutex wake_mutex;
	std::condition_variable wake_condition;
	std::unique_ptr<thread_function_wrapper> f;

public:
	thread_worker() :
			running(false), f(nullptr) {
	}

	template<typename Fn, typename ... Args>
	bool submit(std::future<typename std::result_of<Fn(Args...)>::type>& fut,
			Fn&& fn, Args&&... args) {
		typedef typename std::result_of<Fn(Args...)>::type result_type;

		std::unique_lock<std::mutex> lk(wake_mutex);
		if (f != nullptr) {
			return false;
		}

		// make function, want to do inside lock to prevent multiple unnecessary allocations
		auto fw = make_unique_wrapper(std::forward<Fn>(fn),
				std::forward<Args>(args)...);
		std::promise<result_type> prom;
		fut = prom.get_future();
		std::unique_ptr<thread_function_wrapper> fp(
				new thread_function_wrapper(std::move(fw), std::move(prom)));

		f = std::move(fp);
		lk.unlock();
		wake_condition.notify_one();

		return true;
	}

	bool isBusy() {
		return running.load();
	}

	void terminate() {
		terminated.store(true);
		wake_condition.notify_all();
	}

	void run() {
		while (!terminated.load()) {
			std::unique_lock<std::mutex> lk(wake_mutex);
			wake_condition.wait(lk, [&] {
				return (f != nullptr || terminated.load());
			});
			running.store(true);
			std::unique_ptr<thread_function_wrapper> myf = std::move(f);
			lk.unlock();

			// run function
			if (myf != nullptr) {
				myf->invoke();
			}
			running.store(false);
		}
	}

	void operator()() {
		run();
	}
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

	thread_pool(int nthreads = 0);
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

class async_thread_pool {
	std::vector<std::shared_ptr<thread_worker>> workers;
	std::vector<std::thread> threads;
	thread_group<std::vector<std::thread>> joiner;

public:
	async_thread_pool(int nthreads) :
			joiner(threads) {

		if (nthreads < 0) {
			nthreads = std::max(std::thread::hardware_concurrency(), 2u);
		}

		try {
			workers.reserve(nthreads);
			threads.reserve(nthreads);
			for (unsigned i = 0; i < nthreads; ++i) {
				workers.push_back(std::make_shared<thread_worker>());
				threads.push_back(std::thread(&thread_worker::run, workers.back()));
			}
		} catch (...) {
			throw;
		}
	}

	~async_thread_pool() {
		for (auto& worker : workers) {
			worker->terminate();
		}
	}

	template<typename Fn, typename ...Args>
	std::future<typename std::result_of<Fn(Args...)>::type> async(Fn&& fn,
			Args&&... args) {
		typedef typename std::result_of<Fn(Args...)>::type result_type;

		std::future<result_type> fut;

		bool asynced = false;
		// find a free worker?
		for (auto& worker : workers) {
			if (!worker->isBusy()) {
				if (worker->submit(fut, std::forward<Fn>(fn),
						std::forward<Args>(args)...)) {
					asynced = true;
					break;
				}
			}
		}

		// otherwise, run now
		if (!asynced) {
			std::promise<result_type> prom;
			fut = prom.get_future();
			prom.set_value(
					invoke(std::forward<Fn>(fn), std::forward<Args>(args)...));
		}
		return fut;
	}

};

} // concurrency
} // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
