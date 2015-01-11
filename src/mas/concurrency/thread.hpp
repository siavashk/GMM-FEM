#include "mas/concurrency/bind_simple.hpp"
#include <future>

namespace mas {
namespace concurrency {

template<typename Container>
thread_group<Container>::thread_group(Container& threads) :
		c(threads) {
}

template<typename Container>
thread_group<Container>::~thread_group() {
	for (auto t = begin(c); t < end(c); t++) {
		if (t->joinable()) {
			t->join();
		}
	}
}

template<typename Fn, typename ... Args>
std::future<typename std::result_of<Fn(Args...)>::type> thread_pool::submit_back(
		Fn&& fn, Args&&... args) {

	typedef typename std::result_of<Fn(Args...)>::type result_type;

	auto fw = wrap_future(std::ref(fn), std::forward<Args>(args)...);
	std::future<result_type> fut = fw.get_future();
	std::unique_ptr<thread_function_wrapper> fp(new thread_function_wrapper(std::move(fw)));

	workQueue.push_back(std::move(fp));
	return fut;
}

template<typename Fn, typename ... Args>
std::future<typename std::result_of<Fn(Args...)>::type> thread_pool::submit_front(
		Fn&& fn, Args&&... args) {
	typedef typename std::result_of<Fn(Args...)>::type result_type;

	auto fw = wrap_future(std::ref(fn), std::forward<Args>(args)...);
	std::future<result_type> fut = fw.get_future();
	std::unique_ptr<thread_function_wrapper> fp(new thread_function_wrapper(std::move(fw)));

	workQueue.push_front(std::move(fp));
	return fut;
}

} // concurrency
} // mas
