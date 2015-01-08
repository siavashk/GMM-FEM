#include "mas/concurrency/bind_simple.hpp"

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

//template<typename F>
//void_function_wrapper::impl_F<F>::impl_F(F&& f_) :
//        f(std::move(f_)) {
//}

template<typename F>
void void_function_wrapper::impl_F<F>::call() {
    f();
}

template<typename F>
void_function_wrapper::void_function_wrapper(F&& f) :
        impl(new impl_F<F>(std::move(f))) {
}

template<typename FunctionType>
std::future<typename std::result_of<FunctionType()>::type> thread_pool::submit_back(
        FunctionType f) {
    typedef typename std::result_of<FunctionType()>::type result_type;
    std::packaged_task < result_type() > task(std::move(f));
    std::future<result_type> res(task.get_future());
    workQueue.push_back(std::move(task));
    return res;
}

template<typename FunctionType>
std::future<typename std::result_of<FunctionType()>::type> thread_pool::submit_front(
        FunctionType f) {
    typedef typename std::result_of<FunctionType()>::type result_type;
    std::packaged_task < result_type() > task(std::move(f));
    std::future<result_type> res(task.get_future());
    workQueue.push_front(std::move(task));
    return res;
}

} // concurrency
} // mas
