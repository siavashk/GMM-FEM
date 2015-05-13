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
    bool done;
    struct tfw_base {
        virtual void invoke() = 0;
    };

    template<typename Fn, typename ... Args>
    struct tfw_impl: tfw_base {
        std::unique_ptr<function_wrapper<Fn(Args...)>> f_;
        promise<typename function_wrapper_t<Fn, Args...>::result_type> p_;
    public:
        tfw_impl(std::unique_ptr<function_wrapper<Fn(Args...)>>&& f,
                promise<typename function_wrapper_t<Fn, Args...>::result_type> && p) :
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
            promise<typename function_wrapper_t<Fn, Args...>::result_type> && p) :
            done(false), f_(
                    new tfw_impl<Fn, Args...>(std::move(f), std::move(p))) {
    }

    void invoke() {
        if (!done) {
            f_->invoke();
            done = true;
        }
    }

    void operator()() {
        invoke();
    }

    thread_function_wrapper(const thread_function_wrapper& f) = delete;
    thread_function_wrapper& operator=(const thread_function_wrapper& f) = delete;

    // thread_function_wrapper(thread_function_wrapper&& f) = default;
	thread_function_wrapper(thread_function_wrapper&& f) :
		done(f.done), f_(std::move(f.f_)) {}
    // thread_function_wrapper& operator=(thread_function_wrapper&& f) = default;
    thread_function_wrapper& operator=(thread_function_wrapper&& f) {
        this->done = f.done;
        this->f_ = std::move(f.f_);
        return *this;
    }
};

class async_thread_worker {
    std::atomic_flag reserve_flag;
    std::atomic<bool> terminated;
    std::mutex wake_mutex;
    std::condition_variable wake_condition;
    std::unique_ptr<thread_function_wrapper> f;

public:
    async_thread_worker() :
            //reserve_flag(ATOMIC_FLAG_INIT), terminated(false), f(nullptr) {
			reserve_flag(), terminated(false), f(nullptr) {
    }

    template<typename Fn, typename ... Args>
    future<typename std::result_of<Fn(Args...)>::type> submit(Fn&& fn,
            Args&&... args) {

        typedef typename std::result_of<Fn(Args...)>::type result_type;

        if (!reserve_flag.test_and_set()) {
            throw "worker not reserved";
        }
        // make function, want to do inside lock to prevent multiple unnecessary allocations
        auto fw = make_unique_wrapper(std::forward<Fn>(fn),
                std::forward<Args>(args)...);
        promise<result_type> prom;
        auto fut = prom.get_future();
        std::unique_ptr<thread_function_wrapper> fp(
                new thread_function_wrapper(std::move(fw), std::move(prom)));

        // only want to actually set if f is nullptr
        std::unique_lock<std::mutex> lk(wake_mutex);
        if (f != nullptr) {
            // run pending task now
            f->invoke();
            f = nullptr;
        }
        f = std::move(fp);
        lk.unlock();

        wake_condition.notify_one();

        return fut;
    }

    // separated so we can test rather than forward arguments and have it fail
    bool reserve_worker() {
        return (!reserve_flag.test_and_set());
    }

    void free_worker() {
        reserve_flag.clear();
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
            std::unique_ptr<thread_function_wrapper> myf = std::move(f);
            lk.unlock();

            // run function
            if (myf != nullptr) {
                myf->invoke();
            }
            reserve_flag.clear();  // available to reserve again
        }
    }

    void operator()() {
        run();
    }
};

template<typename ... Signature>
class simple_async_thread_worker {
};

template<typename ... Fn, typename ... Args>
class simple_async_thread_worker<std::tuple<Fn...>, std::tuple<Args...>> {

    typedef typename function_wrapper_t<Fn..., Args...>::result_type result_type;
    typedef simple_promise<std::tuple<Fn...>, std::tuple<Args...>> promise_type;
    typedef typename build_index_tuple<sizeof...(Fn)>::type IndicesFn;
    typedef typename build_index_tuple<sizeof...(Args)>::type IndicesArgs;

    // bare-bones: function is fixed (fixed args only),
    std::tuple<Fn...> fixed;
    std::unique_ptr<std::tuple<Args...>> vargs;
    std::unique_ptr<promise_type> prom;

    std::atomic_flag reserve_flag;
    std::atomic<bool> terminated;
    std::mutex wake_mutex;
    std::condition_variable wake_condition;


private:

    template<std::size_t ... IndicesFn, std::size_t ... IndicesArgs>
        result_type invoke(std::tuple<Args...>&& args,
                index_tuple<IndicesFn...>, index_tuple<IndicesArgs...>) {

            return mas::concurrency::invoke(
                    std::forward<Fn>(std::get<IndicesFn>(fixed))...,
                    std::forward<Args>(std::get<IndicesArgs>(args))...);
        }

    void invoke(std::unique_ptr<promise_type>&& prom, std::unique_ptr<std::tuple<Args...>>&& args) {
        prom->set_value( invoke(std::forward<std::tuple<Args...>>(*args), IndicesFn(), IndicesArgs()));
    }

public:
    simple_async_thread_worker(std::tuple<Fn...> fixed) :
        fixed(std::move(fixed)), vargs(nullptr), prom(nullptr), reserve_flag(ATOMIC_FLAG_INIT), terminated(false) {
    }

    simple_future<std::tuple<Fn...>, std::tuple<Args...>> submit(Args&&... args) {

        if (!reserve_flag.test_and_set()) {
            throw "worker not reserved";
        }

        // only want to actually set if f is nullptr
        std::unique_lock<std::mutex> lk(wake_mutex);
        if(prom != nullptr) {
            // run pending task now
            invoke(std::move(prom), std::move(vargs));
        }
        prom = std::unique_ptr<promise_type>(new promise_type());
        vargs = std::unique_ptr<std::tuple<Args...>>(new std::tuple<Args...>(std::forward<Args>(args)...));
        auto fut = prom->get_future();
        lk.unlock(); // promise might now get gobbled up, which is why we needed it first

        wake_condition.notify_one();

        return fut;

    }

    // separated so we can test rather than forward arguments and have it fail
    bool reserve_worker() {
        return (!reserve_flag.test_and_set());
    }

    void free_worker() {
        reserve_flag.clear();
    }

    void terminate() {
        terminated.store(true);
        wake_condition.notify_all();
    }

    void run() {
        while (!terminated.load()) {
            std::unique_lock<std::mutex> lk(wake_mutex);
            wake_condition.wait(lk, [&] {
                        return (prom != nullptr || terminated.load());
                    });

            if (prom != nullptr) {
                invoke(std::move(prom), std::move(vargs));
                prom = nullptr;        // signal that it has been used
            }
            lk.unlock();
            reserve_flag.clear();  // available to reserve again
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
    future<typename std::result_of<_Fn(_Args...)>::type> submit_back(_Fn&& __fn,
            _Args&&... __args);

    template<typename _Fn, typename ... _Args>
    future<typename std::result_of<_Fn(_Args...)>::type> submit_front(
            _Fn&& __fn, _Args&&... __args);

    bool run_pending_task();

};

class async_thread_pool {
    std::vector<std::shared_ptr<async_thread_worker>> workers;
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
            for (int i = 0; i < nthreads; ++i) {
                workers.push_back(std::make_shared<async_thread_worker>());
                threads.push_back(
                        std::thread(&async_thread_worker::run, workers.back()));
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
    inline future<typename std::result_of<Fn(Args...)>::type> async(Fn&& fn,
            Args&&... args) {
        return async(std::launch::async | std::launch::deferred,
                std::forward<Fn>(fn), std::forward<Args>(args)...);
    }

    /**
     * nullptr if none available
     * @return
     */
    async_thread_worker* reserve_thread() {
        for (auto& worker : workers) {
            if (worker->reserve_worker()) {
                return worker.get();
            }
        }
        return nullptr;
    }

    // nullptr if can't schedule asynchronously
    template<typename Fn, typename ...Args>
    std::unique_ptr<future<typename std::result_of<Fn(Args...)>::type>> maybe_async(
                Fn&& fn, Args&&... args) {
        // find a free worker
        for (auto& worker : workers) {
            if (worker->reserve_worker()) {
                auto fut = worker->submit(std::forward<Fn>(fn),
                        std::forward<Args>(args)...);
                return fut;
            }
        }
        return nullptr;
    }

    template<typename Fn, typename ...Args>
    future<typename std::result_of<Fn(Args...)>::type> async(std::launch launch,
            Fn&& fn, Args&&... args) {
        typedef typename std::result_of<Fn(Args...)>::type result_type;

        if (launch == (std::launch::deferred | std::launch::async)) {
            // find a free worker?
            for (auto& worker : workers) {
                if (worker->reserve_worker()) {
                    auto fut = worker->submit(std::forward<Fn>(fn),
                            std::forward<Args>(args)...);
                    return fut;
                }
            }
        } else if (launch == std::launch::async) {
            // spin off new thread
            auto fw = make_unique_wrapper(std::forward<Fn>(fn),
                    std::forward<Args>(args)...);
            promise<result_type> prom;
            future<result_type> fut = prom.get_future();
            thread_function_wrapper wrapped(std::move(fw), std::move(prom));

            std::thread async_thread(std::move(wrapped));
            async_thread.detach();
            return fut;
        }

        // otherwise, run now
        //if (!asynced) {
        std::unique_ptr<typename function_wrapper_t<Fn, Args...>::type> f =
                make_unique_wrapper(std::forward<Fn>(fn),
                        std::forward<Args>(args)...);
        std::unique_ptr<deferred_function<Fn, Args...>> dfp(
                new deferred_function<Fn, Args...>(std::move(f)));
        return future<result_type>(std::move(dfp));
        //}
    }

    template<typename Fn, typename ...Args>
    static future<typename std::result_of<Fn(Args...)>::type> launch(
            std::launch policy, Fn&& fn, Args&&... args) {

        typedef typename std::result_of<Fn&(Args&&...)>::type result_type;

        if (policy == std::launch::async) {
            // spin off new thread
            auto fw = make_unique_wrapper(std::forward<Fn>(fn),
                    std::forward<Args>(args)...);
            promise<result_type> prom;
            future<result_type> fut = prom.get_future();
            thread_function_wrapper wrapped(std::move(fw), std::move(prom));

            std::thread async_thread(std::move(wrapped));
            async_thread.detach();
            return fut;
        } else {
            std::unique_ptr<typename function_wrapper_t<Fn, Args...>::type> f =
                    make_unique_wrapper(std::forward<Fn>(fn),
                            std::forward<Args>(args)...);
            std::unique_ptr<deferred_function<Fn, Args...>> dfp(
                    new deferred_function<Fn, Args...>(std::move(f)));
            auto fut = future<result_type>(std::move(dfp));
            return fut;
        }
    }

};

template<typename ... Signature>
class simple_async_thread_pool {
};

template<typename ... Fn, typename ... Args>
class simple_async_thread_pool<std::tuple<Fn...>, std::tuple<Args...>> {
    typedef simple_async_thread_worker<std::tuple<Fn...>, std::tuple<Args...>> worker_type;
    typedef typename function_wrapper_t<Fn..., Args...>::result_type result_type;

    std::vector<std::shared_ptr<worker_type>> workers;
    std::vector<std::thread> threads;
    thread_group<std::vector<std::thread>> joiner;
    std::tuple<Fn...> fixed;

public:

    simple_async_thread_pool(int nthreads, Fn... fn) :
    joiner(threads), fixed(std::forward<Fn>(fn)...) {

        if (nthreads < 0) {
            nthreads = std::max(std::thread::hardware_concurrency(), 2u);
        }

        try {
            workers.reserve(nthreads);
            threads.reserve(nthreads);
            for (int i = 0; i < nthreads; ++i) {
                workers.push_back(std::make_shared<worker_type>(fixed));
                threads.push_back(
                std::thread(&worker_type::run, workers.back()));
            }
        } catch (...) {
            throw;
        }
    }

    simple_async_thread_pool(Fn... fn) :
    simple_async_thread_pool(-1, std::forward<Fn>(fn)...) {}

    ~simple_async_thread_pool() {
        for (auto& worker : workers) {
            worker->terminate();
        }
    }

    inline simple_future<std::tuple<Fn...>, std::tuple<Args...>> async(Args&&... args) {
        return async(std::launch::async | std::launch::deferred, std::forward<Args>(args)...);
    }

    template<std::size_t ... IndicesFn, std::size_t ... IndicesArgs>
    result_type invoke(std::tuple<Args...>&& args,
            index_tuple<IndicesFn...>, index_tuple<IndicesArgs...>) {

        return mas::concurrency::invoke(
                std::forward<Fn>(std::get<IndicesFn>(fixed))...,
                std::forward<Args>(std::get<IndicesArgs>(args))...);
    }

    simple_future<std::tuple<Fn...>, std::tuple<Args...>> async(std::launch launch, Args&&... args) {


        if (launch == (std::launch::deferred | std::launch::async)) {
            // find a free worker?
            for (auto& worker : workers) {
                if (worker->reserve_worker()) {
                    return worker->submit(std::forward<Args>(args)...);
                }
            }
        } else if (launch == std::launch::async) {

            simple_promise<std::tuple<Fn...>, std::tuple<Args...>> prom;
            auto fut = prom.get_future();

            // spin off new thread
            auto fun = [&](simple_promise<std::tuple<Fn...>, std::tuple<Args...>>&& prom,
                    std::tuple<Fn...> fixed, std::tuple<Args...>&& args) {
                typedef typename build_index_tuple<sizeof...(Fn)>::type IndicesFn;
                typedef typename build_index_tuple<sizeof...(Args)>::type IndicesArgs;



                prom.set_value(
                        invoke( std::forward<std::tuple<Args...>>(args),
                                IndicesFn(), IndicesArgs())
                        );
            };

            std::thread async_thread(fun, std::move(prom), fixed, std::forward_as_tuple(std::forward<Args>(args)...));
            async_thread.detach();
            return fut;
        }

        // otherwise, run now
        return simple_future<std::tuple<Fn...>, std::tuple<Args...>>(fixed, std::forward_as_tuple(std::forward<Args>(args)...));

    }

};

} // concurrency
}                                            // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
