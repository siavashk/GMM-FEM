#include <algorithm>
#include <atomic>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

// #include "../../../src/mas/concurrency/function.hpp"
#include "mas/concurrency/thread.h"
#include "mas/core/time.h"

using namespace mas::concurrency;

template<typename T>
void print_vec(const T& t) {
    int N = t.size();
    if (N <= 10) {
        for (int i = 0; i < N; i++) {
            std::cout << t[i] << " ";
        }
        std::cout << std::endl;
    }
}

template<typename T>
class tracked_vector {
    static std::atomic<size_t> default_construct_count;
    static std::atomic<size_t> copy_construct_count;
    static std::atomic<size_t> move_construct_count;
    static std::atomic<size_t> copy_assign_count;
    static std::atomic<size_t> move_assign_count;

public:
    std::vector<T> vec;

    tracked_vector() :
            vec() {
        ++default_construct_count;
    }

    tracked_vector(const tracked_vector& cp) :
            vec(cp.vec) {
        ++copy_construct_count;
    }

    tracked_vector(tracked_vector&& mv) :
            vec(std::move(mv.vec)) {
        ++move_construct_count;
    }

    tracked_vector& operator=(const tracked_vector& cp) {
        vec = cp.vec;
        ++copy_assign_count;
        return this;
    }

    tracked_vector& operator=(tracked_vector&& cp) {
        vec = cp.vec;
        ++move_assign_count;
    }

    bool empty() const {
        return vec.empty();
    }

    typename std::vector<T>::size_type size() const {
        return vec.size();
    }

    void reserve(typename std::vector<T>::size_type size) {
        vec.reserve(size);
    }

    void resize(typename std::vector<T>::size_type size) {
        vec.resize(size);
    }

    T& operator[](typename std::vector<T>::size_type idx) {
        return vec[idx];
    }

    const T& operator[](typename std::vector<T>::size_type idx) const {
        return vec[idx];
    }

    T& front() {
        return vec.front();
    }

    T& back() {
        return vec.back();
    }

    void push_back(T val) {
        vec.push_back(val);
    }

    void pop_back() {
        vec.pop_back();
    }

    typename std::vector<T>::iterator begin() {
        return vec.begin();
    }

    typename std::vector<T>::iterator end() {
        return vec.end();
    }

    static void print_counts() {
        std::cout << "Default construct: " << default_construct_count.load()
                << std::endl;
        std::cout << "Copy construct: " << copy_construct_count.load()
                << std::endl;
        std::cout << "Move construct: " << move_construct_count.load()
                << std::endl;
        std::cout << "Copy assign: " << copy_assign_count.load() << std::endl;
        std::cout << "Move assign: " << move_assign_count.load() << std::endl;
    }

    static void reset_counts() {
        default_construct_count.store(0);
        copy_construct_count.store(0);
        move_construct_count.store(0);
        copy_assign_count.store(0);
        move_assign_count.store(0);
    }

};

template<typename T>
std::atomic<size_t> tracked_vector<T>::default_construct_count(0);
template<typename T>
std::atomic<size_t> tracked_vector<T>::copy_construct_count(0);
template<typename T>
std::atomic<size_t> tracked_vector<T>::move_construct_count(0);
template<typename T>
std::atomic<size_t> tracked_vector<T>::copy_assign_count(0);
template<typename T>
std::atomic<size_t> tracked_vector<T>::move_assign_count(0);

template<typename T>
tracked_vector<T> do_quick_sort(tracked_vector<T> chunk_data) {
    if (chunk_data.size() < 2) {
        return chunk_data;
    }

    T middle = chunk_data.front();

    tracked_vector<T> new_lower_chunk = std::move(chunk_data);
    tracked_vector<T> new_upper_chunk;

    // split into an upper and lower chunk
    size_t nlower = 0;
    size_t len = new_lower_chunk.size();
    for (size_t i = 1; i < len; ++i) {
        if (new_lower_chunk[i] > middle) {
            new_upper_chunk.push_back(new_lower_chunk[i]);
        } else {
            new_lower_chunk[nlower++] = new_lower_chunk[i];
        }
    }
    new_lower_chunk.resize(nlower);

    tracked_vector<T> new_higher = do_quick_sort<T>(std::move(new_upper_chunk));

    // mis-direction
//	tracked_vector<T> new_lower = do_quick_sort<T>(std::move(new_lower_chunk));
//	auto vec = std::make_shared<tracked_vector<T>>(std::move(new_lower));
//	tracked_vector<T> result = std::move(*vec);

    tracked_vector<T> result = do_quick_sort<T>(std::move(new_lower_chunk));

    // promise route
//	 std::promise<tracked_vector<T>> pt;
//	 pt.set_value(do_quick_sort<T>(std::move(new_lower_chunk)));
//	 std::future<tracked_vector<T>> new_lower = pt.get_future();
//	 tracked_vector<T> result = new_lower.get();

//	future<tracked_vector<T>> new_lower = async_thread_pool::launch(std::launch::deferred, do_quick_sort<T>, std::move(new_lower_chunk));
//	tracked_vector<T> result = new_lower.get();

//	 promise<tracked_vector<T>> pt;
//	 pt.set_value(do_quick_sort<T>(std::move(new_lower_chunk)));
//	 future<tracked_vector<T>> new_lower = pt.get_future();
//	 tracked_vector<T> result = new_lower.get();

    result.reserve(result.size() + 1 + new_higher.size());
    result.push_back(middle);
    for (size_t i = 0; i < new_higher.size(); i++) {
        result.push_back(new_higher[i]);
    }

    return result;
}

template<typename T>
tracked_vector<T> do_parallel_copy_sort(tracked_vector<T> chunk_data,
        mas::concurrency::thread_pool& pool) {
    if (chunk_data.size() < 2) {
        return chunk_data;
    }

    T middle = chunk_data.front();

    tracked_vector<T> new_lower_chunk = std::move(chunk_data);
    tracked_vector<T> new_upper_chunk;

    // split into an upper and lower chunk
    size_t nlower = 0;
    size_t len = new_lower_chunk.size();
    for (size_t i = 1; i < len; ++i) {
        if (new_lower_chunk[i] > middle) {
            new_upper_chunk.push_back(new_lower_chunk[i]);
        } else {
            new_lower_chunk[nlower++] = new_lower_chunk[i];
        }
    }
    new_lower_chunk.resize(nlower);

    std::future<tracked_vector<T>> new_lower = pool.submit_front(
            do_parallel_copy_sort<T>, std::ref(new_lower_chunk),
            std::ref(pool));
    tracked_vector<T> new_higher = do_parallel_copy_sort<T>(new_upper_chunk,
            pool);

    while (!(new_lower.wait_for(std::chrono::seconds(0))
            == std::future_status::ready)) {
        pool.run_pending_task();
    }

    tracked_vector<T> result = new_lower.get();
    result.reserve(result.size() + 1 + new_higher.size());
    result.push_back(middle);
    for (size_t i = 0; i < new_higher.size(); i++) {
        result.push_back(new_higher[i]);
    }

    return result;
}

std::mutex mut;

template<typename T>
tracked_vector<T> do_parallel_async_copy_sort(tracked_vector<T> chunk_data,
        mas::concurrency::async_thread_pool& pool) { //, int startidx, int endidx, bool spawned) {

//	if (spawned) {
//		std::lock_guard<std::mutex> lk(mut);
//		std::cout << "starting (" << startidx <<  ", " << endidx << ")" << std::endl;
//		print_vec(chunk_data);
//	}

    if (chunk_data.size() < 2) {
        return chunk_data;
    }

    T middle = chunk_data.front();
    tracked_vector<T> new_lower_chunk = std::move(chunk_data);
    tracked_vector<T> new_upper_chunk;

    // split into an upper and lower chunk
    size_t nlower = 0;
    size_t len = new_lower_chunk.size();
    for (size_t i = 1; i < len; ++i) {
        if (new_lower_chunk[i] > middle) {
            new_upper_chunk.push_back(new_lower_chunk[i]);
        } else {
            new_lower_chunk[nlower++] = new_lower_chunk[i];
        }
    }
    new_lower_chunk.resize(nlower);

//      size_t nupper = new_upper_chunk.size();
//	{
//		std::lock_guard<std::mutex> lk(mut);
//		std::cout << "spawning (" << startidx <<  ", " << startidx+nlower-1 << ")" << std::endl;
//		print_vec(new_lower_chunk);
//	}
//	tracked_vector<T> low_cpy = new_lower_chunk;

    auto worker = pool.reserve_thread();
    if (worker != nullptr) {
        // spin off async
        future<tracked_vector<T>> new_lower = worker->submit(do_parallel_async_copy_sort<T>, std::move(new_lower_chunk),
                std::ref(pool));

        tracked_vector<T> new_higher = do_parallel_async_copy_sort<T>(
                std::move(new_upper_chunk), pool); //, startidx+nlower+1, startidx+nlower+nupper, false);

        tracked_vector<T> result = new_lower.get();
        result.reserve(result.size() + 1 + new_higher.size());
        result.push_back(middle);
        for (size_t  i = 0; i < new_higher.size(); i++) {
            result.push_back(new_higher[i]);
        }
        return result;
    }

    // run sequentially
    tracked_vector<T> result = do_parallel_async_copy_sort<T>(
            std::move(new_lower_chunk), pool);
    tracked_vector<T> new_higher = do_parallel_async_copy_sort<T>(
            std::move(new_upper_chunk), pool);

    result.reserve(result.size() + 1 + new_higher.size());
    result.push_back(middle);
    for (size_t i = 0; i < new_higher.size(); i++) {
        result.push_back(new_higher[i]);
    }

    return result;
}

template<typename T>
tracked_vector<T> parallel_async_quick_sort(tracked_vector<T> input,
        size_t  nthreads = -1) {
    if (input.empty()) {
        return input;
    }
    // copy_sorter<T> s;
    if (nthreads < 0) {
        nthreads = std::max(std::thread::hardware_concurrency() - 1, 1u);
    }
    mas::concurrency::async_thread_pool pool(nthreads);

    return do_parallel_async_copy_sort(std::move(input), pool); //, 0, input.size()-1, false);
}

struct simple_async_sorter {

public:

    tracked_vector<int> do_sort(tracked_vector<int> chunk_data, bool lower) {

        if (chunk_data.size() < 2) {
            return chunk_data;
        }

        double middle = chunk_data.front();
        tracked_vector<int> new_lower_chunk = std::move(chunk_data);
        tracked_vector<int> new_upper_chunk;

        // split into an upper and lower chunk
        size_t  nlower = 0;
        size_t  len = new_lower_chunk.size();
        for (size_t  i = 1; i < len; ++i) {
            if (new_lower_chunk[i] > middle) {
                new_upper_chunk.push_back(new_lower_chunk[i]);
            } else {
                new_lower_chunk[nlower++] = new_lower_chunk[i];
            }
        }
        new_lower_chunk.resize(nlower);

        auto new_lower = pool.async(std::move(new_lower_chunk), true);

        tracked_vector<int> new_higher = do_sort(std::move(new_upper_chunk), false);
        tracked_vector<int> result = new_lower.get();

        result.reserve(result.size() + 1 + new_higher.size());
        result.push_back(middle);
        for (size_t i = 0; i < new_higher.size(); i++) {
            result.push_back(new_higher[i]);
        }

        return result;
    }

    typedef tracked_vector<int> vec_type;
    typedef std::tuple<decltype(&simple_async_sorter::do_sort),
            simple_async_sorter*> fn_type;
    typedef std::tuple<vec_type, bool> args_type;
    simple_async_thread_pool<fn_type, args_type> pool;

    simple_async_sorter(int threads) :
        pool (threads, &simple_async_sorter::do_sort, this)
    {}
};

template<typename T>
tracked_vector<T> parallel_simple_async_quick_sort(tracked_vector<T> input,
        int nthreads = -1) {
    if (input.empty()) {
        return input;
    }
    // copy_sorter<T> s;
    if (nthreads < 0) {
        nthreads = std::max(std::thread::hardware_concurrency() - 1, 1u);
    }

    simple_async_sorter sorter(nthreads);

    return sorter.do_sort(std::move(input), false);
}

template<typename T>
tracked_vector<T> do_parallel_std_async_copy_sort(
        tracked_vector<T> chunk_data) {
    if (chunk_data.size() < 2) {
        return chunk_data;
    }

    T middle = chunk_data.front();

    tracked_vector<T> new_lower_chunk;
    tracked_vector<T> new_upper_chunk;

    // split into an upper and lower chunk
    size_t  nlower = 0;
    for (size_t  i = 1; i < chunk_data.size(); i++) {
        if (chunk_data[i] > middle) {
            new_upper_chunk.push_back(chunk_data[i]);
        } else {
            new_lower_chunk.push_back(chunk_data[i]);
        }
    }

    std::future<tracked_vector<T>> new_lower = std::async(
            do_parallel_std_async_copy_sort<T>, std::ref(new_lower_chunk));
    tracked_vector<T> new_higher = do_parallel_std_async_copy_sort<T>(
            new_upper_chunk);

    tracked_vector<T> result = new_lower.get();
    result.reserve(result.size() + 1 + new_higher.size());
    result.push_back(middle);
    for (size_t  i = 0; i < new_higher.size(); i++) {
        result.push_back(new_higher[i]);
    }

    return result;
}

template<typename T>
tracked_vector<T> parallel_std_async_quick_sort(tracked_vector<T> input) {
    if (input.empty()) {
        return input;
    }
    return do_parallel_std_async_copy_sort(input);
}

template<typename T>
tracked_vector<T> parallel_quick_sort(tracked_vector<T> input) {
    if (input.empty()) {
        return input;
    }
    // copy_sorter<T> s;
    mas::concurrency::thread_pool pool(std::thread::hardware_concurrency());
    //mas::concurrency::thread_pool pool(1);

    return do_parallel_copy_sort(input, pool);
}

template<typename T>
tracked_vector<T> quick_sort(tracked_vector<T> input) {
    if (input.empty()) {
        return input;
    }
    return do_quick_sort(std::move(input));
}

template<typename T>
bool check_equal(const tracked_vector<T>& v1, const tracked_vector<T>& v2) {
    if (v1.size() != v2.size()) {
        return false;
    }

    for (size_t i = 0; i < v1.size(); ++i) {
        if (v1[i] != v2[i]) {
            return false;
        }
    }
    return true;
}

void doSortTest() {
    tracked_vector<int> vals;
    int N = 20000000;
    for (int i = 0; i < N; i++) {
        vals.push_back(i);
    }
    std::random_shuffle(vals.begin(), vals.end());
    tracked_vector<int> vals2 = vals;

    tracked_vector<int>::reset_counts();

    mas::time::Timer timer;
    timer.start();
    tracked_vector<int> out = quick_sort(std::move(vals));
    timer.stop();
    double ms = timer.getMilliseconds();
    printf("Finished sequential in %g ms\n", ms);
    fflush(stdout);

    tracked_vector<int>::print_counts();
    tracked_vector<int>::reset_counts();

    timer.start();
    tracked_vector<int> out2 = parallel_async_quick_sort(
            std::move(vals2), 3);
    timer.stop();
    double pms = timer.getMilliseconds();

    print_vec(out);
    tracked_vector<int>::print_counts();
    printf("Finished parallel in %g ms\n", pms);
    fflush(stdout);

    if (!check_equal(out, out2)) {
        printf("WHAT?!?!  Answers are different\n");
        fflush(stdout);
        print_vec(out2);
    }

    std::cout << "DONE" << std::endl;
}

int main(int argc, char **argv) {

    doSortTest();

    return 0;
}

