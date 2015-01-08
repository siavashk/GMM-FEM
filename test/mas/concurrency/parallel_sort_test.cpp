#include "mas/concurrency/thread.h"
#include "mas/core/time.h"
#include <list>
#include <chrono>
#include <iostream>

template<typename T>
struct sorter {
    mas::concurrency::thread_pool pool;

    std::vector<T> do_sort(std::vector<T>&& chunk_data, int& counter) {
        if (chunk_data.empty()) {
            return chunk_data;
        }

        std::vector<T> middle;
        middle.push_back(std::move(chunk_data.front()));
        T const& partition_val = *middle.begin();

        std::vector<T> new_lower_chunk;

        // split into an upper and lower chunk
        int nupper = 0;
        for (int i=1; i<chunk_data.size(); i++) {
            if (chunk_data[i] > partition_val) {
                chunk_data[nupper++] = std::move(chunk_data[i]);
            } else if (chunk_data[i] == partition_val) {
                middle.push_back(std::move(chunk_data[i]));
            } else {
                new_lower_chunk.push_back(std::move(chunk_data[i]));
            }
        }
        chunk_data.resize(nupper);

        auto f = mas::concurrency::bind_simple(&sorter::do_sort, this, std::move(new_lower_chunk), std::ref(counter));
        std::future<std::vector<T> > new_lower = pool.submit_back(f);

        std::vector<T> new_higher(do_sort(std::move(chunk_data), counter));

        while (!(new_lower.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) {
            pool.run_pending_task();
        }

        std::vector<T> result = std::move(new_lower.get());
        result.reserve(result.size()+middle.size()+new_higher.size());
        for (int i=0; i<middle.size(); i++) {
            result.push_back(std::move(middle[i]));
        }
        for (int i=0; i<new_higher.size(); i++) {
            result.push_back(std::move(new_higher[i]));
        }

        return result;
    }
};

template<typename T>
std::vector<T> parallel_quick_sort(std::vector<T>&& input) {
    if (input.empty()) {
        return input;
    }
    sorter<T> s;
    int counter = 10;
    return s.do_sort(std::move(input), counter);
}

int main(int argc, char **argv) {

    std::vector<double> vals;
    int N = 1000000;
    for (int i=0; i<N; i++) {
        vals.push_back(i);
    }
    std::random_shuffle(vals.begin(), vals.end());

    mas::time::Timer timer;
    timer.start();
    std::vector<double> out = parallel_quick_sort(std::move(vals));
    timer.stop();
    double ms = timer.getMilliseconds();

    if (N <= 100) {
        for (int i=0; i<N; i++) {
            std::cout << out[i] << " ";
        }
        std::cout << std::endl;
    }

    printf("Finished in %g ms\n", ms);
    fflush(stdout);

    std::cout << "DONE" << std::endl;

}

