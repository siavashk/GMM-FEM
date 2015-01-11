#include "mas/concurrency/thread.h"
#include "mas/core/time.h"
#include <list>
#include <chrono>
#include <iostream>

template<typename T>
std::vector<T> do_copy_sort(const std::vector<T>& chunk_data, mas::concurrency::thread_pool& pool) {
	if (chunk_data.empty()) {
		return chunk_data;
	}

	T middle = chunk_data.front();

	std::vector<T> new_lower_chunk;
	std::vector<T> new_upper_chunk;

	// split into an upper and lower chunk
	int nupper = 0;
	for (int i=1; i<chunk_data.size(); i++) {
		if (chunk_data[i] > middle) {
			new_upper_chunk.push_back(chunk_data[i]);
		} else {
			new_lower_chunk.push_back(chunk_data[i]);
		}
	}


	std::future<std::vector<T>> new_lower = pool.submit_back(do_copy_sort<T>, std::ref(new_lower_chunk), std::ref(pool));
	std::vector<T> new_higher(do_copy_sort<T>(new_upper_chunk, pool));

	while (!(new_lower.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) {
		pool.run_pending_task();
	}

	std::vector<T> result = new_lower.get();

	result.reserve(result.size()+1+new_higher.size());
	result.push_back(middle);
	for (int i=0; i<new_higher.size(); i++) {
		result.push_back(new_higher[i]);
	}

	return result;
}

template<typename T>
std::vector<T> parallel_quick_sort(const std::vector<T>& input) {
    if (input.empty()) {
        return input;
    }
    // copy_sorter<T> s;
    mas::concurrency::thread_pool pool(std::thread::hardware_concurrency());

    return do_copy_sort(input, pool);
}

void doSortTest() {
	std::vector<double> vals;
    int N = 1000000;
    for (int i=0; i<N; i++) {
        vals.push_back(i);
    }
    std::random_shuffle(vals.begin(), vals.end());

    mas::time::Timer timer;
    timer.start();
    std::vector<double> out = parallel_quick_sort(vals);

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

std::unique_ptr<mas::concurrency::thread_pool> ppool;
std::atomic<int> proccount(0);

double bind_me(int d, double max) {

//	printf("Running task %d\n", d);
//	fflush(stdout);
	if (d < max) {
		std::future<double> f = ppool->submit_back(bind_me, ++proccount, max);

		while (!(f.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) {
			ppool->run_pending_task();
		}

		d += f.get();
	}
	//printf("Done running task %d\n", d);

	return d;
}

void doBindTest() {
	double max = 100;

	ppool = std::unique_ptr<mas::concurrency::thread_pool>(new mas::concurrency::thread_pool(1));
	//std::future<double> outf = pool.submit_back(bind_me, d, max, std::ref(pool));
	std::future<double> outf = ppool->submit_back(bind_me, 0, max);

	//double x = f();
	double out = outf.get();
}

int main(int argc, char **argv) {

    doSortTest();
	// doBindTest();

}

