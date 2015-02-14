#include "mas/concurrency/queue.h"
#include "mas/concurrency/thread.h"
#include "mas/core/time.h"
#include <iostream>
#include <thread>
#include <array>
#include <atomic>
#include <vector>

template<size_t N = 10>
struct data {
    static std::atomic<int> num;
public:
    std::array<std::string, N> strs;
    int idx;

    data() {
        for (int i = 0; i < strs.size(); i++) {
            strs[i] = "hello world";
        }
        idx = num++;
    }

    //    ~data() {
    //        printf("Deleted data %d\n", idx);
    //    }

    data(int idx) {
        for (int i = 0; i < strs.size(); i++) {
            strs[i] = "hello world";
        }
        this->idx = idx;
    }
};

using mydata = data<10>;

template<size_t N>
std::atomic<int> data<N>::num(0);

void doQueueTest() {

    mas::time::Timer timer;
    timer.start();

    std::vector<std::thread> threads;
    {
        mas::concurrency::threadsafe_queue<std::unique_ptr<mydata>> queue;
        mas::concurrency::thread_group < std::vector< std::thread >> thread_group(threads);
        size_t maxIdx = 10000000;

        auto producer =
                [&queue, &maxIdx] (int threadIdx) {
                    bool done = false;
                    while (!done) {
                        std::unique_ptr<mydata> next(new mydata());
                        int nidx = next->idx;
                        queue.push_back(std::move(next));
                        // std::cout << "Thread " << threadIdx << " produced item " << next.idx << std::endl;
                        if (nidx >= maxIdx) {
                            done = true;
                        }
                    }
                };

        auto consumer =
                [&queue, &maxIdx] (int threadIdx) {
                    bool done = false;
                    std::unique_ptr<mydata> next(nullptr);
                    while (!done) {
                        bool success = queue.pop(next);
                        if (success) {
                            // std::cout << "Thread " << threadIdx << " consumed item " << next.idx << std::endl;
                            if (next->idx >= maxIdx) {
                                done = true;
                            }
                        }
                    }
                };

        // 2 consumers, 2 producers
        threads.push_back(std::thread(producer, 1));
        threads.push_back(std::thread(producer, 2));
        threads.push_back(std::thread(consumer, 3));
        threads.push_back(std::thread(consumer, 4));

        fflush(stdout);

    }

    timer.stop();
    double ms = timer.getMilliseconds();
    printf("Completion time: %g ms\n", ms);
    fflush(stdout);

}

int main(int argc, char **argv) {
    doQueueTest();
}
