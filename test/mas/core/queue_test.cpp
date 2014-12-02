/*
 * heap_test_cmd.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: antonio
 */

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include "mas/core/queue.h"
#include "mas/core/time.h"

void print_vec(const std::vector<int>& v) {
    for (int i = 0; i < v.size(); i++) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
}

void check_equal(const std::vector<int>& v1, const std::vector<int>& v2) {
    bool equal = true;
    for (int i = 0; i < v1.size(); i++) {
        if (v1[i] != v2[i]) {
            equal = false;
            break;
        }
    }
    if (equal) {
        std::cout << "vectors are equal" << std::endl;
    } else {
        std::cout << "vectors are not equal" << std::endl;

        if (v1.size() < 50) {
            print_vec(v1);
            print_vec(v2);
        }
    }
}

void do_pop_test() {

    int len = 12000000;
    std::vector<int> v1(len);
    for (int i = 0; i < v1.size(); i++) {
        v1[i] = i;
    }
    std::random_shuffle(v1.begin(), v1.end());

    std::vector<int> v2 = v1;

    auto cmp = [](int a, int b) {return a > b;};

    mas::queue::priority_queue<int, std::vector<int>, decltype(cmp)> mas_queue(
            cmp, std::move(v1));
    std::priority_queue<int, std::vector<int>, decltype(cmp)> std_queue(cmp,
            std::move(v2));

    // pop all out
    mas::time::Timer timer;
    double mas_time, std_time;

    timer.start();
    for (int i = 0; i < len; i++) {
        v1.push_back(mas_queue.pop_top());
    }
    timer.stop();
    mas_time = timer.getMilliseconds();
    std::cout << "mas pop sort time: " << mas_time << "ms" << std::endl;

    timer.start();
    for (int i = 0; i < len; i++) {
        v2.push_back(std_queue.top());
        std_queue.pop();
    }
    timer.stop();
    std_time = timer.getMilliseconds();
    std::cout << "std pop sort time: " << std_time << "ms" << std::endl;

}

void do_random_pop_test() {

    int len = 1200000;
    std::vector<int> v1(len);
    for (int i = 0; i < v1.size(); i++) {
        v1[i] = i;
    }
    std::random_shuffle(v1.begin(), v1.end());

    auto cmp = [](int a, int b) {return a > b;};

    mas::queue::priority_queue<int, std::vector<int>, decltype(cmp)> mas_queue(
            cmp, std::move(v1));

    // pop random positions out of queue
    // randomly pop out entries
    static std::random_device rd;
    static std::mt19937 gen(rd());

    bool valid = true;
    for (auto end = v1.end(); end > v1.begin(); end--) {
        std::uniform_int_distribution<size_t> dis(0,
                std::distance(v1.begin(), end) - 1);
        auto pos = dis(gen);

        int val = mas_queue.peek(pos);
        int next = mas_queue.pop(pos);

        if (val != next) {
            std::cout << "uh oh, incorrect value popped out" << std::endl;
        }

        // ensure valid heap
        valid = mas_queue.is_valid();
        if (!valid) {
            std::cout << "    INVALID" << std::endl;
            break;
        }
    }

    if (valid) {
        std::cout << "mas internal pop passed" << std::endl;
    } else {
        std::cout << "mas internal pop failed" << std::endl;
    }
}

void do_random_update_test() {

    int len = 100000000;

    struct widget {
        double priority;
        size_t loc;       // location in queue
    };

    std::vector<widget> wc;
    for (int i=0; i<len; i++) {
        wc.push_back(widget {(double)i, 0} );
    }
    std::random_shuffle(wc.begin(), wc.end());

    // compare priorities
    auto cmp = [](widget* w1, widget* w2) { return w1->priority < w2->priority; };
    // track position in queue
    auto mov = [](widget* w, const size_t& from, const size_t& to){
       w->loc = to;
    };

    mas::queue::priority_queue<widget*, std::vector<widget*>, decltype(cmp), decltype(mov)> queue(cmp, std::vector<widget*>(),
          mov);

    // push a bunch of pointers onto the stack
    for (int i=0; i<len; i++) {
       queue.push(wc.data()+i);
    }

    // check that loc was set in correct order
    bool valid = true;
    for (int i=0; i<len; i++) {
       widget* w = queue.peek(i);
       if (w->loc != i) {
          std::cout << " loc not set correctly, " << i << " vs " << w->loc << std::endl;
          valid = false;
       }
    }

    if (valid) {
        std::cout << "mas loc passed" << std::endl;
    } else {
        std::cout << "mas loc failed" << std::endl;
    }

    mas::time::Timer timer;

    // uniform random number generator
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // update in random order
    timer.start();
    valid = true;
    for (int i=0; i<len; i++) {

        std::uniform_int_distribution<size_t> dis(0, len-1);
        size_t w1idx = dis(gen);
        size_t w2idx = dis(gen);
        size_t loc = wc[w1idx].loc;  // location in queue

        widget* peek = queue.peek(loc);
        if (wc.data()+w1idx != peek) {
            std::cout << "mas loc not correct" << std::endl;
            valid = false;
        }

        // update
        double oprio = wc[w1idx].priority;
        wc[w1idx].priority = wc[w2idx].priority;  // one at a time
        queue.update(loc);

        loc = wc[w2idx].loc;
        wc[w2idx].priority = oprio;
        queue.update(loc);

    }
    timer.stop();
    double mas_ms = timer.getMilliseconds();

    if (!queue.is_valid()) {
       valid = false;
    }
    if (valid) {
        std::cout << "mas random update passed" << std::endl;
    } else {
        std::cout << "mas random update failed" << std::endl;
    }
    std::cout << "mas time: " << mas_ms << "ms" << std::endl;

}

int main(int argc, char **argv) {

    do_pop_test();
    do_random_pop_test();
    do_random_update_test();

}

