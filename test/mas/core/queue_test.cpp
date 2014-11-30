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

int main(int argc, char **argv) {

    int len = 1200000;
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

