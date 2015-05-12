/*
 * heap_test_cmd.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: antonio
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include "mas/core/heap.h"
#include "mas/core/time.h"

void print_vec(const std::vector<int>& v) {
    for (int i = 0; i < v.size(); i++) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
}

void print_vec(const std::vector<int>& v, const std::vector<int>& idx) {
    for (int i = 0; i < v.size(); i++) {
        std::cout << v[idx[i]] << " ";
    }
    std::cout << std::endl;
}

template<typename RA>
void print_it(RA start, RA end) {
    for (RA it = start; it < end; it++) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;
}

bool check_equal(const std::vector<int>& v1, const std::vector<int>& v2) {
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

    return equal;
}

bool move_test() {

    int len = 5;
    std::vector<int> v1(len);
    std::vector<int> vsort(len);
    std::vector<int> vidx(len);

    for (int i = 0; i < v1.size(); i++) {
        v1[i] = i;
        vidx[i] = i;
    }
    std::random_shuffle(v1.begin(), v1.end());
    vsort = vidx;

    auto cmp = [&v1](int a, int b) {return (v1[a] > v1[b]);};
    auto mv = [&vsort, &v1](int& v, const size_t& a, const size_t& b) {
        if (vsort.size() < 20) {
            std::cout << v1[v] << ": " << a << "->" << b << std::endl;
        }
        vsort[v] = b;
    };

    if (v1.size() < 20) {
        std::cout << "vorig: ";
        print_vec(v1);
    }
    mas::heap::make_heap(vidx.begin(), vidx.end(), cmp, mv);
    if (v1.size() < 20) {
        std::cout << "heap:  ";
        print_vec(v1, vidx);
        std::cout << "vsort: ";
        print_vec(vsort);
    }

    std::cout << "Checking make_heap" << std::endl;
    for (int i = 0; i < v1.size(); i++) {
        if (i != vidx[vsort[i]]) {
            std::cout << "FAIL!!" << std::endl;
        }
    }

    // pop one from the middle
    std::cout << "Checking pop_heap" << std::endl;
    mas::heap::pop_heap(vidx.begin(), vidx.end(), vidx.begin()+vidx.size()/2, cmp, mv);
    std::cout << "heap:  ";  print_vec(v1, vidx);
    mas::heap::pop_heap(vidx.begin(), vidx.end()-1, cmp, mv);
    std::cout << "heap:  ";  print_vec(v1, vidx);

    for (int i = 0; i < v1.size(); i++) {
        if (i != vidx[vsort[i]]) {
            std::cout << "FAIL!!" << std::endl;
        }
    }

    std::cout << "Checking push_heap" << std::endl;
    mas::heap::push_heap(vidx.begin(), vidx.end()-1,  cmp, mv);
    mas::heap::push_heap(vidx.begin(), vidx.end(),  cmp, mv);
    std::cout << "heap:  ";  print_vec(v1, vidx);
    for (int i = 0; i < v1.size(); i++) {
        if (i != vidx[vsort[i]]) {
            std::cout << "FAIL!!" << std::endl;
        }
    }

    std::cout << "Checking sort_heap" << std::endl;
    mas::heap::sort_heap(vidx.begin(), vidx.end(),  cmp, mv);
    std::cout << "vorig: ";  print_vec(v1);
    std::cout << "vsort: ";  print_vec(vsort);
    std::cout << "heap:  ";  print_vec(v1, vidx);
    for (int i = 0; i < v1.size(); i++) {
        if (i != vidx[vsort[i]]) {
            std::cout << "FAIL!!" << std::endl;
        }
    }

    return true;
}

bool basic_heap_test() {
    auto cmp = [](int a, int b) {return (a > b);};
    auto mv = [](int& v, const size_t& a, const size_t& b) {};

    int len = 10;//100000000;
    std::vector<int> v1(len);
    for (int i = 0; i < v1.size(); i++) {
        v1[i] = i;
    }
    std::random_shuffle(v1.begin(), v1.end());

    std::vector<int> v2 = v1;
    std::vector<int> v3 = v1;

    mas::time::Timer timer;

    // check push
    std::vector<int> v1b = v1;
    std::vector<int> v2b = v2;

    if (v2b.size() < 50) {
        print_vec(v2b);
    }

    timer.start();
    for (int i = 0; i < v1.size(); i++) {
        std::push_heap(v1b.begin(), v1b.begin() + i + 1, cmp);
    }
    timer.stop();
    double std_ms = timer.getMilliseconds();
    std::cout << "std push took " << std_ms << "ms" << std::endl;

    timer.start();
    for (int i = 0; i < v2.size(); i++) {
        mas::heap::push_heap(v2b.begin(), v2b.begin() + i + 1, cmp, mv);
    }
    timer.stop();
    double mas_ms = timer.getMilliseconds();
    std::cout << "mas push took " << mas_ms << "ms" << std::endl;

    check_equal(v1b, v2b);

    timer.start();
    std::make_heap(v1.begin(), v1.end(), cmp);
    timer.stop();
    std_ms = timer.getMilliseconds();
    std::cout << "std took " << std_ms << "ms" << std::endl;

    timer.start();
    mas::heap::make_heap(v2.begin(), v2.end(), cmp, mv);
    timer.stop();
    mas_ms = timer.getMilliseconds();
    std::cout << "mas took " << mas_ms << "ms" << std::endl;

    check_equal(v1, v2);

    if (v1.size() < 50) {
        std::cout << "unsort: ";
        print_vec(v1);
    }

    // sequential pops
    timer.start();
    for (auto e = v1.end(); e > v1.begin(); e--) {
        std::pop_heap(v1.begin(), e, cmp);
    }
    timer.stop();
    std_ms = timer.getMilliseconds();
    if (v1.size() < 50) {
        std::cout << "sorted: ";
        print_vec(v1);
    }
    std::cout << "std pop sort took " << std_ms << "ms" << std::endl;

    if (v2.size() < 50) {
        std::cout << "unsort: ";
        print_vec(v2);
    }

    timer.start();
    for (auto e = v2.end(); e > v2.begin(); e--) {
        mas::heap::pop_heap(v2.begin(), e, cmp, mv);
    }
    timer.stop();
    mas_ms = timer.getMilliseconds();
    if (v2.size() < 50) {
        std::cout << "sorted: ";
        print_vec(v2);
    }
    std::cout << "mas pop sort took " << mas_ms << "ms" << std::endl;

    // make heap again
    std::make_heap(v1.begin(), v1.end(), cmp);
    mas::heap::make_heap(v2.begin(), v2.end(), cmp, mv);

    bool equal = check_equal(v1, v2);
    if (!equal) {
        // check if at least valid
        bool valid1 = std::is_heap(v1.begin(), v1.end(), cmp);
        bool valid2 = std::is_heap(v2.begin(), v2.end(), cmp);
        if (!valid1) {
            std::cout << "std NOT valid" << std::endl;
        }
        if (!valid2) {
            std::cout << "mas NOT valid" << std::endl;
        }
        if (valid1 && valid2) {
            std::cout << "although different, both are valid" << std::endl;
        }
    }

    // sort heap
    timer.start();
    std::sort_heap(v1.begin(), v1.end(), cmp);
    timer.stop();
    std_ms = timer.getMilliseconds();
    std::cout << "std sort took " << std_ms << "ms" << std::endl;

    timer.start();
    mas::heap::sort_heap(v2.begin(), v2.end(), cmp, mv);
    timer.stop();
    mas_ms = timer.getMilliseconds();
    std::cout << "mas sort took " << mas_ms << "ms" << std::endl;

    equal = check_equal(v1, v2);
    if (!equal) {
        // check sorted
        for (int i=0; i<v1.size()-1; i++) {
            if (!cmp(v1[i], v1[i+1])) {
                std::cout << "std NOT sorted at " << (i+1) << std::endl;
            }
        }

        for (int i=0; i<v2.size()-1; i++) {
            if (!cmp(v2[i], v2[i+1])) {
                std::cout << "mas NOT sorted at " << (i+1) << std::endl;
            }
        }
    }

    // update heap
    timer.start();
    std::make_heap(v1.begin(), v1.end(), cmp);
    timer.stop();
    std_ms = timer.getMilliseconds();
    std::cout << "std reheap took " << std_ms << "ms" << std::endl;

    timer.start();
    mas::heap::make_heap(v2.begin(), v2.end(), cmp, mv);
    timer.stop();
    mas_ms = timer.getMilliseconds();
    std::cout << "mas reheap took " << mas_ms << "ms" << std::endl;

    check_equal(v1, v2);

    // invalidate part of heap
    int invalidPos = len / 4;
    v1[invalidPos] = 2 * len;
    v2[invalidPos] = v1[invalidPos];

    auto invalid1 = std::is_heap_until(v1.begin(), v1.end(), cmp);
    auto invalid2 = mas::heap::is_heap_until(v2.begin(), v2.end(), cmp);

    std::cout << "std invalid: " << (invalid1 - v1.begin()) << std::endl;
    std::cout << "mas invalid: " << (invalid2 - v2.begin()) << std::endl;

    if (len < 50) {
        print_vec(v2);
    }

    // update heap
    timer.start();
    std::make_heap(v1.begin(), v1.end(), cmp);
    timer.stop();
    std_ms = timer.getMilliseconds();
    std::cout << "std update took " << std_ms << "ms" << std::endl;

    timer.start();
    mas::heap::update_heap(v2.begin(), v2.end(), v2.begin() + invalidPos, cmp,
            mv);
    timer.stop();
    mas_ms = timer.getMilliseconds();
    std::cout << "mas update took " << mas_ms << "ms" << std::endl;

    // validate
    bool valid1 = std::is_heap(v1.begin(), v1.end(), cmp);
    bool valid2 = std::is_heap(v2.begin(), v2.end(), cmp);

    std::cout << "std valid: " << valid1 << std::endl;
    if (!valid1 && v1.size() < 50) {
        print_vec(v1);
    }
    std::cout << "mas valid: " << valid2 << std::endl;
    if (!valid2 && v2.size() < 50) {
        print_vec(v2);
    }

    // invalidate part of heap again
    invalidPos = len / 3;
    v1[invalidPos] = v1[invalidPos] / 2;
    v2[invalidPos] = v1[invalidPos];

    invalid1 = std::is_heap_until(v1.begin(), v1.end(), cmp);
    invalid2 = mas::heap::is_heap_until(v2.begin(), v2.end(), cmp);

    std::cout << "std invalid: " << (invalid1 - v1.begin()) << std::endl;
    std::cout << "mas invalid: " << (invalid2 - v2.begin()) << std::endl;

    if (len < 50) {
        print_vec(v2);
    }

    // update heap
    timer.start();
    std::make_heap(v1.begin(), v1.end(), cmp);
    timer.stop();
    std_ms = timer.getMilliseconds();
    std::cout << "std update took " << std_ms << "ms" << std::endl;

    timer.start();
    mas::heap::update_heap(v2.begin(), v2.end(), v2.begin() + invalidPos, cmp,
            mv);
    timer.stop();
    mas_ms = timer.getMilliseconds();
    std::cout << "mas update took " << mas_ms << "ms" << std::endl;

    // validate
    valid1 = std::is_heap(v1.begin(), v1.end(), cmp);
    std::cout << "std valid: " << valid1 << std::endl;
    if (!valid1 && v1.size() < 50) {
        print_vec(v1);
    }

    valid2 = std::is_heap(v2.begin(), v2.end(), cmp);
    std::cout << "mas valid: " << valid2 << std::endl;
    if (!valid2 && v2.size() < 50) {
        print_vec(v2);
    }

    // randomly pop out entries
    static std::random_device rd;
    static std::mt19937 gen(rd());

    valid1 = true;
    for (auto end = v2.end(); end > v2.begin(); end--) {
        std::uniform_int_distribution<size_t> dis(0,
                std::distance(v2.begin(), end) - 1);
        auto pos = std::next(v2.begin(), dis(gen));

        if (v2.size() < 50) {
            std::cout << "before: ";
            print_it(v2.begin(), end);
            std::cout << "pop:    " << (pos - v2.begin()) << "(" << *pos << ")"
                    << std::endl;
        }

        mas::heap::pop_heap(v2.begin(), end, pos, cmp, mv);

        if (v2.size() < 50) {
            std::cout << "after:  ";
            print_it(v2.begin(), end);
        }

        // ensure valid heap
        valid1 = std::is_heap(v2.begin(), end - 1, cmp);
        if (!valid1) {
            std::cout << "    INVALID" << std::endl;
            break;
        }
    }

    if (valid1) {
        std::cout << "mas internal pop passed" << std::endl;
    } else {
        std::cout << "mas internal pop failed" << std::endl;
        if (v2.size() < 50) {
            print_vec(v2);
        }
    }

    return true;
}

bool parallel_heap_test() {

    auto cmp = [](int a, int b) {return (a > b);};
    auto mv = [](int& v, const size_t& a, const size_t& b) {};

    int len = 200000000;
    std::vector<int> v1(len);
    for (int i = 0; i < v1.size(); i++) {
        v1[i] = i;
    }
    std::random_shuffle(v1.begin(), v1.end());

    std::vector<int> v2 = v1;

    mas::time::Timer timer;

    timer.start();
    mas::heap::make_heap(v1.begin(), v1.end(), cmp);
    timer.stop();
    double ms = timer.getMilliseconds();
    std::cout << "mas make_heap took " << ms << "ms" << std::endl;

    timer.start();
    // mas::heap::parallel_make_heap(v2.begin(), v2.end(), cmp, 32, std::thread::hardware_concurrency());
    mas::heap::parallel_make_heap(v2.begin(), v2.end(), cmp, 32, 0);
    timer.stop();
    ms = timer.getMilliseconds();
    std::cout << "mas parallel_make_heap took " << ms << "ms" << std::endl;

    check_equal(v1, v2);

    return true;
}

int main(int argc, char **argv) {
    // basic_heap_test();
    // move_test();

    //for (int i=0; i<2; i++) {
        parallel_heap_test();
    //}
    return 0;
}

