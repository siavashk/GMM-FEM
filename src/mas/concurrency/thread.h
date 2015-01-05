/*
 * thread
 *
 *  Created on: Jan 2, 2015
 *      Author: antonio
 */

#ifndef MAS_CONCURRENCY_THREAD_H_
#define MAS_CONCURRENCY_THREAD_H_

#include <thread>

namespace mas {
namespace thread {

/**
 * A class to automatically clean up after a group of threads,
 * joining any last threads before destruction
 */
template<typename RandomThreadIterator>
class thread_group {
private:
    RandomThreadIterator tbegin;
    RandomThreadIterator tend;
public:
    template<typename Container>
    explicit thread_group(Container& threads_);
    explicit thread_group(RandomThreadIterator tbegin,
            RandomThreadIterator tend);

    ~thread_group();

};

} // thread
} // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
