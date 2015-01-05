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

} // concurrency
} // mas

#include "mas/concurrency/thread.hpp"

#endif /* MAS_CONCURRENCY_THREAD_H_ */
