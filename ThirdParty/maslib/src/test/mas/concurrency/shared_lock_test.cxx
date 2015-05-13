#include <iostream>
#include <atomic>
#include <mutex>
#include <vector>
#include "mas/concurrency/thread.h"
#include "mas/concurrency/shared_mutex.h"

void doSharedTest() {

    using namespace mas::concurrency;

    mas::concurrency::shared_timed_mutex mutex;
    int val = 10;
    std::atomic<int> aval;
    aval.store(val);

}

int main(int argc, char **argv) {

    doSharedTest();

    return 0;

}
