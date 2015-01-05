namespace mas {
namespace thread {

template<typename RandomThreadIterator>
template<typename Container>
thread_group<RandomThreadIterator>::thread_group(Container& threads_) :
        tbegin(std::begin(threads_)), tend(std::end(threads_)) {
}

template<typename RandomThreadIterator>
thread_group<RandomThreadIterator>::thread_group(RandomThreadIterator tbegin,
        RandomThreadIterator tend) :
        tbegin(tbegin), tend(tend) {
}

template<typename RandomThreadIterator>
thread_group<RandomThreadIterator>::~thread_group() {
    for (RandomThreadIterator t = tbegin; t < tend; t++) {
        if (t->joinable()) {
            t->join();
        }
    }
}

} // thread
} // mas
