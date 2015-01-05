namespace mas {
namespace concurrency {

template<typename Container>
thread_group<Container>::thread_group(Container& threads) :
        c(threads) {
}

template<typename Container>
thread_group<Container>::~thread_group() {
    for (auto t = begin(c); t < end(c); t++) {
        if (t->joinable()) {
            t->join();
        }
    }
}

} // concurrency
} // mas
