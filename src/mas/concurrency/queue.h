#ifndef MAS_CONCURRENCY_QUEUE_H_
#define MAS_CONCURRENCY_QUEUE_H_

#include <atomic>
#include <algorithm>

#define MAS_CONCURRENCY_CACHE_LINE_SIZE 64
#define MAS_CONCURRENCY_CACHE_SEPARATE 0

#if MAS_CONCURRENCY_CACHE_SEPARATE > 0
#define ADD_CACHE_PAD(name,space) char name[MAS_CONCURRENCY_CACHE_LINE_SIZE - (space)]
#else
#define ADD_CACHE_PAD(name,space)
#endif

namespace mas {
namespace concurrency {

template<typename T>
class threadsafe_queue {
private:
    struct node {
        node(T* val) :
                value(val), next(nullptr) {
        }
        T* value;
        std::atomic<node*> next;ADD_CACHE_PAD(pad, sizeof(T*)+sizeof(std::atomic<node*>));
    };ADD_CACHE_PAD(pad0, sizeof(T*)+sizeof(std::atomic<node*>));

    node* first;ADD_CACHE_PAD(pad1,sizeof(node*));

    std::atomic<bool> frontLock;ADD_CACHE_PAD(pad2,sizeof(std::atomic<bool>));

    node* last;ADD_CACHE_PAD(pad3,sizeof(node*));

    std::atomic<bool> backLock;ADD_CACHE_PAD(pad4,sizeof(std::atomic<bool>));

public:
    threadsafe_queue();

    ~threadsafe_queue();

    void push_front(const T& t);
    void push_back(const T& t);

    void push_front(T&& t);
    void push_back(T&& t);

    bool pop(T& result);
};

}
}

#include "mas/concurrency/queue.hpp"

#endif /* MAS_CONCURRENCY_QUEUE_H_ */
