#ifndef MAS_CORE_QUEUE_HPP_
#define MAS_CORE_QUEUE_HPP_

#include "mas/core/heap.h"

namespace mas {
namespace queue {

#if __cplusplus >= 201103L
#define __MAS_MOVE(__a) std::move(__a)
#else
#define __MAS_MOVE(__a) (__a)
#endif

/**
 *  @brief  Default constructor.
 */
template<typename ValueType, typename Sequence, typename Compare>
priority_queue<ValueType, Sequence, Compare>::priority_queue(const Compare& x,
        const Sequence& s) :
        c(s), comp(x) {
    mas::heap::make_heap(c.begin(), c.end(), comp);
}

#if __cplusplus >= 201103L
/**
 *  @brief  Moves a provided sequence into the queue
 */
template<typename ValueType, typename Sequence, typename Compare>
priority_queue<ValueType, Sequence, Compare>::priority_queue(const Compare& x,
        Sequence&& s) :
        c(__MAS_MOVE(s)), comp(x) {
    mas::heap::make_heap(c.begin(), c.end(), comp);
}
#endif // __cplusplus

/**
 *  @brief  Builds a %queue from a range.
 *  @param  first  An input iterator.
 *  @param  last  An input iterator.
 *  @param  x  A comparison functor describing a strict weak ordering.
 *  @param  s  An initial sequence with which to start.
 */
template<typename ValueType, typename Sequence, typename Compare>
template<typename InputIterator>
priority_queue<ValueType, Sequence, Compare>::priority_queue(
        InputIterator first, InputIterator last, const Compare& x,
        const Sequence& s) :
        c(s), comp(x) {
    c.insert(c.end(), first, last);
    mas::heap::make_heap(c.begin(), c.end(), comp);
}
#if __cplusplus >= 201103L
/**
 * @brief Moves the provided sequence into the queue
 */
template<typename ValueType, typename Sequence, typename Compare>
template<typename InputIterator>
priority_queue<ValueType, Sequence, Compare>::priority_queue(
        InputIterator first, InputIterator last, const Compare& x,
        Sequence&& s) :
        c(__MAS_MOVE(s)), comp(x) {
    c.insert(c.end(), first, last);
    mas::heap::make_heap(c.begin(), c.end(), comp);
}
#endif //__cplusplus

/**
 *  @brief Returns true if the %queue is empty.
 */
template<typename ValueType, typename Sequence, typename Compare>
bool priority_queue<ValueType, Sequence, Compare>::empty() const {
    return c.empty();
}

/** @brief Returns the number of elements in the %queue.  */
template<typename ValueType, typename Sequence, typename Compare>
typename priority_queue<ValueType, Sequence, Compare>::size_type priority_queue<ValueType, Sequence, Compare>::size() const {
    return c.size();
}

/**
 *  @brief Returns a read-only (constant) reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare>
typename priority_queue<ValueType, Sequence, Compare>::const_reference priority_queue<
        ValueType, Sequence, Compare>::top() const {
    return c.front();
}

/**
 *  @brief  Add data to the %queue.
 *  @param  x  Data to be added.
 */
template<typename ValueType, typename Sequence, typename Compare>
void priority_queue<ValueType, Sequence, Compare>::push(const value_type& x) {
    c.push_back(x);
    mas::heap::push_heap(c.begin(), c.end(), comp);
}

#if __cplusplus >= 201103L
/**
 *  @brief  Move data to the %queue.
 *  @param  x  Data to be moved to the queue.
 */
template<typename ValueType, typename Sequence, typename Compare>
void priority_queue<ValueType, Sequence, Compare>::push(value_type&& x) {
    c.push_back(__MAS_MOVE(x));
    mas::heap::push_heap(c.begin(), c.end(), comp);
}

template<typename ValueType, typename Sequence, typename Compare>
template<typename ... Args>
void priority_queue<ValueType, Sequence, Compare>::emplace(Args&&... __args) {
    c.emplace_back(std::forward<Args>(__args)...);
    mas::heap::push_heap(c.begin(), c.end(), comp);
}
#endif // __cplusplus

/**
 *  @brief  Removes first element.
 */
template<typename ValueType, typename Sequence, typename Compare>
void priority_queue<ValueType, Sequence, Compare>::pop() {
    mas::heap::pop_heap(c.begin(), c.end(), comp);
    c.pop_back();
}

#if __cplusplus >= 201103L
/**
 *  @brief  Swaps all elements with another priority queue
 */
template<typename ValueType, typename Sequence, typename Compare>
void priority_queue<ValueType, Sequence, Compare>::swap(priority_queue& __pq)
        noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp)))
        {
    std::swap(c, __pq.c);
    std::swap(comp, __pq.comp);
}
#endif  // __cplusplus

// NON-STANDARD pop and retrieve top element in queue
/**
 * @brief Removes and returns the first element
 * @return the first element in the queue (moved if c++11)
 */
template<typename ValueType, typename Sequence, typename Compare>
typename priority_queue<ValueType, Sequence, Compare>::value_type priority_queue<
        ValueType, Sequence, Compare>::pop_top() {
    // move out top element
    value_type top = __MAS_MOVE(c.front());
    mas::heap::pop_heap(c.begin(), c.end(), comp);
    c.pop_back(); // remove last element
    return top;
}

/**
 * @brief Updates the queue if any priorities have changed
 */
template<typename ValueType, typename Sequence, typename Compare>
void priority_queue<ValueType, Sequence, Compare>::update() {
    mas::heap::make_heap(c.begin(), c.end, comp);
}

/**
 * @brief Updates the queue given that a single priority has
 * changed at iterator location c.begin()+loc
 * @param loc location within the container that has an updated priority
 *
 * This is much faster than re-heaping the entire queue
 */
template<typename ValueType, typename Sequence, typename Compare>
void priority_queue<ValueType, Sequence, Compare>::update(size_type loc) {
    mas::heap::update_heap(c.begin(), c.end(), c.begin() + loc, comp);
}

} // mas
} // queue

#if __cplusplus >= 201103L

// override swap and uses_allocator
namespace std {

template<typename ValueType, typename Sequence, typename Compare>
inline void swap(mas::queue::priority_queue<ValueType, Sequence, Compare>& x,
        mas::queue::priority_queue<ValueType, Sequence, Compare>& y)
                noexcept(noexcept(x.swap(y)))
                {
    x.swap(y);
}

template<typename ValueType, typename Sequence, typename Compare, typename Alloc>
struct uses_allocator<mas::queue::priority_queue<ValueType, Sequence, Compare>,
        Alloc> : public uses_allocator<Sequence, Alloc>::type {
};

} // std

#endif // __cplusplus

#endif /* MAS_CORE_QUEUE_HPP_ */
