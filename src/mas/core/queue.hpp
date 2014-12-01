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
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
priority_queue<ValueType, Sequence, Compare, MoveCallback>::priority_queue(
        const Compare& x, const Sequence& s, const MoveCallback& m) :
        c(s), comp(x), mov(m) {
    mas::heap::make_heap(c.begin(), c.end(), comp, mov);
}

#if __cplusplus >= 201103L
/**
 *  @brief  Moves a provided sequence into the queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
priority_queue<ValueType, Sequence, Compare, MoveCallback>::priority_queue(
        const Compare& x, Sequence&& s, const MoveCallback& m) :
        c(__MAS_MOVE(s)), comp(x), mov(m) {
    mas::heap::make_heap(c.begin(), c.end(), comp, mov);
}
#endif // __cplusplus

/**
 *  @brief  Builds a %queue from a range.
 *  @param  first  An input iterator.
 *  @param  last  An input iterator.
 *  @param  x  A comparison functor describing a strict weak ordering.
 *  @param  s  An initial sequence with which to start.
 *  @param  m  a callback function to be executed after an element is moved
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
template<typename InputIterator>
priority_queue<ValueType, Sequence, Compare, MoveCallback>::priority_queue(
        InputIterator first, InputIterator last, const Compare& x,
        const Sequence& s, const MoveCallback& m) :
        c(s), comp(x), mov(m) {
    c.insert(c.end(), first, last);
    mas::heap::make_heap(c.begin(), c.end(), comp, mov);
}
#if __cplusplus >= 201103L
/**
 * @brief Moves the provided sequence into the queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
template<typename InputIterator>
priority_queue<ValueType, Sequence, Compare, MoveCallback>::priority_queue(
        InputIterator first, InputIterator last, const Compare& x, Sequence&& s,
        const MoveCallback& m) :
        c(__MAS_MOVE(s)), comp(x), mov(m) {
    c.insert(c.end(), first, last);
    mas::heap::make_heap(c.begin(), c.end(), comp, mov);
}
#endif //__cplusplus

/**
 *  @brief Returns true if the %queue is empty.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
bool priority_queue<ValueType, Sequence, Compare, MoveCallback>::empty() const {
    return c.empty();
}

/** @brief Returns the number of elements in the %queue.  */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::size_type priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::size() const {
    return c.size();
}

/**
 *  @brief Returns a read-only (constant) reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::const_reference priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::top() const {
    return c.front();
}

/**
 *  @brief  Add data to the %queue.
 *  @param  x  Data to be added.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::push(
        const value_type& x) {
    c.push_back(x);
    mas::heap::push_heap(c.begin(), c.end(), comp, mov);
}

#if __cplusplus >= 201103L
/**
 *  @brief  Move data to the %queue.
 *  @param  x  Data to be moved to the queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::push(
        value_type&& x) {
    c.push_back(__MAS_MOVE(x));
    mas::heap::push_heap(c.begin(), c.end(), comp, mov);
}

template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
template<typename ... Args>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::emplace(
        Args&&... __args) {
    c.emplace_back(std::forward<Args>(__args)...);
    mas::heap::push_heap(c.begin(), c.end(), comp, mov);
}
#endif // __cplusplus

/**
 *  @brief  Removes first element.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::pop() {
    mas::heap::pop_heap(c.begin(), c.end(), comp, mov);
    c.pop_back();
}

#if __cplusplus >= 201103L
/**
 *  @brief  Swaps all elements with another priority queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::swap(
        priority_queue& __pq)
                noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp)) && noexcept(swap(mov, __pq.mov)))
                {
    std::swap(c, __pq.c);
    std::swap(comp, __pq.comp);
    std::swap(mov, __pq.mov);
}
#endif  // __cplusplus

// NON-STANDARD pop and retrieve top element in queue
/**
 * @brief Removes and returns the first element
 * @return the first element in the queue (moved if c++11)
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::value_type priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::pop_top() {
    // move out top element
    value_type top = __MAS_MOVE(c.front());
    mas::heap::pop_heap(c.begin(), c.end(), comp, mov);
    c.pop_back(); // remove last element
    return top;
}

/**
 * @brief Removes and returns the element located at position
 *        loc inside the queue's container
 * @return the removed element from the queue (moved if c++11)
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::value_type priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::pop(size_type loc) {
    // move out top element
    value_type val = __MAS_MOVE(*(c.begin() + loc));
    mas::heap::pop_heap(c.begin(), c.end(), c.begin() + loc, comp, mov);
    c.pop_back(); // remove last element
    return val;
}

/**
 * @brief Updates the queue if any priorities have changed
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::update() {
    mas::heap::make_heap(c.begin(), c.end, comp, mov);
}

/**
 * @brief Updates the queue given that a single priority has
 * changed at iterator location c.begin()+loc
 * @param loc location within the container that has an updated priority
 *
 * This is much faster than re-heaping the entire queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::update(
        size_type loc) {
    mas::heap::update_heap(c.begin(), c.end(), c.begin() + loc, comp, mov);
}

/**
 *  @brief Returns a read-only (constant) reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::const_reference priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::peek() const {
    return c.front();
}

/**
 *  @brief Returns a read-only (constant) reference to the data at position loc
 *  in the container
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::const_reference priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::peek(size_type loc) const {
    return *(c.begin() + loc);
}

/**
 *  @brief Returns a reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::reference priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::get() {
    return c.front();
}

/**
 *  @brief Returns a read-only (constant) reference to the data at position loc
 *  in the container
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
typename priority_queue<ValueType, Sequence, Compare, MoveCallback>::reference priority_queue<
        ValueType, Sequence, Compare, MoveCallback>::get(size_type loc) {
    return *(c.begin() + loc);
}

template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
bool priority_queue<ValueType, Sequence, Compare, MoveCallback>::is_valid() const {
    return mas::heap::is_heap(c.begin(), c.end(), comp);
}

template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
template<typename IterateCallback>
void priority_queue<ValueType, Sequence, Compare, MoveCallback>::iterate(const IterateCallback& cb) const {
    for (auto it = c.begin(); it < c.end(); it++) {
        cb(it);
    }
}

} // mas
} // queue

#if __cplusplus >= 201103L

// override swap and uses_allocator
namespace std {

template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback>
inline void swap(
        mas::queue::priority_queue<ValueType, Sequence, Compare, MoveCallback>& x,
        mas::queue::priority_queue<ValueType, Sequence, Compare, MoveCallback>& y)
                noexcept(noexcept(x.swap(y)))
                {
    x.swap(y);
}

template<typename ValueType, typename Sequence, typename Compare,
        typename MoveCallback, typename Alloc>
struct uses_allocator<
        mas::queue::priority_queue<ValueType, Sequence, Compare, MoveCallback>,
        Alloc> : public uses_allocator<Sequence, Alloc>::type {
};

} // std

//--------------------------------------------------------
// KEYED PRIORITY QUEUE IMPLEMENTATION
//--------------------------------------------------------

namespace mas {
namespace queue {

#endif // __cplusplus

/**
 *  @brief  Default constructor.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        const Compare& x, const Sequence& s) :
        c(s), comp(x), keyc(), keystack(), keymap() {

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}

/**
 *  @brief  Default constructor.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        const Compare& x, const Sequence& s, KeySequence &keys) :
        c(s), comp(x), keyc(), keystack(), keymap() {

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;
    keys = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}

#if __cplusplus >= 201103L
/**
 *  @brief  Moves a provided sequence into the queue.  The KeySequence argument gets populated
 *  with a set of keys corresponding to the provided elements.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        const Compare& x, Sequence&& s) :
        c(__MAS_MOVE(s)), comp(x), keyc(), keystack(), keymap() {

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}

/**
 *  @brief  Moves a provided sequence into the queue.  The KeySequence argument gets populated
 *  with a set of keys corresponding to the provided elements.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        const Compare& x, Sequence&& s, KeySequence& keys) :
        c(__MAS_MOVE(s)), comp(x), keyc(), keystack(), keymap() {

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;
    keys = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}
#endif // __cplusplus

/**
 *  @brief  Builds a %queue from a range.
 *  @param  first  An input iterator.
 *  @param  last  An input iterator.
 *  @param  x  A comparison functor describing a strict weak ordering.
 *  @param  s  An initial sequence with which to start.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
template<typename InputIterator>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        InputIterator first, InputIterator last, const Compare& x,
        const Sequence& s) :
        c(s), comp(x), keyc(), keystack(), keymap() {

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    c.insert(c.end(), first, last);

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}

/**
 *  @brief  Builds a %queue from a range.
 *  @param  first  An input iterator.
 *  @param  last  An input iterator.
 *  @param  x  A comparison functor describing a strict weak ordering.
 *  @param  s  An initial sequence with which to start.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
template<typename InputIterator>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        InputIterator first, InputIterator last, const Compare& x,
        const Sequence& s, KeySequence &keys) :
        c(s), comp(x), keyc(), keystack(), keymap() {

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    c.insert(c.end(), first, last);

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;
    keys = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}

#if __cplusplus >= 201103L
/**
 * @brief Moves the provided sequence into the queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
template<typename InputIterator>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        InputIterator first, InputIterator last, const Compare& x, Sequence&& s) :
        c(__MAS_MOVE(s)), comp(x), keyc(), keystack(), keymap() {

    c.insert(c.end(), first, last);

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    c.insert(c.end(), first, last);

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}

/**
 * @brief Moves the provided sequence into the queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
template<typename InputIterator>
keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::keyed_priority_queue(
        InputIterator first, InputIterator last, const Compare& x, Sequence&& s,
        KeySequence &keys) :
        c(__MAS_MOVE(s)), comp(x), keyc(), keystack(), keymap() {

    c.insert(c.end(), first, last);

    // initialize internal functions
    mcb = MCB_INIT;
    keycomp = KEYCOMP_INIT;

    c.insert(c.end(), first, last);

    // populate indices
    for (size_type i = 0; i < c.size(); i++) {
        keyc.push_back(i);
    }
    keymap = keyc;
    keys = keyc;

    mas::heap::make_heap(keyc.begin(), keyc.end(), keycomp, mcb);
}
#endif //__cplusplus

/**
 *  @brief Returns true if the %queue is empty.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
bool keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::empty() const {
    return keyc.empty();
}

/** @brief Returns the number of elements in the %queue.  */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::size_type keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::size() const {
    return keyc.size();
}

/**
 *  @brief Returns a read-only (constant) reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::const_reference keyed_priority_queue<ValueType, Sequence,
        Compare, KeySequence, KeyStack>::top() const {
    return *(c.begin() + keyc.front());  // first element
}

/**
 *  @brief  Add data to the %queue, returning a key for future references to the item.
 *  @param  x  Data to be added.
 *  @return key value corresponding to the new element
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::size_type keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::push(const value_type& x) {

    size_type key;

    // get next appropriate key
    if (keystack.empty()) {
        // add x to end of container
        key = c.size();
        c.push_back(x);
        keymap.push_back(keyc.size());       // added to back of queue
    } else {
        key = keystack.top();
        keystack.pop();
        *(c.begin() + key) = x; // copy x into container
        *(keymap.begin() + key) = keyc.size(); // added to back of queue
    }

    // add key to heap and push
    keyc.push_back(key);
    mas::heap::push_heap(keyc.begin(), keyc.end(), keycomp, mcb);

    return key;
}

#if __cplusplus >= 201103L
/**
 *  @brief  Move data to the %queue, returning a key for future references
 *  to this item.
 *  @param  x  Data to be moved to the queue.
 *  @return key value corresponding to new element
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::size_type keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::push(value_type&& x) {

    size_type key;

    // get next appropriate key
    if (keystack.empty()) {
        // add x to end of container
        key = c.size();
        c.push_back(__MAS_MOVE(x));
        keymap.push_back(keyc.size());       // added to back of queue
    } else {
        key = keystack.top();
        keystack.pop();
        *(c.begin() + key) = __MAS_MOVE(x);    // move x into container
        *(keymap.begin() + key) = keyc.size(); // added to back of queue
    }

    // add key to heap and push
    keyc.push_back(key);
    mas::heap::push_heap(keyc.begin(), keyc.end(), keycomp, mcb);

    return key;
}

/**
 * Adds a new element to the queue, returning a key for future references to this item.
 * @param __args argument for constructing the new element
 * @return key key value corresponding to new element
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
template<typename ... Args>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::size_type keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::emplace(Args&&... __args) {

    size_type key;

    // get next appropriate key
    if (keystack.empty()) {
        // add x to end of container
        key = c.size();
        c.emplace_back(std::forward<Args>(__args)...);
        keymap.push_back(keyc.size());       // added to back of queue
    } else {
        key = keystack.top();
        keystack.pop();
        c.emplace(c.begin() + key, std::forward<Args>(__args)...);
        *(keymap.begin() + key) = keyc.size(); // added to back of queue
    }

    // add key to heap and push
    keyc.push_back(key);
    mas::heap::push_heap(keyc.begin(), keyc.end(), keycomp, mcb);

    return key;
}
#endif // __cplusplus

/**
 *  @brief  Removes first element.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
void keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::pop() {

    // free the key
    const size_type& key = keyc.front();
    // if it's the last key, decrease container size
    if (key == c.size() - 1) {
        c.pop_back();
        keymap.pop_back();

        // check if we can clear any keys from the stack
        while (!keystack.empty() && (keystack.top() == c.size() - 1)) {
            c.pop_back();
            keymap.pop_back();
            keystack.pop();
        }

    } else {
        // add key to potential stack
        keystack.push(key);
    }

    mas::heap::pop_heap(keyc.begin(), keyc.end(), keycomp, mcb);
    keyc.pop_back();
}

#if __cplusplus >= 201103L
/**
 *  @brief  Swaps all elements with another priority queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
void keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::swap(
        keyed_priority_queue& __pq)
                noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp))
                        && noexcept(swap(mcb, __pq.mcb))
                        && noexcept(swap(keyc, __pq.keyc))
                        && noexcept(swap(keymap, __pq.keymap))
                        && noexcept(swap(keycomp, __pq.keycomp))
                        && noexcept(swap(keystack, __pq.keystack))) {
    std::swap(c, __pq.c);
    std::swap(comp, __pq.comp);
    std::swap(mcb, __pq.mcb);
    std::swap(keyc, __pq.keyc);
    std::swap(keymap, __pq.keymap);
    std::swap(keycomp, __pq.keycomp);
    std::swap(keystack, __pq.keystack);
}
#endif  // __cplusplus

// NON-STANDARD pop and retrieve top element in queue
/**
 * @brief Removes and returns the first element
 * @return the first element in the queue (moved if c++11)
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::value_type keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::pop_top() {

    // free the key
    const size_type& key = keyc.front();

    // move out element
    value_type top = __MAS_MOVE(*(c.begin() + key));

    // if it's the last key, decrease container size
    if (key == c.size() - 1) {
        c.pop_back();
        keymap.pop_back();

        // check if we can clear any keys from the stack
        while (!keystack.empty() && (keystack.top() == c.size() - 1)) {
            c.pop_back();
            keystack.pop();
            keymap.pop_back();
        }

    } else {
        // add key to potential stack
        keystack.push(key);
    }

    mas::heap::pop_heap(keyc.begin(), keyc.end(), keycomp, mcb);
    keyc.pop_back();

    return top;
}

/**
 * @brief Removes and returns the element located at position
 *        key inside the queue's container
 * @param key the key corresponding to the element to remove
 * @return the removed element from the queue (moved if c++11)
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::value_type keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::pop(size_type key) {

    size_type loc = *(keymap.begin() + key); // location in queue

    // move out element
    value_type val = __MAS_MOVE(*(c.begin() + key));

    // move out top element
    // if it's the last key, decrease container size
    if (key > 0 && key == c.size() - 1) {
        c.pop_back();
        keymap.pop_back();

        // check if we can clear any keys from the stack
        while (!keystack.empty() && (keystack.top() == c.size() - 1)) {
            c.pop_back();
            keystack.pop();
            keymap.pop_back();
        }

    } else {
        // add key to potential stack
        keystack.push(key);
    }

    // pop out key heap element at loc
    mas::heap::pop_heap(keyc.begin(), keyc.end(), keyc.begin() + loc, keycomp,
            mcb);
    keyc.pop_back(); // remove last element

    return val;
}

/**
 * @brief Updates the queue if any priorities have changed
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
void keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::update() {

    // update heap
    mas::heap::make_heap(keyc.begin(), keyc.end, keycomp, mcb);
}

/**
 * @brief Updates the queue given that a single priority has
 * changed at iterator location c.begin()+loc
 * @param loc location within the container that has an updated priority
 *
 * This is much faster than re-heaping the entire queue
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
void keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::update(
        size_type key) {

    size_type loc = *(keymap.begin() + key); // location in queue
    mas::heap::update_heap(keyc.begin(), keyc.end(), keyc.begin() + loc,
            keycomp, mcb);
}

/**
 *  @brief Returns a read-only (constant) reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::const_reference keyed_priority_queue<ValueType, Sequence,
        Compare, KeySequence, KeyStack>::peek() const {
    size_type key = keyc.front();
    return *(c.begin() + key);
}

/**
 *  @brief Returns a read-only (constant) reference to the data corresponding to the
 *  provided key
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::const_reference keyed_priority_queue<ValueType, Sequence,
        Compare, KeySequence, KeyStack>::peek(size_type key) const {
    return *(c.begin() + key);
}

/**
 *  @brief Returns a reference to the data at the first
 *  element of the %queue.
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::reference keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::get() {
    size_type key = keyc.front();
    return *(c.begin() + key);
}

/**
 *  @brief Returns a reference to the data corresponding to the
 *  provided key
 */
template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
typename keyed_priority_queue<ValueType, Sequence, Compare, KeySequence,
        KeyStack>::reference keyed_priority_queue<ValueType, Sequence, Compare,
        KeySequence, KeyStack>::get(size_type key) {
    return *(c.begin() + key);
}

template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
bool keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::is_valid() const {
    return mas::heap::is_heap(keyc.begin(), keyc.end(), keycomp);
}

template<typename ValueType, typename Sequence, typename Compare, typename KeySequence, typename KeyStack>
template<typename IterateCallback>
void keyed_priority_queue<ValueType, Sequence, Compare, KeySequence, KeyStack>::iterate(const IterateCallback& cb) const {
    for (const auto it = c.begin(); it < c.end(); c++) {
        cb(it);
    }
}

} // mas
} // queue

#if __cplusplus >= 201103L

// override swap and uses_allocator
namespace std {

template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack>
inline void swap(
        mas::queue::keyed_priority_queue<ValueType, Sequence, Compare,
                KeySequence, KeyStack>& x,
        mas::queue::keyed_priority_queue<ValueType, Sequence, Compare,
                KeySequence, KeyStack>& y) noexcept(noexcept(x.swap(y)))
        {
    x.swap(y);
}

template<typename ValueType, typename Sequence, typename Compare,
        typename KeySequence, typename KeyStack, typename Alloc>
struct uses_allocator<
        mas::queue::keyed_priority_queue<ValueType, Sequence, Compare,
                KeySequence, KeyStack>, Alloc> : public uses_allocator<Sequence,
        Alloc>::type {
};

} // std

#endif // __cplusplus

#endif /* MAS_CORE_QUEUE_HPP_ */
