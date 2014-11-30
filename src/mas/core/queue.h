#ifndef MAS_CORE_QUEUE_H_
#define MAS_CORE_QUEUE_H_

#include <vector>
#include <stack>

namespace mas {
namespace queue {

/**
 * @brief  A modified version of the priority queue, using mas::heap
 * to control heap operations.
 */
template<typename ValueType, typename Sequence = std::vector<ValueType>,
        typename Compare = std::less<typename Sequence::value_type> >
class priority_queue {
public:
    typedef typename Sequence::value_type value_type;
    typedef typename Sequence::reference reference;
    typedef typename Sequence::const_reference const_reference;
    typedef typename Sequence::size_type size_type;
    typedef Sequence container_type;


protected:
    //  See queue::c for notes on these names.
    Sequence c;
    Compare comp;

public:

    explicit priority_queue(const Compare& x = Compare(), const Sequence& s =
            Sequence());

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare& x =
            Compare(), const Sequence& s = Sequence());

    bool empty() const;
    size_type size() const;
    const_reference top() const;
    void push(const value_type& x);
    void pop();

#if __cplusplus >= 201103L
    explicit priority_queue(const Compare& x = Compare(), Sequence&& s =
            Sequence());
    template<typename InputIterator>
        priority_queue(InputIterator first, InputIterator last, const Compare& x =
                Compare(), Sequence&& s = Sequence());
    void push(value_type&& x);

    template<typename ... _Args>
    void emplace(_Args&&... __args);

    void swap(priority_queue& __pq)
                        noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp)));
#endif // __cplusplus


    // NON-STANDARD pop and retrieve top element in queue

    value_type pop_top();

    void update();
    void update(size_type loc);

};

/**
 * @brief  A modified-modified version of the priority queue, using mas::heap
 * to control heap operations, and an internal list of indices to keep track
 * of container positions
 */
template<typename ValueType, typename Sequence = std::vector<ValueType>,
        typename Compare = std::less<typename Sequence::value_type>,
        typename IndexSequence = std::vector<Sequence::size_type>,
        typename IndexStack = std::stack<Sequence::size_type, IndexSequence> >
class mutable_priority_queue {
public:
    typedef typename Sequence::value_type value_type;
    typedef typename Sequence::reference reference;
    typedef typename Sequence::const_reference const_reference;
    typedef typename Sequence::size_type size_type;
    typedef Sequence container_type;
    typedef typename Sequence::size_type key_type;


protected:
    //  See queue::c for notes on these names.
    Sequence c;
    Compare comp;
    IndexSequence keys;
    IndexStack free_keys;

public:

    explicit mutable_priority_queue(const Compare& x = Compare(), const Sequence& s =
            Sequence());

    template<typename InputIterator>
    mutable_priority_queue(InputIterator first, InputIterator last, const Compare& x =
            Compare(), const Sequence& s = Sequence());

    bool empty() const;
    size_type size() const;
    const_reference top() const;
    key_type push(const value_type& x);
    void pop();

#if __cplusplus >= 201103L
    explicit mutable_priority_queue(const Compare& x = Compare(), Sequence&& s =
            Sequence());
    template<typename InputIterator>
        mutable_priority_queue(InputIterator first, InputIterator last, const Compare& x =
                Compare(), Sequence&& s = Sequence());
    void push(value_type&& x);

    template<typename ... _Args>
    key_type emplace(_Args&&... __args);

    void swap(priority_queue& __pq)
                        noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp))
                                && noexcept(swap(keys, __pq.keys)) && noexcept(swap(free_keys, __pq.free_keys)));
#endif // __cplusplus


    // NON-STANDARD pop and retrieve top element in queue

    value_type pop_top();

    void update();
    void update(key_type loc);

};

} // mas
} // queue

#include "mas/core/queue.hpp"

#endif /* MAS_CORE_QUEUE_HPP_ */

