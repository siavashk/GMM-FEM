#ifndef MAS_CORE_QUEUE_H_
#define MAS_CORE_QUEUE_H_

#include <vector>
#include <stack>

namespace mas {
namespace queue {

// null move callback
template<typename SizeType, typename ValueType>
struct null_mov_callback {
    void operator()(ValueType &v, const SizeType& x, const SizeType& y) const
    { };
};

/**
 * @brief  A modified version of the priority queue, using mas::heap
 * to control heap operations.  Allows specification of a "move callback"
 * function to aid in keeping track of container locations externally.
 *
 * MoveCallback must define the operator:<br>
 * <code>
 * void operator() (Sequence::value_type& val, const Sequence::size_type& x, const Sequence::size_type& y) const;
 * </code>
 *
 */
template<typename ValueType, typename Sequence = std::vector<ValueType>,
        typename Compare = std::less<typename Sequence::value_type>,
        typename MoveCallback = mas::queue::null_mov_callback<typename Sequence::size_type, typename Sequence::value_type> >
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
    MoveCallback mov;

public:

    explicit priority_queue(const Compare& x = Compare(), const Sequence& s =
            Sequence(), const MoveCallback& m = MoveCallback());

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare& x =
            Compare(), const Sequence& s = Sequence(), const MoveCallback& m = MoveCallback());

    bool empty() const;
    size_type size() const;
    const_reference top() const;
    void push(const value_type& x);
    void pop();

#if __cplusplus >= 201103L
    explicit priority_queue(const Compare& x = Compare(), Sequence&& s =
            Sequence(), const MoveCallback& m = MoveCallback());
    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare& x =
            Compare(), Sequence&& s = Sequence(), const MoveCallback& m = MoveCallback());

    void push(value_type&& x);

    template<typename ... _Args>
    void emplace(_Args&&... __args);

    void swap(priority_queue& __pq)
    noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp)) && noexcept(swap(mov, __pq.mov)));
#endif // __cplusplus


    // NON-STANDARD pop and retrieve top element in queue

    value_type pop_top();
    value_type pop(size_type loc);

    const_reference peek() const;
    const_reference peek(size_type loc) const;

    void update();
    void update(size_type loc);

    bool is_valid() const;

};

/**
 * @brief  A modified-modified version of the priority queue, using mas::heap
 * to control heap operations, and an internal list of indices to keep track
 * of container positions
 */
template<typename ValueType, typename Sequence = std::vector<ValueType>,
        typename Compare = std::less<typename Sequence::value_type>,
        typename IndexSequence = std::vector<typename Sequence::size_type>,
        typename IndexStack = std::stack<typename Sequence::size_type, IndexSequence> >
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

    void swap(mutable_priority_queue& __pq)
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

