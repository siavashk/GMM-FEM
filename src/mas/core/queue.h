#ifndef MAS_CORE_QUEUE_H_
#define MAS_CORE_QUEUE_H_

#include <vector>
#include <stack>

namespace mas {
namespace queue {

// null move callback
template<typename SizeType, typename ValueType>
struct null_mov_callback {
    void operator()(ValueType &v, const SizeType& x, const SizeType& y) const {
    }
    ;
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
        typename MoveCallback = mas::queue::null_mov_callback<
                typename Sequence::size_type, typename Sequence::value_type> >
class priority_queue {
public:
    typedef typename Sequence::value_type value_type;
    typedef typename Sequence::reference reference;
    typedef typename Sequence::const_reference const_reference;
    typedef typename Sequence::size_type size_type;
    typedef Sequence container_type;

    /// null move callback
    struct null_mov_callback {
        void operator()(value_type &v, const size_type& x,
                const size_type& y) const {
        }
        ;
    };

protected:
    //  See queue::c for notes on these names.
    Sequence c;
    Compare comp;
    MoveCallback mov;

public:

    bool empty() const;
    size_type size() const;
    const_reference top() const;
    void push(const value_type& x);
    void pop();

#if __cplusplus < 201103L

    explicit priority_queue(const Compare& x = Compare(), const Sequence& s =
            Sequence(), const MoveCallback& m = MoveCallback());

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare& x =
            Compare(), const Sequence& s = Sequence(), const MoveCallback& m =
            MoveCallback());

#else

    // duplicated due to possible ambiguity for default constructors
    explicit priority_queue(const Compare& x = Compare(), Sequence&& s =
            Sequence(), const MoveCallback& m = MoveCallback());

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare& x =
            Compare(), Sequence&& s = Sequence(), const MoveCallback& m =
            MoveCallback());

    explicit priority_queue(const Compare& x, const Sequence& s,
            const MoveCallback& m = MoveCallback());

    template<typename InputIterator>
    priority_queue(InputIterator first, InputIterator last, const Compare& x,
            const Sequence& s, const MoveCallback& m = MoveCallback());

    void push(value_type&& x);

    template<typename ... _Args>
    void emplace(_Args&&... __args);

    void swap(
            priority_queue& __pq)
                    noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp)) && noexcept(swap(mov, __pq.mov)));
#endif // __cplusplus

    // NON-STANDARD pop and retrieve top element in queue

    value_type pop_top();
    value_type pop(size_type loc);

    const_reference peek() const;
    const_reference peek(size_type loc) const;

    reference get();
    reference get(size_type key);

    void update();
    void update(size_type loc);

    bool is_valid() const;

    template<typename IterateCallback>
    void iterate(const IterateCallback& cb) const;

};

/**
 * @brief  A modified version of the priority queue for easier handling
 * of updates using keys that track positions.
 *
 * This queue keeps two lists: the set of objects in the queue,
 * and a list of keys that correspond to those objects.  Whenever an object
 * is added to the queue, a key is returned.  This key can later be used
 * to signal the queue that the priority for that object has changed.
 *
 * For the update to work properly, only one priority should be changed
 * at a time, immediately followed by an update of the queue using the
 * corresponding key.
 *
 */
template<typename ValueType, typename Sequence = std::vector<ValueType>,
        typename Compare = std::less<typename Sequence::value_type>,
        typename KeySequence = std::vector<typename Sequence::size_type>,
        typename KeyStack = std::stack<typename Sequence::size_type> >

class keyed_priority_queue {
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
    KeySequence keyc;   // sequence of indices used by heap
    KeySequence keymap; // map holding queue locations
    KeyStack keystack;

#if __cplusplus >= 201103L

    // move callback
    std::function<void(const size_type&, const size_type&, const size_type&)> mcb;
#define MCB_INIT [this](const size_type& v, const size_type& i, const size_type& j) {*(keymap.begin()+v) = j;}

    std::function<bool(const size_type&, const size_type&)> keycomp;
#define KEYCOMP_INIT [this](const size_type& x, const size_type& y) { return comp(*(c.begin()+x), *(c.begin()+y)); }

#else
    // move callback
    template<typename SizeType, typename ValueType>
    struct mov_callback {
        void operator()(ValueType &v, const SizeType& i, const SizeType& j) const
        {   *(keymap.begin()+v) = j;};
    };

    mov_callback mcb;

#define MCB_INIT mov_callback()

    // move callback
    template<typename SizeType, typename ValueType>
    struct key_compare {
        bool operator()(const SizeType &x, const SizeType& y) const
        {   return comp(*(c.begin()+x), *(c.begin()+y));};
    };

    key_compare keycomp;
#define KEYCOMP_INIT key_compare()

    explicit keyed_priority_queue(const Compare& x = Compare(),
            const Sequence& s = Sequence());
    explicit keyed_priority_queue(const Compare& x, const Sequence& s, KeySequence& keys);

#endif // __cplusplus

public:

#if __cplusplus >= 201103L

    explicit keyed_priority_queue(const Compare& x = Compare(), Sequence&& s =
            Sequence());
    explicit keyed_priority_queue(const Compare& x, Sequence&& s,
            KeySequence &keys);

    template<typename InputIterator>
    keyed_priority_queue(InputIterator first, InputIterator last,
            const Compare& x = Compare(), Sequence&& s = Sequence());
    template<typename InputIterator>
    keyed_priority_queue(InputIterator first, InputIterator last,
            const Compare& x, Sequence&& s, KeySequence &keys);

    explicit keyed_priority_queue(const Compare& x, const Sequence& s);
    explicit keyed_priority_queue(const Compare& x, const Sequence& s,
            KeySequence &keys);

    template<typename InputIterator>
    keyed_priority_queue(InputIterator first, InputIterator last,
            const Compare& x, const Sequence& s);
    template<typename InputIterator>
    keyed_priority_queue(InputIterator first, InputIterator last,
            const Compare& x, const Sequence& s, KeySequence &keys);

#else

    explicit keyed_priority_queue(const Compare& x = Compare(),
            const Sequence& s = Sequence());
    explicit keyed_priority_queue(const Compare& x, const Sequence& s, KeySequence& keys);

    template<typename InputIterator>
    keyed_priority_queue(InputIterator first, InputIterator last,
            const Compare& x = Compare(), const Sequence& s = Sequence());
    template<typename InputIterator>
    keyed_priority_queue(InputIterator first, InputIterator last,
            const Compare& x, const Sequence& s, KeySequence& keys);

#endif // __cplusplus

    bool empty() const;
    size_type size() const;
    const_reference top() const;

    size_type push(const value_type& x);
    void pop();

#if __cplusplus >= 201103L

    size_type push(value_type&& x);

    template<typename ... _Args>
    size_type emplace(_Args&&... __args);

    void swap(
            keyed_priority_queue& __pq)
                    noexcept(noexcept(swap(c, __pq.c)) && noexcept(swap(comp, __pq.comp))
                            && noexcept(swap(mcb, __pq.mcb))
                            && noexcept(swap(keyc, __pq.keyc))
                            && noexcept(swap(keymap, __pq.keymap))
                            && noexcept(swap(keycomp, __pq.keycomp))
                            && noexcept(swap(keystack, __pq.keystack)));
#endif // __cplusplus

    // NON-STANDARD pop and retrieve top element in queue

    value_type pop_top();
    value_type pop(size_type key);

    const_reference peek() const;
    const_reference peek(size_type key) const;

    reference get();
    reference get(size_type key);

    void update();
    void update(size_type key);

    bool is_valid() const;

    template<typename IterateCallback>
    void iterate(const IterateCallback& cb) const;

};

} // mas
} // queue

#include "mas/core/queue.hpp"

#endif /* MAS_CORE_QUEUE_HPP_ */

