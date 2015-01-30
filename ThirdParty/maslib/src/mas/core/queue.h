#ifndef MAS_CORE_QUEUE_H_
#define MAS_CORE_QUEUE_H_

#include <vector>
#include <stack>

namespace mas {
namespace queue {

// Null callback
template<typename ReferenceType, typename SizeType>
struct null_callback {
   void operator()(ReferenceType v, const SizeType a, const SizeType b) {}
};

/**
 * @brief  A modified version of the priority queue, using mas::heap
 * to control heap operations.  Allows specification of a "moved callback"
 * that is useful to track positions of elements in the queue.
 *
 * The move callback function must define the operation:<br>
 * <code>
 * move(Sequence::value_type& val, const Sequence::size_type a, const Sequence::size_type b);
 * </code>
 * which will be called after <code>val</code> is moved from location <code>a</code> to <code>b</code>
 * in the queue.  The callback can either be an operator on a struct, or a general function handler (e.g. lambda in c++11).
 *
 */
template<typename ValueType, typename Sequence = std::vector<ValueType>,
      typename Compare = std::less<typename Sequence::value_type>,
      typename MoveCallback = mas::queue::null_callback<typename Sequence::reference,
            typename Sequence::size_type> >
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
         Compare(), Sequence&& s = Sequence(), const MoveCallback& m = MoveCallback());

   explicit priority_queue(const Compare& x, const Sequence& s, const MoveCallback& m =
         MoveCallback());

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
   reference get(size_type loc);

   void update();
   void update(size_type loc);

   bool is_valid() const;

   template<typename IterateCallback>
   void iterate(const IterateCallback& cb);

};



}// mas
} // queue

#include "mas/core/queue.hpp"

#endif /* MAS_CORE_QUEUE_HPP_ */

