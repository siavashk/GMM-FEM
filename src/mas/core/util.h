/*
 * util.h
 *
 *  Created on: Nov 18, 2014
 *      Author: antonio
 */

#ifndef MAS_CORE_UTIL_H_
#define MAS_CORE_UTIL_H_

#include <queue>

namespace mas {

/**
 *  @brief  A modified container automatically sorting its contents, allowing
 *  non-const removal of first element.
 *
 *  @ingroup sequences
 *
 *  @tparam _Tp  Type of element.
 *  @tparam _Sequence  Type of underlying sequence, defaults to vector<_Tp>.
 *  @tparam _Compare  Comparison function object type, defaults to
 *                    less<_Sequence::value_type>.
 *
 * This is an extension to the standard std::priority_queue, allowing
 * one to extract a non-const reference to the top element of the queue.
 * To preserve the invariants, the element is removed.
 *
 @note Sorting of the elements takes place as they are added to,
 *  and removed from, the %priority_queue using the
 *  %priority_queue's member functions.  If you access the elements
 *  by other means, and change their data such that the sorting
 *  order would be different, the %priority_queue will not re-sort
 *  the elements for you.  (How could it know to do so?)
 */
template<typename _Tp, typename _Sequence = vector<_Tp>,
		typename _Compare = less<typename _Sequence::value_type> >
class priority_queue: std::priority_queue<_Tp, _Sequence, _Compare> {
public:
	/**
	 *  @brief  Default constructor creates no elements.
	 */
#if __cplusplus < 201103L
	explicit
	priority_queue(const _Compare& __x = _Compare(),
			const _Sequence& __s = _Sequence())
	: std::priority_queue(__x, __s) {}
#else
	explicit priority_queue(const _Compare& __x, const _Sequence& __s) :
			std::priority_queue(__x, __s) {
	}

	explicit priority_queue(const _Compare& __x = _Compare(), _Sequence&& __s =
			_Sequence()) :
			std::priority_queue(std::move(__s), __x) {
	}
#endif

	/**
	 *  Returns a read-only (constant) reference to the data at the first
	 *  element of the %queue.
	 */
	const_reference top() const {
		__glibcxx_requires_nonempty();
		return c.front();
	}

	/**
	 *  @brief  Removes first element.
	 *
	 *  This is a typical %queue operation.  It shrinks the %queue
	 *  by one.  The time complexity of the operation depends on the
	 *  underlying sequence.
	 *
	 *  Note that no data is returned, and if the first element's
	 *  data is needed, it should be retrieved before pop() is
	 *  called.
	 */
	void pop() {
		__glibcxx_requires_nonempty();
		std::pop_heap(c.begin(), c.end(), comp);
		c.pop_back();
	}

#if __cplusplus >= 201103L

	/**
	 *  @brief  Removes and returns the first element.
	 *
	 *  This is a typical %queue operation, simultaneously doing
	 *  a %top() and %pop().  It uses move semantics to transfer the
	 *  element out of the container, then shrinks the %queue
	 *  by one.  The returned element is a non-const version.
	 *  The time complexity of the operation depends on the
	 *  underlying sequence.
	 *
	 */
	value_type pop_top() {
		__glibcxx_requires_nonempty();

		// arrange so that back contains desired
		std::pop_heap(c.begin(), c.end(), comp);
		value_type top = std::move(c.back());
		c.pop_back();
		return top;
	}

#endif

};

} // end namespace mas

#endif /* SRC_MAS_CORE_UTIL_H_ */
