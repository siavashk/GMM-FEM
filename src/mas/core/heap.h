#ifndef MAS_CORE_HEAP_H_
#define MAS_CORE_HEAP_H_

namespace mas {
namespace heap {

//======================================================================
// std-compliant heap implementation
//======================================================================

template<typename RandomAccessIterator, typename Compare>
void make_heap(RandomAccessIterator first, RandomAccessIterator last,
		Compare compare);

template<typename RandomAccessIterator>
void make_heap(RandomAccessIterator first, RandomAccessIterator last);

template<typename RandomAccessIterator, typename Compare>
void push_heap(RandomAccessIterator first, RandomAccessIterator last,
		Compare compare);
template<typename RandomAccessIterator>
void push_heap(RandomAccessIterator first, RandomAccessIterator last);

template<typename RandomAccessIterator, typename Compare>
void pop_heap(RandomAccessIterator first, RandomAccessIterator last,
		Compare compare);
template<typename RandomAccessIterator>
void pop_heap(RandomAccessIterator first, RandomAccessIterator last);

template<typename RandomAccessIterator, typename Compare>
void sort_heap(RandomAccessIterator first, RandomAccessIterator last,
		Compare compare);
template<typename RandomAccessIterator>
void sort_heap(RandomAccessIterator first, RandomAccessIterator last);

template<typename RandomAccessIterator, typename Compare>
RandomAccessIterator is_heap_until(RandomAccessIterator first,
		RandomAccessIterator last, Compare compare);
template<typename RandomAccessIterator>
RandomAccessIterator is_heap_until(RandomAccessIterator first,
		RandomAccessIterator last);

template<typename RandomAccessIterator, typename Compare>
bool is_heap(RandomAccessIterator first, RandomAccessIterator last,
		Compare compare);
template<typename RandomAccessIterator>
bool is_heap(RandomAccessIterator first, RandomAccessIterator last);

//======================================================================
// Added non-standard heap functions
//======================================================================

template<typename RandomAccessIterator, typename Compare>
void update_heap(RandomAccessIterator first, RandomAccessIterator last,
		RandomAccessIterator pos, Compare compare);
template<typename RandomAccessIterator>
void update_heap(RandomAccessIterator first, RandomAccessIterator last,
		RandomAccessIterator pos);

template<typename RandomAccessIterator>
void pop_heap(RandomAccessIterator first, RandomAccessIterator last,
        RandomAccessIterator pos);
template<typename RandomAccessIterator, typename Compare>
void pop_heap(RandomAccessIterator first, RandomAccessIterator last,
        RandomAccessIterator pos, Compare compare);

//======================================================================
// Callback versions
//======================================================================


template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
void pop_heap(RandomAccessIterator first, RandomAccessIterator last,
        RandomAccessIterator pos, Compare compare, MoveCallback moved);

template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
void make_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare, MoveCallback moved);

template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
void push_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare, MoveCallback moved);

template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
void pop_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare, MoveCallback moved);

template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
void sort_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare, MoveCallback moved);

//========================================================================
// Parallel versions
//========================================================================

#if __cplusplus >= 201103L

template<typename RandomAccessIterator, typename Compare>
void parallel_make_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare,
        typename std::iterator_traits<RandomAccessIterator>::difference_type blockSize,
        int maxThreads);

template<typename RandomAccessIterator, typename Compare>
inline void parallel_make_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare) {
    parallel_make_heap<RandomAccessIterator,Compare>(first, last, compare, 32, 0);
}

template<typename RandomAccessIterator>
void parallel_make_heap(RandomAccessIterator first, RandomAccessIterator last,
        typename std::iterator_traits<RandomAccessIterator>::difference_type blockSize,
        int maxThreads);

template<typename RandomAccessIterator>
inline void parallel_make_heap(RandomAccessIterator first, RandomAccessIterator last) {
    parallel_make_heap<RandomAccessIterator>(first, last, 32, 0);
}

template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
void parallel_make_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare, MoveCallback moved,
        typename std::iterator_traits<RandomAccessIterator>::difference_type blockSize,
        int maxThreads);

template<typename RandomAccessIterator, typename Compare, typename MoveCallback>
inline void parallel_make_heap(RandomAccessIterator first, RandomAccessIterator last,
        Compare compare, MoveCallback moved) {
    parallel_make_heap(first, last, compare, moved, 32, 0);
}

#endif

}
}

#include "mas/core/heap.hpp"

#endif
