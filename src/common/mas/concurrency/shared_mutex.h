#ifndef MAS_CONCURRENCY_SHARED_MUTEX
#define MAS_CONCURRENCY_SHARED_MUTEX

// #include <bits/c++config.h>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <climits>
// #include <bits/functexcept.h>

namespace mas {
namespace concurrency {

/// shared_timed_mutex
class shared_timed_mutex {
    // Based on Howard Hinnant's reference implementation from N2406
private:
    typedef std::mutex mutex_t;
    typedef std::condition_variable cond_t;

    mutex_t _mutex;
    cond_t _gate1;
    cond_t _gate2;
    unsigned _state;

    static constexpr unsigned _write_entered = 1U
            << (sizeof(unsigned) * CHAR_BIT - 1);
    static constexpr unsigned _n_readers = ~_write_entered;

public:
    shared_timed_mutex();

    ~shared_timed_mutex();

    shared_timed_mutex(const shared_timed_mutex&) = delete;
    shared_timed_mutex& operator=(const shared_timed_mutex&) = delete;

    // Exclusive ownership
    void lock();
    bool try_lock();

    template<typename Rep, typename Period>
    bool try_lock_for(const std::chrono::duration<Rep, Period>& rel_time);

    template<typename Clock, typename Duration>
    bool try_lock_until(const std::chrono::time_point<Clock, Duration>& abs_time);

    void unlock();

    // Shared ownership

    void lock_shared();
    bool try_lock_shared();

    template<typename Rep, typename Period>
    bool try_lock_shared_for( const std::chrono::duration<Rep, Period>& rel_time);

    template<typename Clock, typename Duration>
    bool try_lock_shared_until(const std::chrono::time_point<Clock, Duration>& abs_time);

    void unlock_shared();
};

/// shared_lock
template<typename Mutex>
class shared_lock {
public:
    typedef Mutex mutex_type;
public:
private:
    Mutex* _mutex;
    bool _owns;

public:
    shared_lock(shared_lock const&) = delete;
    shared_lock& operator=(shared_lock const&) = delete;

    // Shared locking
    shared_lock() noexcept;
    explicit shared_lock(Mutex& m);

    shared_lock(Mutex& m, std::defer_lock_t) noexcept;
    shared_lock(Mutex& m, std::try_to_lock_t);
    shared_lock(Mutex& m, std::adopt_lock_t);

    template<typename Clock, typename Duration>
    shared_lock(Mutex& m, const std::chrono::time_point<Clock, Duration>& abs_time);

    template<typename Rep, typename Period>
    shared_lock(Mutex& m, const std::chrono::duration<Rep, Period>& rel_time);

    ~shared_lock();

    shared_lock(shared_lock&& sl) noexcept;
    shared_lock& operator=(shared_lock&& sl) noexcept;

    void lock();
    bool try_lock();

    template<typename Rep, typename Period>
    bool try_lock_for(const std::chrono::duration<Rep, Period>& rel_time);

    template<typename Clock, typename Duration>
    bool try_lock_until(const std::chrono::time_point<Clock, Duration>& abs_time);

    void unlock();

    // Setters

    void
    swap(shared_lock& sl) noexcept;

    Mutex* release() noexcept;

    // Getters
    bool owns_lock() const noexcept;
    explicit operator bool() const noexcept;

    Mutex* mutex() const noexcept;


};

/// Swap specialization for shared_lock
template<typename Mutex>
void swap(shared_lock<Mutex>& x, shared_lock<Mutex>& y) noexcept;

} // concurrency
} // mas

#include "mas/concurrency/shared_mutex.hpp"

#endif // MAS_CONCURRENCY_SHARED_MUTEX
