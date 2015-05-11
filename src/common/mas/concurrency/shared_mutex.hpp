#include <system_error>
#include <utility>

namespace mas {
namespace concurrency {

shared_timed_mutex::shared_timed_mutex() :
        _state(0) {
}

shared_timed_mutex::~shared_timed_mutex() {
    // grab lock on mutex?  GCC just checks that _state == 0
    // std::lock_guard<mutex_t> _lg(_mutex);
}

void shared_timed_mutex::lock() {
    std::unique_lock<mutex_t> lk(_mutex);
    while (_state & _write_entered) {
        _gate1.wait(lk);
    }

    _state |= _write_entered;
    while (_state & _n_readers) {
        _gate2.wait(lk);
    }
}

bool shared_timed_mutex::try_lock() {
    std::unique_lock<mutex_t> lk(_mutex, std::try_to_lock);
    if (lk.owns_lock() && _state == 0) {
        _state = _write_entered;
        return true;
    }
    return false;
}

template<typename Rep, typename Period>
bool shared_timed_mutex::try_lock_for(
        const std::chrono::duration<Rep, Period>& rel_time) {
    std::unique_lock<mutex_t> lk(_mutex, rel_time);
    if (lk.owns_lock() && _state == 0) {
        _state = _write_entered;
        return true;
    }
    return false;
}

template<typename Clock, typename Duration>
bool shared_timed_mutex::try_lock_until(
        const std::chrono::time_point<Clock, Duration>& abs_time) {
    std::unique_lock<mutex_t> lk(_mutex, abs_time);
    if (lk.owns_lock() && _state == 0) {
        _state = _write_entered;
        return true;
    }
    return false;
}

void shared_timed_mutex::unlock() {
    {
        std::lock_guard<mutex_t> lk(_mutex);
        _state = 0;
    }
    _gate1.notify_all();
}

void shared_timed_mutex::lock_shared() {
    std::unique_lock<mutex_t> lk(_mutex);
    while ((_state & _write_entered) || (_state & _n_readers) == _n_readers) {
        _gate1.wait(lk);
    }
    unsigned num_readers = (_state & _n_readers) + 1;
    _state &= ~_n_readers;
    _state |= num_readers;
}

bool shared_timed_mutex::try_lock_shared() {
    std::unique_lock<mutex_t> lk(_mutex, std::try_to_lock);
    unsigned num_readers = _state & _n_readers;
    if (lk.owns_lock() && !(_state & _write_entered)
            && num_readers != _n_readers) {
        ++num_readers;
        _state &= ~_n_readers;
        _state |= num_readers;
        return true;
    }
    return false;
}

template<typename Rep, typename Period>
bool shared_timed_mutex::try_lock_shared_for(
        const std::chrono::duration<Rep, Period>& rel_time) {
    std::unique_lock<mutex_t> lk(_mutex, rel_time);
    if (lk.owns_lock()) {
        unsigned num_readers = _state & _n_readers;
        if (!(_state & _write_entered) && num_readers != _n_readers) {
            ++num_readers;
            _state &= ~_n_readers;
            _state |= num_readers;
            return true;
        }
    }
    return false;
}

template<typename Clock, typename Duration>
bool shared_timed_mutex::try_lock_shared_until(
        const std::chrono::time_point<Clock, Duration>& abs_time) {
    std::unique_lock<mutex_t> lk(_mutex, abs_time);
    if (lk.owns_lock()) {
        unsigned num_readers = _state & _n_readers;
        if (!(_state & _write_entered) && num_readers != _n_readers) {
            ++num_readers;
            _state &= ~_n_readers;
            _state |= num_readers;
            return true;
        }
    }
    return false;
}

void shared_timed_mutex::unlock_shared() {
    std::lock_guard<mutex_t> lk(_mutex);
    unsigned num_readers = (_state & _n_readers) - 1;
    _state &= ~_n_readers;
    _state |= num_readers;
    if (_state & _write_entered) {
        if (num_readers == 0) {
            _gate2.notify_one();
        }
    } else {
        if (num_readers == _n_readers - 1) {
            _gate1.notify_one();
        }
    }
}

template<typename Mutex>
shared_lock<Mutex>::shared_lock() noexcept : _mutex(nullptr), _owns(false) {
}

template<typename Mutex>
shared_lock<Mutex>::shared_lock(mutex_type& m) :
        _mutex(&m), _owns(true) {
    _mutex.lock_shared();
}

template<typename Mutex>
shared_lock<Mutex>::shared_lock(mutex_type& m, std::defer_lock_t) noexcept
: _mutex(&m), _owns(false) {}

template<typename Mutex>
shared_lock<Mutex>::shared_lock(mutex_type& m, std::try_to_lock_t) :
        _mutex(&m), _owns(m.try_lock_shared()) {
}

template<typename Mutex>
shared_lock<Mutex>::shared_lock(mutex_type& m, std::adopt_lock_t) :
        _mutex(&m), _owns(true) {
}
template<typename Mutex>
template<typename Clock, typename Duration>
shared_lock<Mutex>::shared_lock(mutex_type& m,
        const std::chrono::time_point<Clock, Duration>& abs_time) :
        _mutex(&m), _owns(m.try_lock_shared_until(abs_time)) {
}

template<typename Mutex>
template<typename Rep, typename Period>
shared_lock<Mutex>::shared_lock(mutex_type& m,
        const std::chrono::duration<Rep, Period>& rel_time) :
        _mutex(&m), _owns(m.try_lock_shared_for(rel_time)) {
}

template<typename Mutex>
shared_lock<Mutex>::~shared_lock() {
    if (_owns) {
        _mutex->unlock_shared();
    }
}

template<typename Mutex>
shared_lock<Mutex>::shared_lock(shared_lock&& sl) noexcept :
shared_lock() {
    swap(sl);
}

template<typename Mutex>
shared_lock<Mutex>& shared_lock<Mutex>::operator=(shared_lock<Mutex>&& sl) noexcept {
    shared_lock<Mutex>(std::move(sl)).swap(*this);
    return *this;
}


template<typename Mutex>
void shared_lock<Mutex>::lock() {
    _mutex->lock_shared();
    _owns = true;
}

template<typename Mutex>
bool shared_lock<Mutex>::try_lock() {
    return _owns = _mutex->try_lock_shared();
}

template<typename Mutex>
template<typename Rep, typename Period>
bool shared_lock<Mutex>::try_lock_for(const std::chrono::duration<Rep, Period>& rel_time) {
    return _owns = _mutex->try_lock_shared_for(rel_time);
}

template<typename Mutex>
template<typename Clock, typename Duration>
bool shared_lock<Mutex>::try_lock_until(const std::chrono::time_point<Clock, Duration>& abs_time) {
    return _owns = _mutex->try_lock_shared_until(abs_time);
}

template<typename Mutex>
void shared_lock<Mutex>::unlock() {
    if (!_owns) {
        throw std::system_error(int(std::errc::resource_deadlock_would_occur));
    }
    _mutex->unlock_shared();
    _owns = false;
}

// Setters
template<typename Mutex>
void shared_lock<Mutex>::swap(shared_lock& sl) noexcept {
    std::swap(_mutex, sl._mutex);
    std::swap(_owns, sl._owns);
}

template<typename Mutex>
typename shared_lock<Mutex>::mutex_type* shared_lock<Mutex>::release() noexcept {
    mutex_type* r = _mutex;
    _mutex = nullptr;
    _owns = false;
    return r;
}

// Getters
template<typename Mutex>
bool shared_lock<Mutex>::owns_lock() const noexcept {
    return _owns;
}

template<typename Mutex>
shared_lock<Mutex>::operator bool() const noexcept {
    return _owns;
}

template<typename Mutex>
typename shared_lock<Mutex>::mutex_type* shared_lock<Mutex>::mutex() const noexcept {
    return _mutex;
}

/// Swap specialization for shared_lock
template<typename Mutex>
void swap(shared_lock<Mutex>& x, shared_lock<Mutex>& y) noexcept {
    x.swap(y);
}

} // concurrency
} // mas
