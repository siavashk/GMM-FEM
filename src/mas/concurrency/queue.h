#ifndef MAS_CONCURRENCY_QUEUE_H_
#define MAS_CONCURRENCY_QUEUE_H_

#include <atomic>
#include <mutex>
#include <algorithm>
#include <memory>
#include <deque>

#define MAS_CONCURRENCY_CACHE_LINE_SIZE 64
#define MAS_CONCURRENCY_CACHE_SEPARATE 0

#if MAS_CONCURRENCY_CACHE_SEPARATE > 0
#define ADD_CACHE_PAD(name,space) struct { char p[MAS_CONCURRENCY_CACHE_LINE_SIZE - (space)]; } name
#else
#define ADD_CACHE_PAD(name,space) struct {} name
#endif

namespace mas {
namespace concurrency {

template<typename T>
class threadsafe_queue {
private:
	std::mutex mutex;
	size_t size_ = 0;

	struct node {
		node(T* val) :
				value(val), next(nullptr) {
		}
		T* value;
		std::atomic<node*> next;
		ADD_CACHE_PAD(pad, sizeof(T*)+sizeof(std::atomic<node*>));
	};
	ADD_CACHE_PAD(pad0, sizeof(T*)+sizeof(std::atomic<node*>));

	node* first;
	ADD_CACHE_PAD(pad1,sizeof(node*));

	std::atomic<bool> frontLock;
	ADD_CACHE_PAD(pad2,sizeof(std::atomic<bool>));

	node* last;
	ADD_CACHE_PAD(pad3,sizeof(node*));

	std::atomic<bool> backLock;
	ADD_CACHE_PAD(pad4,sizeof(std::atomic<bool>));

public:
	threadsafe_queue();

	~threadsafe_queue();

	void push_front(const T& t);
	void push_back(const T& t);

	void push_front(T&& t);
	void push_back(T&& t);

	bool pop(T& result);

	size_t size();
};

template<typename T>
class simple_threadsafe_queue {
private:
	std::mutex mutex;
	std::deque<std::unique_ptr<T>> data;

public:
	simple_threadsafe_queue() : data(){}
	~simple_threadsafe_queue(){}

	void push_front(const T& t) {
		std::lock_guard<std::mutex> lk(mutex);
		data.push_front(std::unique_ptr<T>(new T(t)));
	}

	void push_back(const T& t) {
		std::lock_guard<std::mutex> lk(mutex);
		data.push_back(std::unique_ptr<T>(new T(t)));
	}

	void push_front(T&& t) {
		std::lock_guard<std::mutex> lk(mutex);
		data.push_front(std::unique_ptr<T>(new T(std::move(t))));
	}

	void push_back(T&& t) {
		std::lock_guard<std::mutex> lk(mutex);
		data.push_back(std::unique_ptr<T>(new T(std::move(t))));
	}

	bool pop(T& result) {
		std::lock_guard<std::mutex> lk(mutex);
		std::unique_ptr<T> t;
		if (data.size() > 0) {
			t = std::move(data.front());
			data.pop_front();
			result = std::move(*t);
			return true;
		}
		return false;
	}
};

}
}

#include "mas/concurrency/queue.hpp"

#endif /* MAS_CONCURRENCY_QUEUE_H_ */
