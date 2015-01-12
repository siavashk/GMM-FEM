namespace mas {
namespace concurrency {

template<typename T>
threadsafe_queue<T>::threadsafe_queue() {
	first = last = new node(nullptr);
	backLock = frontLock = false;
}

template<typename T>
threadsafe_queue<T>::~threadsafe_queue() {
	while (first != nullptr) {
		node* tmp = first;
		first = tmp->next;
		delete tmp->value;
		delete tmp;
	}
}

template<typename T>
void threadsafe_queue<T>::push_back(const T& t) {
	node* tmp = new node(new T(t));
	while (backLock.exchange(true)) {
	}
	bool didfirstlock = false;

	// in case simultaneously trying to push_front and push_back
	// XXX not sure if this is solved...
	if (last == first) {
		while (frontLock.exchange(true)) {
		}
		// now own both locks, so can't push front and push back simultaneously,
		// over-writing first->next == back->next
		didfirstlock = true;
	}
	last->next = tmp;
	last = tmp;

	if (didfirstlock) {
		frontLock = false;
	}

	backLock = false;
}

template<typename T>
void threadsafe_queue<T>::push_front(const T& t) {
	node* tmp = new node(new T(t));
	while (frontLock.exchange(true)) {
	}
	tmp->next = first->next;
	first->next = tmp;
	frontLock = false;
}

template<typename T>
void threadsafe_queue<T>::push_back(T&& t) {
	node* tmp = new node(new T(std::move(t)));
	while (backLock.exchange(true)) {
	}
	bool didfirstlock = false;
	// in case simultaneously trying to push_front and push_back
	if (last == first) {
		while (frontLock.exchange(true)) {
		}
		// now own both locks, so can't push front and push back simultaneously,
		// over-writing first->next == back->next
		didfirstlock = true;
	}
	last->next = tmp;
	last = tmp;

	if (didfirstlock) {
		frontLock = false;
	}
	backLock = false;
}

template<typename T>
void threadsafe_queue<T>::push_front(T&& t) {
	node* tmp = new node(new T(std::move(t)));
	while (frontLock.exchange(true)) {
	}
	tmp->next.store(first->next);
	first->next.store(tmp);
	frontLock = false;
}

template<typename T>
bool threadsafe_queue<T>::pop(T& result) {

	while (frontLock.exchange(true)) {
	}
	node* f = first;
	node* next = first->next;
	if (next != nullptr) {
		T* val = next->value;
		next->value = nullptr;
		first = next;
		frontLock = false;

		result = std::move_if_noexcept(*val);
		delete val;
		delete f;
		return true;
	}
	frontLock = false;
	return false;
}

}
}
