#include <tuple>
#include <functional>
#include <future>
#include <memory>
#include <iostream>

namespace mas {
namespace concurrency {

namespace type_traits {
template<typename T>
struct target_type {
	typedef void type;
};

template<typename Class, typename Member>
struct target_type<Member Class::*> {
	typedef Class type;
};

//Is reference to pointer target or derived
template<typename Object, typename Pointer>
struct is_target_reference: public std::integral_constant<bool,
		std::is_reference<Object>::value
				&& std::is_base_of<typename target_type<Pointer>::type,
						typename std::decay<Object>::type>::value> {
};
}

// Member function, by reference
template<typename Functor, typename Object, typename ... Args>
inline typename std::enable_if<
		std::is_member_function_pointer<typename std::decay<Functor>::type>::value
				&& type_traits::is_target_reference<Object&&,
						typename std::decay<Functor>::type>::value,

		typename std::result_of<Functor(Object,Args&&...)>::type>::type invoke(
		Functor&& functor, Object&& object, Args&&... args) {
	return (object.*functor)(std::forward<Args>(args)...);
}

// Member function, by pointer
template<typename Functor, typename Object, typename ... Args>
inline typename std::enable_if<
		std::is_member_function_pointer<typename std::decay<Functor>::type>::value
			&& !type_traits::is_target_reference<Object&&,
				typename std::decay<Functor>::type>::value,

		typename std::result_of<Functor(Object,Args&&...)>::type>::type invoke(
		Functor&& functor, Object&& object,	Args&&... args) {
	return ((*std::forward<Object>(object)).*functor)(	std::forward<Args>(args)...);
}

// Member object, by reference
template<typename Functor, typename Object>
inline typename std::enable_if<
		std::is_member_object_pointer<typename std::decay<Functor>::type>::value
			&& type_traits::is_target_reference<Object&&,
				typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor(Object)>::type>::type
		invoke(Functor&& functor, Object&& object) {
	return object.*functor;
}

// member object, by pointer
template<typename Functor, typename Object>
inline typename std::enable_if<
		std::is_member_object_pointer<typename std::decay<Functor>::type>::value
			&& !type_traits::is_target_reference<Object&&,
				typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor(Object)>::type>::type
		invoke(Functor&& functor, Object&& object) {
	return (*std::forward<Object>(object)).*functor;
}

// UNKNOWN!!!
template<typename Functor, typename ... Args>
inline typename std::enable_if<
		std::is_member_pointer<typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor(Args...)>::type>::type
		invoke(Functor&& functor, Args&&... args) {
	std::cout << "WHAT IS THIS?" << std::endl;
	return std::forward<Functor>(functor)(std::forward<Args>(args)...);
}

// Function/objects with operator(...)
template<typename Functor, typename ... Args>
inline typename std::enable_if<
		!std::is_member_pointer<typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor&(Args...)>::type>::type
		invoke(Functor&& functor, Args&&... args) {
	std::cout << "REGULAR FUNCTION" << std::endl;
	return std::forward<Functor>(functor)(std::forward<Args>(args)...);
}

// Index tuple for accessing arguments
template<std::size_t ... Indexes>
struct index_tuple {
	typedef index_tuple<Indexes..., sizeof...(Indexes)> next;
};

// Builds an index_tuple<0, 1, 2, ..., Num-1>.
template<std::size_t Num>
struct build_index_tuple {
	typedef typename build_index_tuple<Num - 1>::type::next type;
};

template<>
struct build_index_tuple<0> {
	typedef index_tuple<> type;
};

//template<typename Signature>
//class function_wrapper;

template<typename Signature>
class fun_type;

template<typename Fn, typename ... Args>
class function_wrapper {
public:
	typedef typename std::result_of<Fn(Args...)>::type result_type;

private:
	std::tuple<Fn, Args&&...> bound;

	template<std::size_t ... Indices>
	result_type invoke(index_tuple<Indices...>) {
		// std::bind always forwards bound arguments as lvalues,
		// but this type can call functions which only accept rvalues.
		//return std::forward<Fn>(std::get<0>(bound))(std::forward<Args>(std::get<Indices+1>(bound))...);
		return mas::concurrency::invoke(std::forward<Fn>(std::get<0>(bound)), std::forward<Args>(std::get<Indices>(bound))...);
	}

public:

	//template<typename ...Args2, typename = typename std::enable_if<sizeof...(Args) == sizeof...(Args2)>::type>
	explicit function_wrapper(const Fn& fn, Args&&... args) :
			bound(fn, std::forward<Args>(args)...) {
	}

	//template<typename... Args2, typename = typename std::enable_if< sizeof...(Args) == sizeof...(Args2)>::type>
	explicit function_wrapper(Fn&& fn, Args&&... args) :
			bound(std::move(fn), std::forward<Args>(args)...) {
	}

	function_wrapper(const function_wrapper&) = delete;
	function_wrapper(function_wrapper&&) = default;

	result_type operator()() {
		return invoke();
	}

	result_type invoke() {
		typedef typename build_index_tuple<sizeof...(Args)>::type Indices;
		return invoke(Indices());
	}
};

template<typename Fn, typename ... Args>
function_wrapper<Fn, Args...> wrap(Fn&& fn, Args&&... args) {
	return function_wrapper<Fn, Args...>(std::move(fn),	std::forward<Args>(args)...);
}

template<typename Fn, typename ... Args>
function_wrapper<Fn, Args...> wrap(const Fn& fn, Args&&... args) {
	return function_wrapper<Fn, Args...>(fn, std::forward(args)...);
}

template<typename Fn, typename ... Args>
class future_function_wrapper {
	typedef typename std::result_of<Fn(Args...)>::type result_type;

private:
	std::tuple<Fn, Args&&...> bound;
	std::promise<result_type> promise;

	template<std::size_t ... Indices>
	void invoke(index_tuple<Indices...>) {
		// std::bind always forwards bound arguments as lvalues,
		// but this type can call functions which only accept rvalues.
		//return std::forward<Fn>(std::get<0>(bound))(std::forward<Args>(std::get<Indices+1>(bound))...);
		promise.set_value(
				mas::concurrency::invoke(std::forward<Fn>(std::get<0>(bound)),std::forward<Args>(std::get<Indices+1>(bound))...));
	}

public:
	explicit future_function_wrapper(const Fn& fn, Args... args) :
		bound(fn, std::forward<Args>(args)...) {
	}

	explicit future_function_wrapper(Fn&& fn, Args&&... args) :
			bound(std::move(fn), std::forward<Args>(args)...) {
	}

	future_function_wrapper(const future_function_wrapper&) = delete;
	future_function_wrapper(future_function_wrapper&&) = default;

	void operator()() {
		invoke();
	}

	void invoke() {
		typedef typename build_index_tuple<sizeof...(Args)>::type Indices;
		invoke(Indices());
	}

	std::future<result_type> get_future() {
		return promise.get_future();
	}

};

//class future_function_wrapper {
//	struct ffw_base {
//		virtual void invoke() = 0;
//	};
//
//	template<typename Fn, typename ... Args>
//	struct ffw : ffw_base {
//		typedef typename function_wrapper<Fn,Args...>::result_type result_type;
//		function_wrapper<Fn,Args...> f;
//		std::promise<result_type> p;
//
//		ffw(const Fn& fn, Args&&... args) {
//			f = wrap(fn, std::forward<Args>(args)...);
//		}
//
//		ffw(Fn&& fn, Args&&... args) {
//			f = wrap(std::move(fn), std::forward<Args>(args)...);
//		}
//
//		ffw(function_wrapper<Fn,Args...>&& fw, std::promise<result_type>&& promise) :
//			f(std::move(fw)), p(std::move(promise)){
//		}
//
//		std::future<result_type> get_future() {
//			return p.get_future();
//		}
//
//		void invoke() {
//			p.set(f.invoke());
//		}
//	};
//
//	std::unique_ptr<ffw_base> f;
//
//public:
//	template<typename Fn, typename... Args>
//	explicit future_function_wrapper(
//			std::future<typename function_wrapper<Fn,Args...>::result_type>& future,
//			Fn&& fn, Args&&... args) {
//		auto ff = new ffw<Fn,Args...>(std::move(fn), std::forward<Args>(args)...);
//		future = std::move(ff->get_future());
//		f = std::unique_ptr<ffw_base>(ff);
//	}
//
//	template<typename Fn, typename... Args>
//	explicit future_function_wrapper(
//			std::future<typename function_wrapper<Fn,Args...>::result_type>& future,
//			const Fn& fn, Args&&... args) {
//		auto ff = new ffw<Fn,Args...>(fn, std::forward<Args>(args)...);
//		future = std::move(ff->get_future());
//		f = std::unique_ptr<ffw_base>(ff);
//	}
//
//	template<typename Fn, typename... Args>
//		explicit future_function_wrapper(
//				function_wrapper<Fn,Args...>&& fw,
//				std::promise<typename function_wrapper<Fn,Args...>::result_type>&& p) {
//		f = std::unique_ptr<ffw_base>(new ffw<Fn,Args...>(std::move(fw), std::move(p)));
//	}
//
//	void invoke() {
//		f->invoke();
//	}
//
//};


template<typename Fn, typename ... Args>
future_function_wrapper<Fn, Args...> wrap_future(Fn&& fn, Args&&... args) {
	return future_function_wrapper<Fn, Args...>(std::move(fn), std::forward<Args>(args)...);
}

template<typename Fn, typename ... Args>
future_function_wrapper<Fn, Args...> wrap_future(const Fn& fn, Args&&... args) {
	return future_function_wrapper<Fn, Args...>(fn, std::forward(args)...);
}

}
}
