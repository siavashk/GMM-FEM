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

//is reference to pointer target or derived
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
	std::cout << "MEMBER FUNCTION, BY REFERENCE" << std::endl;
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
	std::cout << "MEMBER FUNCTION, BY POINTER" << std::endl;
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
	std::cout << "MEMBER OBJECT, BY REFERENCE" << std::endl;
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
	std::cout << "MEMBER OBJECT, BY POINTER" << std::endl;
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
		typename std::result_of<Functor&(Args&&...)>::type>::type
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
	typedef typename std::result_of<Fn&(Args&&...)>::type result_type;

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
class future_function_wrapper {
	typedef typename std::result_of<Fn(Args...)>::type result_type;

private:
	std::tuple<Fn, Args&&...> bound;
	std::promise<result_type> promise;

	template<std::size_t ... Indices>
	void invoke(index_tuple<Indices...>) {
		// std::bind always forwards bound arguments as lvalues,
		// but this type can call functions which accept rvalues.
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

template<typename Tp>
struct maybe_wrap_pointer {
    typedef Tp type;

    static const Tp&  __do_wrap(const Tp& x) {
        return x;
    }

    static Tp&& __do_wrap(Tp&& x) {
        return static_cast<Tp&&>(x);
    }
};

template<>
struct maybe_wrap_pointer<void> {
    typedef void type;
};

template<typename Fn, typename... Args>
struct wrap_helper {
    typedef maybe_wrap_pointer<typename std::decay<Fn>::type> maybe_type;
    typedef typename maybe_type::type fn_type;
    typedef function_wrapper<fn_type(typename std::decay<Args>::type...)> type;
};


//// Simplified version of std::bind for internal use, without support for
//// unbound arguments, placeholders or nested bind expressions.
//template<typename _Callable, typename ... _Args>
//typename detail::_Bind_simple_helper<_Callable, _Args...>::__type bind_simple(
//        _Callable&& __callable, _Args&&... __args) {
//    typedef detail::_Bind_simple_helper<_Callable, _Args...> __helper_type;
//    typedef typename __helper_type::__maybe_type __maybe_type;
//    typedef typename __helper_type::__type __result_type;
//    return __result_type(
//            __maybe_type::__do_wrap(std::forward<_Callable>(__callable)),
//            std::forward<_Args>(__args)...);
//}

template<typename Fn, typename ... Args>
typename wrap_helper<Fn,Args...>::type wrap(Fn&& fn, Args&&... args) {
	typedef wrap_helper<Fn, Args...> helper_type;
	typedef typename helper_type::maybe_type maybe_type;
	typedef typename helper_type::type result_type;
	return result_type(
			maybe_type::__do_wrap(std::forward<Fn>(fn),std::forward<Args>(args)...));
	//return function_wrapper<Fn, Args...>(std::forward<Fn>(fn),std::forward<Args>(args)...);
}

//template<typename Fn, typename ... Args>
//function_wrapper<Fn, Args...> wrap(const Fn& fn, Args&&... args) {
//	return function_wrapper<Fn, Args...>(fn, std::forward<Args>(args)...);
//}

template<typename Fn, typename ... Args>
future_function_wrapper<Fn, Args...> wrap_future(Fn&& fn, Args&&... args) {
	return future_function_wrapper<Fn, Args...>(std::move(fn), std::forward<Args>(args)...);
}

//template<typename Fn, typename ... Args>
//future_function_wrapper<Fn, Args...> wrap_future(const Fn& fn, Args&&... args) {
//	return future_function_wrapper<Fn, Args...>(fn, std::forward<Args>(args)...);
//}

}
}
