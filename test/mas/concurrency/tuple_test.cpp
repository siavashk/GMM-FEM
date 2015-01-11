#include <iostream>
#include <memory>
#include <utility>
#include <tuple>
#include <thread>
#include <functional>
#include <string>
#include <vector>

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

template<typename Functor, typename Object, typename ... Args>
inline typename std::enable_if<
		std::is_member_function_pointer<typename std::decay<Functor>::type>::value
				&& type_traits::is_target_reference<Object&&,
						typename std::decay<Functor>::type>::value,

		typename std::result_of<Functor(Object,Args&&...)>::type>::type invoke(
		Functor&& functor, Object&& object, Args&&... args) {
	std::cout << "MEMBER FUNCTION, REFERENCE" << std::endl;
	return (object.*functor)(std::forward<Args>(args)...);
}

template<typename Functor, typename Object, typename ... Args>
inline typename std::enable_if<
		std::is_member_function_pointer<typename std::decay<Functor>::type>::value
			&& !type_traits::is_target_reference<Object&&,
				typename std::decay<Functor>::type>::value,

		typename std::result_of<Functor(Object,Args&&...)>::type>::type invoke(
		Functor&& functor, Object&& object,	Args&&... args) {
	std::cout << "MEMBER FUNCTION, POINTER" << std::endl;
	return ((*std::forward<Object>(object)).*functor)(	std::forward<Args>(args)...);
}

template<typename Functor, typename Object>
inline typename std::enable_if<
		std::is_member_object_pointer<typename std::decay<Functor>::type>::value
			&& type_traits::is_target_reference<Object&&,
				typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor(Object)>::type>::type
		invoke(Functor&& functor, Object&& object) {
	std::cout << "MEMBER OBJECT, REFERENCE" << std::endl;
	return object.*functor;
}

template<typename Functor, typename Object>
inline typename std::enable_if<
		std::is_member_object_pointer<typename std::decay<Functor>::type>::value
			&& !type_traits::is_target_reference<Object&&,
				typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor(Object)>::type>::type
		invoke(Functor&& functor, Object&& object) {
	std::cout << "MEMBER OBJECT, POINTER" << std::endl;
	return (*std::forward<Object>(object)).*functor;
}

template<typename Functor, typename ... Args>
inline typename std::enable_if<
		std::is_member_pointer<typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor(Args...)>::type>::type
		invoke(Functor&& functor, Args&&... args) {
	std::cout << "WHAT IS THIS?" << std::endl;
	return std::forward<Functor>(functor)(std::forward<Args>(args)...);
}

template<typename Functor, typename ... Args>
inline typename std::enable_if<
		!std::is_member_pointer<typename std::decay<Functor>::type>::value,
		typename std::result_of<Functor&(Args...)>::type>::type
		invoke(Functor&& functor, Args&&... args) {
	std::cout << "REGULAR FUNCTION" << std::endl;
	return std::forward<Functor>(functor)(std::forward<Args>(args)...);
}

///**
// * Invoke a function object, which may be either a member pointer or a
// * function object. The first parameter will tell which.
// */
//template<typename _Functor, typename ... _Args>
//inline typename std::enable_if<
//(		!std::is_member_pointer<_Functor>::value
//		&& !std::is_function<_Functor>::value
//		&& !std::is_function<typename std::remove_pointer<_Functor>::type>::value
//		&& !std::is_function<typename std::remove_reference<_Functor>::type>::value),
//		typename std::result_of<_Functor&(_Args&&...)>::type>::type __invoke_(
//				_Functor&& __f, _Args&&... __args) {
//	std::cout << "Operator" << std::endl;
//	return __f(std::forward<_Args>(__args)...);
//}
//
//template<typename _Functor, typename ... _Args>
//inline typename std::enable_if<
//(std::is_member_pointer<_Functor>::value
//		&& !std::is_function<_Functor>::value
//		&& !std::is_function<
//		typename std::remove_pointer<_Functor>::type>::value),
//		typename std::result_of<_Functor(_Args&&...)>::type>::type __invoke_(
//				_Functor& __f, _Args&&... __args) {
//	std::cout << "Member operator?" << std::endl;
//	return std::mem_fn(__f)(std::forward<_Args>(__args)...);
//}
//
//template<typename _Functor, typename ... _Args>
//inline typename std::enable_if<
//(std::is_member_function_pointer<_Functor>::value),
//typename std::result_of<_Functor(_Args&&...)>::type>::type __invoke_(
//		_Functor&& __f, _Args&&... __args) {
//	std::cout << "Member function" << std::endl;
//	return std::mem_fn(__f)(std::forward<_Args>(__args)...);
//}
//
	template<typename Functor, typename Object, typename ... Args>
	void __test_(Functor&& __f, Object&& object,Args&&... __args) {
		std::cout << "Member function: " << std::is_member_function_pointer<Functor>::value << std::endl;
		std::cout << "Decayed function pointer: " << std::is_member_function_pointer<typename std::decay<Functor>::type>::value << std::endl;
		std::cout << "Target reference: " << type_traits::is_target_reference<Object&&, typename std::decay<Functor>::type>::value << std::endl;
	}
//
//// To pick up function references (that will become function pointers)
//template<typename _Functor, typename ... _Args>
//inline typename std::enable_if<
//(std::is_pointer<_Functor>::value
//		&& std::is_function<typename std::remove_pointer<_Functor>::type>::value),
//		typename std::result_of<_Functor(_Args&&...)>::type>::type __invoke_(
//				_Functor __f, _Args&&... __args) {
//	std::cout << "Function" << std::endl;
//	return __f(std::forward<_Args>(__args)...);
//}

	int repeat(std::vector<int>&& i) {
		int sum = 0;
		for (int v : i) {
			sum += v;
		}
		i.clear();
		return sum;
	}

//	int repeat(std::vector<int> i) {
//		int sum = 0;
//		for (int v : i) {
//			sum += v;
//		}
//		i.clear();
//		return sum;
//	}

	class crr {
		int v;
	public:
		crr(int v) :
				v(v) {
		}

		int blah(std::vector<int>&& l) {
			int sum = v;
			for (int v : l) {
				sum -= v;
			}
			return sum;
		}

		int operator()(std::vector<int>&& l) {
			return blah(std::move(l));
		}
	};

	class brr {
	public:
		int v;
		crr c;
		std::vector<int> b;
		brr(int k) :
				v(k), c(k) {
		}

		int blah(std::vector<int>&& l) {
			int sum = v;
			for (int v : l) {
				sum += v;
			}
			b = std::move(l);
			return sum;
		}

		int operator()(std::vector<int>&& l) {
			return blah(std::move(l));
		}
	};

//template <class T>
//struct is_member_pointer
//{
//  template <class X, class Y>
//  static char is_ptr(Y X::*);
//  static double is_ptr(...);
//
//  static T t;
//  enum { value = sizeof(is_ptr(t)) == sizeof(char) };
//};

template<typename X, typename Y>
bool is_member_pointer(Y X::*) {
	return true;
}

template<typename X>
bool is_member_pointer(X x) {
	return false;
}

int fa(int b) {
	return b;
}

int fa(int b, int c) {
	return b+c;
}

int doWrapTest() {

	int i = 10;
	std::vector<int> v {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	brr b {2};
	std::unique_ptr<brr> bp(new brr {5});

	// auto f = function_wrapper<std::function<int(std::unique_ptr<int>&&)>, std::unique_ptr<int>&&>(repeat, std::move(pnt));
	// auto f = function_wrapper<std::function<int(std::unique_ptr<int>&&)>, std::unique_ptr<int>&&>(repeat, std::move(pnt));
	// auto f = function_wrapper<decltype(&repeat),std::vector<int>&&>(repeat, std::move(v));
	// auto f = wrap(&repeat, std::move(v));
	// auto f = wrap(std::move(b), std::move(v));
	// auto f = wrap(&brr::blah,&b, std::move(v));
	// auto f = function_wrapper<decltype(&brr::blah),brr*,std::vector<int>&&>(&brr::blah, &b, std::move(v));
	// auto f = wrap(&brr::blah, std::move(b), std::move(v));

	int outa, outb, outc, outd, oute;
	outa = invoke<int(&)(int,int)>(fa, 1, 16);
	outa = invoke<int(int,int)>(fa, 1, 16);
	outa = invoke<int(int)>(fa, 1);
	outa = invoke(&repeat, std::move(v));

	outb = invoke(b, std::move(v));
	outb = invoke(std::ref(b), std::move(v));
	outc = invoke(std::move(b), std::move(v));

	outd = invoke(&brr::blah, b, std::move(v));
	outd = invoke(&brr::blah, &b, std::move(v));
	outd = invoke(&brr::blah, std::move(b), std::move(v));
	outd = invoke(&brr::blah, bp, std::move(v));
	oute = invoke(&brr::v, b);
	oute = invoke(&brr::v, &b);

	//__test_(&brr::blah, b, std::move(v));
	//bool t = std::is_member_function_pointer<decltype(&brr::blah)>::value;
	//std::cout << t << std::endl;

	std::cout << outa << std::endl;
	std::cout << outb << std::endl;
	std::cout << outc << std::endl;
	std::cout << outd << std::endl;


	return 0;

}

int main(int argc, char **argv) {
	doWrapTest();
}

