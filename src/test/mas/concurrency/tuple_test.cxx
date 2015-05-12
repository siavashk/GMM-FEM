#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "mas/concurrency/bind_simple.hpp"
#include "mas/concurrency/function.hpp"

int regular_function(std::vector<int>&& i) {
	int sum = 0;
	for (int v : i) {
		sum += v;
	}
	return sum;
}

int overloaded_function(int b) {
	return b;
}

int overloaded_function(int b, int c) {
	return b + c;
}

class class_a {
public:
	int a;
	class_a() : a(0) {}
	int operator()(int v) {
		a = a+v;
		return a;
	}
	void add(int v) {
		a += v;
	}
};

class class_c {
	int v;
public:
	class_c(int v) :
			v(v) {
	}

	int member_function_move(std::vector<int>&& l) {
		int sum = v;
		for (int v : l) {
			sum -= v;
		}
		return sum;
	}

	static int static_function(int v) {
		return v;
	}

	int operator()(std::vector<int>&& l) {
		return member_function_move(std::move(l));
	}
};

class class_b {
public:
	int member_object;
	class_c c;
	std::vector<int> b;
	class_b(int k) :
			member_object(k), c(k) {
	}

	int member_function_move(std::vector<int>&& l) {
		int sum = member_object;
		for (int v : l) {
			sum += v;
		}
		b = std::move(l);
		return sum;
	}

	int operator()(std::vector<int>&& l) {
		return member_function_move(std::move(l));
	}
};

int doInvokeTest() {

	std::vector<int> v { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	class_b b { 2 };
	std::unique_ptr<class_b> bp(new class_b { 5 });

	using mas::concurrency::invoke;

	int outa, outb, outc, outd, oute;
	(void)outa;
	(void)outb;
	(void)outc;
	(void)outd;
	(void)oute;
	outa = invoke<int (&)(int, int)>(overloaded_function, 1, 16);
	outa = invoke<int(int, int)>(overloaded_function, 1, 16);
	outa = invoke<int(int)>(overloaded_function, 1);
	outa = invoke(&regular_function, std::move(v));

	outb = invoke(b, std::move(v));
	outb = invoke(std::ref(b), std::move(v));
	outb = invoke(std::move(b), std::move(v));

	outc = invoke(&class_c::static_function,17);

	outd = invoke(&class_b::member_function_move, b, std::move(v));
	outd = invoke(&class_b::member_function_move, &b, std::move(v));
	outd = invoke(&class_b::member_function_move, std::move(b), std::move(v));
	outd = invoke(&class_b::member_function_move, bp, std::move(v));

	oute = invoke(&class_b::member_object, b);
	oute = invoke(&class_b::member_object, &b);

	class_a a;
	invoke(&class_a::add, a, 10);
	std::cout << a.a << std::endl;

	//	std::cout << outa << std::endl;
	//	std::cout << outb << std::endl;
	//	std::cout << outc << std::endl;
	//	std::cout << outd << std::endl;
	//	std::cout << oute << std::endl;

	return 0;

}

void doWrapTest() {
	// auto f = function_wrapper<std::function<int(std::unique_ptr<int>&&)>, std::unique_ptr<int>&&>(regular_function, std::move(pnt));
	// auto f = function_wrapper<std::function<int(std::unique_ptr<int>&&)>, std::unique_ptr<int>&&>(repeat, std::move(pnt));
	// auto f = function_wrapper<decltype(&regular_function),std::vector<int>&&>(regular_function, std::move(member_object));
	// auto f = wrap(&regular_function, std::move(member_object));
	// auto f = wrap(std::move(b), std::move(member_object));
	// auto f = wrap(&class_b::member_function_move,&b, std::move(member_object));
	// auto f = function_wrapper<decltype(&class_b::member_function_move),class_b*,std::vector<int>&&>(&class_b::member_function_move, &b, std::move(member_object));
	// auto f = wrap(&brr::member_function_move, std::move(b), std::move(member_object));

	std::vector<int> v { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
	class_b b { 2 };
	std::unique_ptr<class_b> bp(new class_b { 5 });

	using namespace mas::concurrency;

	// regular
	auto f1 = wrap<int(&)(int, int)>(overloaded_function, 1, 16); f1();
	auto f2 = wrap<int(int, int)>(overloaded_function, 1, 16); f2();
	auto f3 = wrap<int(int)>(overloaded_function, 1); f3();
	auto f4 = wrap(&regular_function, std::move(v)); f4();

	auto f5 = wrap(b, std::move(v)); f5();
	auto f6 = wrap(std::ref(b), std::move(v)); f6();
	auto f7 = wrap(std::move(b), std::move(v)); f7();

	auto f8 = wrap(&class_c::static_function,17); f8();

	print_type_info(&class_b::member_function_move, b, std::move(v));
	std::cout << std::endl;
	print_type_info(&class_b::member_function_move, &b, std::move(v));
	std::cout << std::endl;
	print_type_info(&class_b::member_function_move, std::move(b), std::move(v));
	std::cout << std::endl;
	print_type_info(&class_b::member_function_move, std::move(bp), std::move(v));

//	auto f9  = wrap(&class_b::member_function_move, b, std::move(v)); f9();
	auto f10 = wrap(&class_b::member_function_move, &b, std::move(v)); f10();
//	auto f11 = wrap(&class_b::member_function_move, std::move(b), std::move(v)); f11();
	auto f12 = wrap(&class_b::member_function_move, std::move(bp), std::move(v)); f12();

	class_a a;
	auto f13 = wrap(&class_a::add, &a, 10); f13();
	std::cout << a.a << std::endl;

//	auto f14 = wrap(&class_b::member_object, &b); f14();
	auto f15 = wrap(&class_b::member_object, &b); f15();

}

void doScopeTest() {

	using namespace mas::concurrency;
	std::unique_ptr<function_wrapper_t<int(std::vector<int>&&),std::vector<int>&&>::type> func;
	{
		std::vector<int> v { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
		func = std::move(make_unique_wrapper(regular_function, std::move(v)));
		v.clear();
		v.shrink_to_fit();
	}

	// v should no longer exist, but will the function run?
	int out = func->invoke();
	std::cout << out << std::endl;

}

int main(int argc, char **argv) {
	// doWrapTest();
	// doInvokeTest();
	doScopeTest();
}

