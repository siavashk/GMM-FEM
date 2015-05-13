// Taken from GCC 4.8.2 <functional> and <type_traits>
//
// Copyright (C) 2001-2013 Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

/*
 * Copyright (c) 1997
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 */

#include <tuple>
#include <type_traits>

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#define CONSTEXPR const
#else
#define NOEXCEPT
#define CONSTEXPR
#endif

namespace mas {
namespace concurrency {
namespace detail {

// Meta programming helper types.
template<typename ...>
struct __or_;

template<>
struct __or_<> : public std::false_type {
};

template<typename _B1>
struct __or_<_B1> : public _B1 {
};

template<typename _B1, typename _B2>
struct __or_<_B1, _B2> : public std::conditional<_B1::value, _B1, _B2>::type {
};

template<typename _B1, typename _B2, typename _B3, typename ... _Bn>
struct __or_<_B1, _B2, _B3, _Bn...> : public std::conditional<_B1::value, _B1,
        __or_ <_B2, _B3, _Bn...>>::type {
};

template<typename ...>
struct __and_;

template<>
struct __and_<> : public std::true_type {
};

template<typename _B1>
struct __and_<_B1> : public _B1 {
};

template<typename _B1, typename _B2>
struct __and_<_B1, _B2> : public std::conditional<_B1::value, _B2, _B1>::type {
};

template<typename _B1, typename _B2, typename _B3, typename ... _Bn>
struct __and_<_B1, _B2, _B3, _Bn...> : public std::conditional<_B1::value,
        __and_ <_B2, _B3, _Bn...>, _B1>::type {
};

template<typename _Pp>
struct __not_: public std::integral_constant<bool, !_Pp::value> {
};

struct __sfinae_types {
    typedef char __one;
    typedef struct {
        char __arr[2];
    } __two;
};

template<typename ... _Cond>
using _Require = typename std::enable_if<__and_<_Cond...>::value>::type;

// Stores a tuple of indices.  Also used by bind() to extract the elements
// in a tuple.
template<std::size_t ... _Indexes>
struct _Index_tuple {
    typedef _Index_tuple<_Indexes..., sizeof...(_Indexes)> __next;
};

// Builds an _Index_tuple<0, 1, 2, ..., _Num-1>.
template<std::size_t _Num>
struct _Build_index_tuple {
    typedef typename _Build_index_tuple<_Num - 1>::__type::__next __type;
};

template<>
struct _Build_index_tuple<0> {
    typedef _Index_tuple<> __type;
};

template<typename _MemberPointer>
class _Mem_fn;
template<typename _Tp, typename _Class>
_Mem_fn<_Tp _Class::*>
mem_fn(_Tp _Class::*) NOEXCEPT;

template<typename _Tp>
class __has_result_type_helper: __sfinae_types {
    template<typename _Up>
    struct _Wrap_type {
    };

    template<typename _Up>
    static __one         __test(_Wrap_type<typename _Up::_NTYPE>*);
    template<typename _Up>
    static __two         __test(...);
public:
    static CONSTEXPR bool value = sizeof(__test<_Tp>(0)) == 1;
};

template<typename _Tp>
struct __has_result_type: std::integral_constant<bool,
        __has_result_type_helper<typename std::remove_cv<_Tp>::type>::value> {
};

/// If we have found a result_type, extract it.
template<bool _Has_result_type, typename _Functor>
struct _Maybe_get_result_type {
};

template<typename _Functor>
struct _Maybe_get_result_type<true, _Functor> {
    typedef typename _Functor::result_type result_type;
};

/**
 *  Base class for any function object that has a weak result type, as
 *  defined in 3.3/3 of TR1.
 */
template<typename _Functor>
struct _Weak_result_type_impl: _Maybe_get_result_type<
        __has_result_type<_Functor>::value, _Functor> {
};

/// Retrieve the result type for a function type.
template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...)> {
    typedef _Res result_type;
};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...,...)>
{   typedef _Res result_type;};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...) const> {
    typedef _Res result_type;
};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...,...) const>
{   typedef _Res result_type;};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...) volatile> {
    typedef _Res result_type;
};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...,...) volatile>
{   typedef _Res result_type;};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...) const volatile> {
    typedef _Res result_type;
};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res(_ArgTypes...,...) const volatile>
{   typedef _Res result_type;};

/// Retrieve the result type for a function reference.
template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (&)(_ArgTypes...)> {
    typedef _Res result_type;
};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (&)(_ArgTypes...,...)>
{   typedef _Res result_type;};

/// Retrieve the result type for a function pointer.
template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (*)(_ArgTypes...)> {
    typedef _Res result_type;
};

template<typename _Res, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (*)(_ArgTypes...,...)>
{   typedef _Res result_type;};

/// Retrieve result type for a member function pointer.
template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...)> {
    typedef _Res result_type;
};

template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...,...)>
{   typedef _Res result_type;};

/// Retrieve result type for a const member function pointer.
template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...) const> {
    typedef _Res result_type;
};

template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...,...) const>
{   typedef _Res result_type;};

/// Retrieve result type for a volatile member function pointer.
template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...) volatile> {
    typedef _Res result_type;
};

template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...,...) volatile>
{   typedef _Res result_type;};

/// Retrieve result type for a const volatile member function pointer.
template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...) const volatile> {
    typedef _Res result_type;
};

template<typename _Res, typename _Class, typename ... _ArgTypes>
struct _Weak_result_type_impl<_Res (_Class::*)(_ArgTypes...,...)
const volatile>
{   typedef _Res result_type;};

/**
 *  Strip top-level cv-qualifiers from the function object and let
 *  _Weak_result_type_impl perform the real work.
 */
template<typename _Functor>
struct _Weak_result_type: _Weak_result_type_impl<
        typename std::remove_cv<_Functor>::type> {
};

/// Determines if the type _Tp derives from unary_function.
template<typename _Tp>
struct _Derives_from_unary_function: __sfinae_types {
private:
    template<typename _T1, typename _Res>
    static __one         __test(const volatile std::unary_function<_T1, _Res>*); // It's tempting to change "..." to const volatile void*, but
    // that fails when _Tp is a function type.
    static __two         __test(...);
public:
    static const bool value = sizeof(__test((_Tp*) 0)) == 1;
};

/// Determines if the type _Tp derives from binary_function.
template<typename _Tp>
struct _Derives_from_binary_function: __sfinae_types {
private:
    template<typename _T1, typename _T2, typename _Res>
    static __one         __test(const volatile std::binary_function<_T1, _T2, _Res>*); // It's tempting to change "..." to const volatile void*, but
    // that fails when _Tp is a function type.
    static __two         __test(...);
public:
    static const bool value = sizeof(__test((_Tp*) 0)) == 1;
};

/**
 * Invoke a function object, which may be either a member pointer or a
 * function object. The first parameter will tell which.
 */
template<typename _Functor, typename ... _Args>
inline typename std::enable_if<
        (		!std::is_member_pointer<_Functor>::value
        		&& !std::is_function<_Functor>::value
                && !std::is_function<typename std::remove_pointer<_Functor>::type>::value),
        typename std::result_of<_Functor&(_Args&&...)>::type>::type __invoke_(
        _Functor& __f, _Args&&... __args) {
    return __f(std::forward<_Args>(__args)...);
}

template<typename _Functor, typename ... _Args>
inline typename std::enable_if<
        (std::is_member_pointer<_Functor>::value
                && !std::is_function<_Functor>::value
                && !std::is_function<
                        typename std::remove_pointer<_Functor>::type>::value),
        typename std::result_of<_Functor(_Args&&...)>::type>::type __invoke_(
        _Functor& __f, _Args&&... __args) {
    return std::mem_fn(__f)(std::forward<_Args>(__args)...);
}

// To pick up function references (that will become function pointers)
template<typename _Functor, typename ... _Args>
inline typename std::enable_if<
        (std::is_pointer<_Functor>::value
                && std::is_function<typename std::remove_pointer<_Functor>::type>::value),
        typename std::result_of<_Functor(_Args&&...)>::type>::type __invoke_(
        _Functor __f, _Args&&... __args) {
    return __f(std::forward<_Args>(__args)...);
}

/**
 *  Knowing which of unary_function and binary_function _Tp derives
 *  from, derives from the same and ensures that reference_wrapper
 *  will have a weak result type. See cases below.
 */
template<bool _Unary, bool _Binary, typename _Tp>
struct _Reference_wrapper_base_impl;

// None of the nested argument types.
template<typename _Tp>
struct _Reference_wrapper_base_impl<false, false, _Tp> : _Weak_result_type<_Tp> {
};

// Nested argument_type only.
template<typename _Tp>
struct _Reference_wrapper_base_impl<true, false, _Tp> : _Weak_result_type<_Tp> {
    typedef typename _Tp::argument_type argument_type;
};

// Nested first_argument_type and second_argument_type only.
template<typename _Tp>
struct _Reference_wrapper_base_impl<false, true, _Tp> : _Weak_result_type<_Tp> {
    typedef typename _Tp::first_argument_type first_argument_type;
    typedef typename _Tp::second_argument_type second_argument_type;
};

// All the nested argument types.
template<typename _Tp>
struct _Reference_wrapper_base_impl<true, true, _Tp> : _Weak_result_type<_Tp> {
    typedef typename _Tp::argument_type argument_type;
    typedef typename _Tp::first_argument_type first_argument_type;
    typedef typename _Tp::second_argument_type second_argument_type;
};

template<typename _Tp>
class __has_argument_type_helper: __sfinae_types {
    template<typename _Up>
    struct _Wrap_type {
    };

    template<typename _Up>
    static __one        __test(_Wrap_type<typename _Up::_NTYPE>*);
    template<typename _Up>
    static __two        __test(...);
public:
    static CONSTEXPR bool value = sizeof(__test<_Tp>(0)) == 1;
};

template<typename _Tp>
struct __has_argument_type: std::integral_constant<bool,
        __has_argument_type_helper<typename std::remove_cv<_Tp>::type>::value> {
};

template<typename _Tp>
class __has_first_argument_type_helper: __sfinae_types {
    template<typename _Up>
    struct _Wrap_type {
    };

    template<typename _Up>
    static __one        __test(_Wrap_type<typename _Up::_NTYPE>*);
    template<typename _Up>
    static __two        __test(...);
public:
    static constexpr bool value = sizeof(__test<_Tp>(0)) == 1;
};

template<typename _Tp>
struct __has_first_argument_type: std::integral_constant<bool,
        __has_first_argument_type_helper<typename std::remove_cv<_Tp>::type>::value> {
};

template<typename _Tp>
class __has_second_argument_type_helper: __sfinae_types {
    template<typename _Up>
    struct _Wrap_type {
    };

    template<typename _Up>
    static __one        __test(_Wrap_type<typename _Up::_NTYPE>*);
    template<typename _Up>
    static __two        __test(...);
public:
    static constexpr bool value = sizeof(__test<_Tp>(0)) == 1;
};

template<typename _Tp>
struct __has_second_argument_type: std::integral_constant<bool,
        __has_second_argument_type_helper<typename std::remove_cv<_Tp>::type>::value> {
};

/**
 *  Derives from unary_function or binary_function when it
 *  can. Specializations handle all of the easy cases. The primary
 *  template determines what to do with a class type, which may
 *  derive from both unary_function and binary_function.
 */
template<typename _Tp>
struct _Reference_wrapper_base: _Reference_wrapper_base_impl<
        __has_argument_type<_Tp>::value,
        __has_first_argument_type<_Tp>::value
                && __has_second_argument_type<_Tp>::value, _Tp> {
};

// - a function type (unary)
template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res(_T1)> : std::unary_function<_T1, _Res> {
};

template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res(_T1) const> : std::unary_function<_T1, _Res> {
};

template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res(_T1) volatile> : std::unary_function<_T1,
        _Res> {
};

template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res(_T1) const volatile> : std::unary_function<
        _T1, _Res> {
};

// - a function type (binary)
template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res(_T1, _T2)> : std::binary_function<_T1, _T2,
        _Res> {
};

template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res(_T1, _T2) const> : std::binary_function<_T1,
        _T2, _Res> {
};

template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res(_T1, _T2) volatile> : std::binary_function<
        _T1, _T2, _Res> {
};

template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res(_T1, _T2) const volatile> : std::binary_function<
        _T1, _T2, _Res> {
};

// - a function pointer type (unary)
template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res (*)(_T1)> : std::unary_function<_T1, _Res> {
};

// - a function pointer type (binary)
template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res (*)(_T1, _T2)> : std::binary_function<_T1,
        _T2, _Res> {
};

// - a pointer to member function type (unary, no qualifiers)
template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res (_T1::*)()> : std::unary_function<_T1*, _Res> {
};

// - a pointer to member function type (binary, no qualifiers)
template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res (_T1::*)(_T2)> : std::binary_function<_T1*,
        _T2, _Res> {
};

// - a pointer to member function type (unary, const)
template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res (_T1::*)() const> : std::unary_function<
        const _T1*, _Res> {
};

// - a pointer to member function type (binary, const)
template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res (_T1::*)(_T2) const> : std::binary_function<
        const _T1*, _T2, _Res> {
};

// - a pointer to member function type (unary, volatile)
template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res (_T1::*)() volatile> : std::unary_function<
        volatile _T1*, _Res> {
};

// - a pointer to member function type (binary, volatile)
template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res (_T1::*)(_T2) volatile> : std::binary_function<
        volatile _T1*, _T2, _Res> {
};

// - a pointer to member function type (unary, const volatile)
template<typename _Res, typename _T1>
struct _Reference_wrapper_base<_Res (_T1::*)() const volatile> : std::unary_function<
        const volatile _T1*, _Res> {
};

// - a pointer to member function type (binary, const volatile)
template<typename _Res, typename _T1, typename _T2>
struct _Reference_wrapper_base<_Res (_T1::*)(_T2) const volatile> : std::binary_function<
        const volatile _T1*, _T2, _Res> {
};


// @} group functors

template<typename ... _Types>
struct _Pack: std::integral_constant<size_t, sizeof...(_Types)>
{};

template<typename _From, typename _To, bool = _From::value == _To::value>
struct _AllConvertible: std::false_type {
};

template<typename ... _From, typename ... _To>
struct _AllConvertible<_Pack<_From...>, _Pack<_To...>, true> : __and_<
        std::is_convertible<_From, _To> ...> {
};

template<typename _Tp1, typename _Tp2>
using _NotSame = __not_<std::is_same<typename std::decay<_Tp1>::type,
typename std::decay<_Tp2>::type>>;

/**
 * Derives from @c unary_function or @c binary_function, or perhaps
 * nothing, depending on the number of arguments provided. The
 * primary template is the basis case, which derives nothing.
 */
template<typename _Res, typename ... _ArgTypes>
struct _Maybe_unary_or_binary_function {
};

/// Derives from @c unary_function, as appropriate.
template<typename _Res, typename _T1>
struct _Maybe_unary_or_binary_function<_Res, _T1> : std::unary_function<_T1,
        _Res> {
};

/// Derives from @c binary_function, as appropriate.
template<typename _Res, typename _T1, typename _T2>
struct _Maybe_unary_or_binary_function<_Res, _T1, _T2> : std::binary_function<
        _T1, _T2, _Res> {
};

/// Implementation of @c mem_fn for member function pointers.
template<typename _Res, typename _Class, typename ... _ArgTypes>
class _Mem_fn<_Res (_Class::*)(_ArgTypes...)> : public _Maybe_unary_or_binary_function<
        _Res, _Class*, _ArgTypes...> {
    typedef _Res (_Class::*_Functor)(_ArgTypes...);

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __object, const volatile _Class *,
            _Args&&... __args) const {
        return (std::forward<_Tp>(__object).*__pmf)(
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __ptr, const volatile void *, _Args&&... __args) const {
        return ((*__ptr).*__pmf)(std::forward<_Args>(__args)...);
    }

    // Require each _Args to be convertible to corresponding _ArgTypes
    template<typename ... _Args>
    using _RequireValidArgs
    = _Require<_AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    // Require each _Args to be convertible to corresponding _ArgTypes
    // and require _Tp is not _Class, _Class& or _Class*
    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs2
    = _Require<_NotSame<_Class, _Tp>, _NotSame<_Class*, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    // Require each _Args to be convertible to corresponding _ArgTypes
    // and require _Tp is _Class or derived from _Class
    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs3
    = _Require<std::is_base_of<_Class, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

public:
    typedef _Res result_type;

    explicit _Mem_fn(_Functor __pmf) :
            __pmf(__pmf) {
    }

    // Handle objects
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(_Class& __object, _Args&&... __args) const {
        return (__object.*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(_Class&& __object, _Args&&... __args) const {
        return (std::move(__object).*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle pointers
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(_Class* __object, _Args&&... __args) const {
        return (__object->*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle smart pointers, references and pointers to derived
    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs2<_Tp, _Args...>>
    _Res operator()(_Tp&& __object, _Args&&... __args) const {
        return _M_call(std::forward<_Tp>(__object), &__object,
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs3<_Tp, _Args...>>
    _Res operator()(std::reference_wrapper<_Tp> __ref, _Args&&... __args) const {
        return operator()(__ref.get(), std::forward<_Args>(__args)...);
    }

private:
    _Functor __pmf;
};

/// Implementation of @c mem_fn for const member function pointers.
template<typename _Res, typename _Class, typename ... _ArgTypes>
class _Mem_fn<_Res (_Class::*)(_ArgTypes...) const> : public _Maybe_unary_or_binary_function<
        _Res, const _Class*, _ArgTypes...> {
    typedef _Res (_Class::*_Functor)(_ArgTypes...) const;

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __object, const volatile _Class *,
            _Args&&... __args) const {
        return (std::forward<_Tp>(__object).*__pmf)(
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __ptr, const volatile void *, _Args&&... __args) const {
        return ((*__ptr).*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args>
    using _RequireValidArgs
    = _Require<_AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs2
    = _Require<_NotSame<_Class, _Tp>, _NotSame<const _Class*, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs3
    = _Require<std::is_base_of<_Class, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

public:
    typedef _Res result_type;

    explicit _Mem_fn(_Functor __pmf) :
            __pmf(__pmf) {
    }

    // Handle objects
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(const _Class& __object, _Args&&... __args) const {
        return (__object.*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(const _Class&& __object, _Args&&... __args) const {
        return (std::move(__object).*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle pointers
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(const _Class* __object, _Args&&... __args) const {
        return (__object->*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle smart pointers, references and pointers to derived
    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs2<_Tp, _Args...>>
    _Res operator()(_Tp&& __object, _Args&&... __args) const {
        return _M_call(std::forward<_Tp>(__object), &__object,
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs3<_Tp, _Args...>>
    _Res operator()(std::reference_wrapper<_Tp> __ref, _Args&&... __args) const {
        return operator()(__ref.get(), std::forward<_Args>(__args)...);
    }

private:
    _Functor __pmf;
};

/// Implementation of @c mem_fn for volatile member function pointers.
template<typename _Res, typename _Class, typename ... _ArgTypes>
class _Mem_fn<_Res (_Class::*)(_ArgTypes...) volatile> : public _Maybe_unary_or_binary_function<
        _Res, volatile _Class*, _ArgTypes...> {
    typedef _Res (_Class::*_Functor)(_ArgTypes...) volatile;

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __object, const volatile _Class *,
            _Args&&... __args) const {
        return (std::forward<_Tp>(__object).*__pmf)(
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __ptr, const volatile void *, _Args&&... __args) const {
        return ((*__ptr).*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args>
    using _RequireValidArgs
    = _Require<_AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs2
    = _Require<_NotSame<_Class, _Tp>, _NotSame<volatile _Class*, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs3
    = _Require<std::is_base_of<_Class, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

public:
    typedef _Res result_type;

    explicit _Mem_fn(_Functor __pmf) :
            __pmf(__pmf) {
    }

    // Handle objects
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(volatile _Class& __object, _Args&&... __args) const {
        return (__object.*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(volatile _Class&& __object, _Args&&... __args) const {
        return (std::move(__object).*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle pointers
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(volatile _Class* __object, _Args&&... __args) const {
        return (__object->*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle smart pointers, references and pointers to derived
    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs2<_Tp, _Args...>>
    _Res operator()(_Tp&& __object, _Args&&... __args) const {
        return _M_call(std::forward<_Tp>(__object), &__object,
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs3<_Tp, _Args...>>
    _Res operator()(std::reference_wrapper<_Tp> __ref, _Args&&... __args) const {
        return operator()(__ref.get(), std::forward<_Args>(__args)...);
    }

private:
    _Functor __pmf;
};

/// Implementation of @c mem_fn for const volatile member function pointers.
template<typename _Res, typename _Class, typename ... _ArgTypes>
class _Mem_fn<_Res (_Class::*)(_ArgTypes...) const volatile> : public _Maybe_unary_or_binary_function<
        _Res, const volatile _Class*, _ArgTypes...> {
    typedef _Res (_Class::*_Functor)(_ArgTypes...) const volatile;

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __object, const volatile _Class *,
            _Args&&... __args) const {
        return (std::forward<_Tp>(__object).*__pmf)(
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args>
    _Res _M_call(_Tp&& __ptr, const volatile void *, _Args&&... __args) const {
        return ((*__ptr).*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args>
    using _RequireValidArgs
    = _Require<_AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs2
    = _Require<_NotSame<_Class, _Tp>,
    _NotSame<const volatile _Class*, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

    template<typename _Tp, typename ... _Args>
    using _RequireValidArgs3
    = _Require<std::is_base_of<_Class, _Tp>,
    _AllConvertible<_Pack<_Args...>, _Pack<_ArgTypes...>>>;

public:
    typedef _Res result_type;

    explicit _Mem_fn(_Functor __pmf) :
            __pmf(__pmf) {
    }

    // Handle objects
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(const volatile _Class& __object, _Args&&... __args) const {
        return (__object.*__pmf)(std::forward<_Args>(__args)...);
    }

    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(const volatile _Class&& __object, _Args&&... __args) const {
        return (std::move(__object).*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle pointers
    template<typename ... _Args, typename _Req = _RequireValidArgs<_Args...>>
    _Res operator()(const volatile _Class* __object, _Args&&... __args) const {
        return (__object->*__pmf)(std::forward<_Args>(__args)...);
    }

    // Handle smart pointers, references and pointers to derived
    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs2<_Tp, _Args...>>
    _Res operator()(_Tp&& __object, _Args&&... __args) const {
        return _M_call(std::forward<_Tp>(__object), &__object,
                std::forward<_Args>(__args)...);
    }

    template<typename _Tp, typename ... _Args,
            typename _Req = _RequireValidArgs3<_Tp, _Args...>>
    _Res operator()(std::reference_wrapper<_Tp> __ref, _Args&&... __args) const {
        return operator()(__ref.get(), std::forward<_Args>(__args)...);
    }

private:
    _Functor __pmf;
};

template<typename _Tp, bool>
struct _Mem_fn_const_or_non {
    typedef const _Tp& type;
};

template<typename _Tp>
struct _Mem_fn_const_or_non<_Tp, false> {
    typedef _Tp& type;
};

template<typename _Res, typename _Class>
class _Mem_fn<_Res _Class::*> {
    using __pm_type = _Res _Class::*;

    // This bit of genius is due to Peter Dimov, improved slightly by
    // Douglas Gregor.
    // Made less elegant to support perfect forwarding and noexcept.
    template<typename _Tp>
    auto _M_call(_Tp&& __object, const _Class *) const NOEXCEPT
    -> decltype(std::forward<_Tp>(__object).*std::declval<__pm_type&>())
    {
        return std::forward<_Tp>(__object).*__pm;
    }

    template<typename _Tp, typename _Up>
    auto _M_call(_Tp&& __object, _Up * const *) const NOEXCEPT
    -> decltype((*std::forward<_Tp>(__object)).*std::declval<__pm_type&>())
    {
        return (*std::forward<_Tp>(__object)).*__pm;
    }

    template<typename _Tp>
    auto _M_call(_Tp&& __ptr, const volatile void*) const
            NOEXCEPT(NOEXCEPT((*__ptr).*std::declval<__pm_type&>()))
            -> decltype((*__ptr).*std::declval<__pm_type&>())
            {
        return (*__ptr).*__pm;
    }

public:
    explicit _Mem_fn(_Res _Class::*__pm) NOEXCEPT : __pm(__pm) {}

    // Handle objects
    _Res&
    operator()(_Class& __object) const NOEXCEPT
    {   return __object.*__pm;}

    const _Res&
    operator()(const _Class& __object) const NOEXCEPT
    {   return __object.*__pm;}

    _Res&&
    operator()(_Class&& __object) const NOEXCEPT
    {   return std::forward<_Class>(__object).*__pm;}

    const _Res&&
    operator()(const _Class&& __object) const NOEXCEPT
    {   return std::forward<const _Class>(__object).*__pm;}

    // Handle pointers
    _Res&
    operator()(_Class* __object) const NOEXCEPT
    {   return __object->*__pm;}

    const _Res&
    operator()(const _Class* __object) const NOEXCEPT
    {   return __object->*__pm;}

    // Handle smart pointers and derived
    template<typename _Tp, typename _Req = _Require<_NotSame<_Class*, _Tp>>>
    auto
    operator()(_Tp&& __unknown) const
    NOEXCEPT(NOEXCEPT(std::declval<_Mem_fn*>()->_M_call
                    (std::forward<_Tp>(__unknown), &__unknown)))
    -> decltype(this->_M_call(std::forward<_Tp>(__unknown), &__unknown))
    {   return _M_call(std::forward<_Tp>(__unknown), &__unknown);}

    template<typename _Tp, typename _Req = _Require<std::is_base_of<_Class, _Tp>>>
    auto
    operator()(std::reference_wrapper<_Tp> __ref) const
    NOEXCEPT(NOEXCEPT(std::declval<_Mem_fn&>()(__ref.get())))
    -> decltype((*this)(__ref.get()))
    {   return (*this)(__ref.get());}

private:
    _Res _Class::*__pm;
};

/**
 *  @brief Returns a function object that forwards to the member
 *  pointer @a pm.
 *  @ingroup functors
 */
template<typename _Tp, typename _Class>
inline _Mem_fn<_Tp _Class::*> mem_fn(_Tp _Class::* __pm) NOEXCEPT
{
    return _Mem_fn<_Tp _Class::*>(__pm);
}

/**
 *  @brief Determines if the given type _Tp is a function object
 *  should be treated as a subexpression when evaluating calls to
 *  function objects returned by bind(). [TR1 3.6.1]
 *  @ingroup binders
 */
template<typename _Tp>
struct is_bind_expression: public std::false_type {
};

template<typename _Signature>
struct _Bind_simple;

template<typename _Callable, typename ... _Args>
struct _Bind_simple<_Callable(_Args...)> {
public:
	typedef typename std::result_of<_Callable(_Args...)>::type result_type;

private:
    std::tuple<_Callable, _Args...> _M_bound;

    template<std::size_t... _Indices>
    result_type _M_invoke(_Index_tuple<_Indices...>)
    {
        // std::bind always forwards bound arguments as lvalues,
        // but this type can call functions which only accept rvalues.
        return std::forward<_Callable>(std::get<0>(_M_bound))(
                std::forward<_Args>(std::get<_Indices+1>(_M_bound))...);
    }

public:

    template<typename ... _Args2, typename = typename std::enable_if<sizeof...(_Args) == sizeof...(_Args2)>::type>
    explicit
    _Bind_simple(const _Callable& __callable, _Args2&&... __args)
    : _M_bound(__callable, std::forward<_Args2>(__args)...)
    {}

    template<typename... _Args2, typename = typename
    std::enable_if< sizeof...(_Args) == sizeof...(_Args2)>::type>
    explicit
    _Bind_simple(_Callable&& __callable, _Args2&&... __args)
    : _M_bound(std::move(__callable), std::forward<_Args2>(__args)...)
    {}

    _Bind_simple(const _Bind_simple&) = default;
    _Bind_simple(_Bind_simple&&) = default;

    result_type
    operator()()
    {
        typedef typename _Build_index_tuple<sizeof...(_Args)>::__type _Indices;
        return _M_invoke(_Indices());
    }

};

/**
 *  Maps member pointers into instances of _Mem_fn but leaves all
 *  other function objects untouched. Used by tr1::bind(). The
 *  primary template handles the non--member-pointer case.
 */
template<typename _Tp>
struct _Maybe_wrap_member_pointer {
    typedef _Tp type;

    static const _Tp&
    __do_wrap(const _Tp& __x) {
        return __x;
    }

    static _Tp&&
    __do_wrap(_Tp&& __x) {
        return static_cast<_Tp&&>(__x);
    }
};

///**
// *  Maps member pointers into instances of _Mem_fn but leaves all
// *  other function objects untouched. Used by tr1::bind(). This
// *  partial specialization handles the member pointer case.
// */
//template<typename _Tp, typename _Class>
//struct _Maybe_wrap_member_pointer<_Tp _Class::*> {
//    typedef _Mem_fn<_Tp _Class::*> type;
//
//    static type __do_wrap(_Tp _Class::* __pm) {
//        return type(__pm);
//    }
//};

// Specialization needed to prevent "forming reference to void" errors when
// bind<void>() is called, because argument deduction instantiates
// _Maybe_wrap_member_pointer<void> outside the immediate context where
// SFINAE applies.
template<>
struct _Maybe_wrap_member_pointer<void> {
    typedef void type;
};

template<typename _Func, typename ... _BoundArgs>
struct _Bind_simple_helper {
    typedef _Maybe_wrap_member_pointer<typename std::decay<_Func>::type> __maybe_type;
    typedef typename __maybe_type::type __func_type;
    typedef _Bind_simple<__func_type(typename std::decay<_BoundArgs>::type...)> __type; };

} // detail

// Simplified version of std::bind for internal use, without support for
// unbound arguments, placeholders or nested bind expressions.
template<typename _Callable, typename ... _Args>
typename detail::_Bind_simple_helper<_Callable, _Args...>::__type bind_simple(
        _Callable&& __callable, _Args&&... __args) {
    typedef detail::_Bind_simple_helper<_Callable, _Args...> __helper_type;
    typedef typename __helper_type::__maybe_type __maybe_type;
    typedef typename __helper_type::__type __result_type;
    return __result_type(
            __maybe_type::__do_wrap(std::forward<_Callable>(__callable)),
            std::forward<_Args>(__args)...);
}

} // concurrency
} // mas
