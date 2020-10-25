//
// Created by Harold on 2020/10/25.
//

#ifndef M_MATH_TRAITS_H
#define M_MATH_TRAITS_H

#include <type_traits>
#include <iterator>

template<typename T>
using EnableIfFloatingPoint = typename std::enable_if<std::is_floating_point<T>::value>::type;

template<typename ...>
struct make_void { using type = void; };
template<typename ...Ts>
using void_t = typename make_void<Ts ...>::type;

template<typename T, typename = void>
struct is_iterable : std::false_type {};
template<typename T>
struct is_iterable<T,
        void_t<decltype(std::begin(std::declval<T>()) != std::end(std::declval<T>())),
                decltype(++std::declval<decltype(std::begin(std::declval<T>()))>()),
                decltype(*std::begin(std::declval<T>()))
        >>
        : std::true_type {};

template<typename T, typename = void>
struct is_iterator : std::false_type {};
template<typename T>
struct is_iterator<T,
        typename std::enable_if<!std::is_same<typename std::iterator_traits<T>::value_type, void>::value>::type>
        : std::true_type {};

#endif //M_MATH_TRAITS_H
