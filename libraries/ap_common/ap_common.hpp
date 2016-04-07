#ifndef SUBHUB_AP_COMMON_HPP_INCLUDED
#define SUBHUB_AP_COMMON_HPP_INCLUDED

#define PACKED __attribute__((__packed__))

#define FORMAT(a,b) __attribute__((format(printf, a, b)))

template <typename T, size_t N>
char (&_ARRAY_SIZE_HELPER(T (&_arr)[N]))[N];

template <typename T>
char (&_ARRAY_SIZE_HELPER(T (&_arr)[0]))[0];

#define ARRAY_SIZE(_arr) sizeof(_ARRAY_SIZE_HELPER(_arr))

#endif // SUBHUB_AP_COMMON_HPP_INCLUDED
