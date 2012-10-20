/*
 *  xstdint.h
 *  (for Visual C++)
 *
 *  Copyright (C) 2007 by Kijineko Inc.
 *  Copyright (C) 2007 by TAKAGI Nobuhisa
 */
#pragma once

#ifndef _WIN32
#error  only win32 target supported!
#endif

#ifdef  __cplusplus
namespace std { namespace tr1 {
#endif

typedef signed char       int8_t;
typedef unsigned char     uint8_t;
typedef signed char       int_least8_t;
typedef unsigned char     uint_least8_t;
typedef signed char       int_fast8_t;
typedef unsigned char     uint_fast8_t;

typedef signed short      int16_t;
typedef unsigned short    uint16_t;
typedef signed short      int_least16_t;
typedef unsigned short    uint_least16_t;
typedef signed short      int_fast16_t;
typedef unsigned short    uint_fast16_t;

typedef signed int        int32_t;
typedef unsigned int      uint32_t;
typedef signed int        int_least32_t;
typedef unsigned int      uint_least32_t;
typedef signed int        int_fast32_t;
typedef unsigned int      uint_fast32_t;

typedef signed __int64    int64_t;
typedef unsigned __int64  uint64_t;
typedef signed __int64    int_least64_t;
typedef unsigned __int64  uint_least64_t;
typedef signed __int64    int_fast64_t;
typedef unsigned __int64  uint_fast64_t;

#if _INTEGRAL_MAX_BITS >= 128
typedef signed __int128   int128_t;
typedef unsigned __int128 uint128_t;
typedef signed __int128   intmax_t;
typedef unsigned __int128 uintmax_t;
#else
typedef signed __int64    intmax_t;
typedef unsigned __int64  uintmax_t;
#endif

typedef signed int        intptr_t;
typedef unsigned int      uintptr_t;

#ifdef  __cplusplus
} }
#endif

#if !defined(__cplusplus) || defined(__STDC_CONSTANT_MACROS)

#define INT8_C(x)         x
#define UINT8_C(x)        x
#define INT16_C(x)        x
#define UINT16_C(x)       x
#define INT32_C(x)        x
#define UINT32_C(x)       x##u
#define INT64_C(x)        x##i64
#define UINT64_C(x)       x##ui64

#if _INTEGRAL_MAX_BITS >= 128
#define INTMAX_C(x)       x##i128
#define UINTMAX_C(x)      x##ui128
#else
#define INTMAX_C(x)       x##i64
#define UINTMAX_C(x)      x##ui64
#endif

#endif  /* __STDC_CONSTANT_MACROS */

#if !defined(__cplusplus) || defined(__STDC_LIMIT_MACROS)

#define INT8_MIN          -128
#define INT8_MAX          127
#define UINT8_MAX         255

#define INT_LEAST8_MIN    -128
#define INT_LEAST8_MAX    127
#define UINT_LEAST8_MAX   255

#define INT_FAST8_MIN     -128
#define INT_FAST8_MAX     127
#define UINT_FAST8_MAX    255

#define INT16_MIN         -32768
#define INT16_MAX         32767
#define UINT16_MAX        65535

#define INT_LEAST16_MIN   -32768
#define INT_LEAST16_MAX   32767
#define UINT_LEAST16_MAX  65535

#define INT_FAST16_MIN    -32768
#define INT_FAST16_MAX    32767
#define UINT_FAST16_MAX   65535

#define INT32_MIN         (-2147483647-1)
#define INT32_MAX         2147483647
#define UINT32_MAX        0xffffffff

#define INT_LEAST32_MIN   (-2147483647-1)
#define INT_LEAST32_MAX   2147483647
#define UINT_LEAST32_MAX  0xffffffff

#define INT_FAST32_MIN    (-2147483647-1)
#define INT_FAST32_MAX    2147483647
#define UINT_FAST32_MAX   0xffffffff

#define INT64_MIN         (-9223372036854775807-1)
#define INT64_MAX         9223372036854775807
#define UINT64_MAX        0xffffffffffffffff

#define INT_LEAST64_MIN   (-9223372036854775807-1)
#define INT_LEAST64_MAX   9223372036854775807
#define UINT_LEAST64_MAX  0xffffffffffffffff

#define INT_FAST64_MIN    (-9223372036854775807i64-1)
#define INT_FAST64_MAX    9223372036854775807i64
#define UINT_FAST64_MAX   0xffffffffffffffffui64

#if _INTEGRAL_MAX_BITS >= 128
#define INT128_MIN        (-170141183460469231731687303715884105727i128-1)
#define INT128_MAX        170141183460469231731687303715884105727i128
#define UINT128_MAX       0xffffffffffffffffffffffffffffffffui128

#define INTMAX_MIN        (-170141183460469231731687303715884105727i128-1)
#define INTMAX_MIN        170141183460469231731687303715884105727i128
#define UINTMAX_MAX       0xffffffffffffffffffffffffffffffffui128
#else
#define INTMAX_MIN        (-9223372036854775807-1)
#define INTMAX_MAX        9223372036854775807
#define UINTMAX_MAX       0xffffffffffffffff
#endif

#define INTPTR_MIN        (-2147483647-1)
#define INTPTR_MAX        2147483647
#define UINTPTR_MAX       0xffffffff

#define PTRDIFF_MIN       (-2147483647-1)
#define PTRDIFF_MAX       2147483647

#define SIG_ATOMIC_MIN    (-2147483647-1)
#define SIG_ATOMIC_MAX    2147483647

#undef  SIZE_MAX
#define SIZE_MAX          0xffffffff

#undef  WCHAR_MIN
#define WCHAR_MIN         0
#undef  WCHAR_MAX
#define WCHAR_MAX         0xffff

#undef  WINT_MIN
#define WINT_MIN          0
#undef  WINT_MAX
#define WINT_MAX          0xffff

#endif  /* __STDC_LIMIT_MACROS */
