/*
 *  stdint.h
 *  (for Visual C++)
 *
 *  Copyright (C) 2007 by Kijineko Inc.
 *  Copyright (C) 2007 by TAKAGI Nobuhisa
 */
#pragma once
#include <xstdint.h>

#ifdef  __cplusplus

using ::std::tr1::int8_t;
using ::std::tr1::uint8_t;
using ::std::tr1::int_least8_t;
using ::std::tr1::uint_least8_t;
using ::std::tr1::int_fast8_t;
using ::std::tr1::uint_fast8_t;

using ::std::tr1::int16_t;
using ::std::tr1::uint16_t;
using ::std::tr1::int_least16_t;
using ::std::tr1::uint_least16_t;
using ::std::tr1::int_fast16_t;
using ::std::tr1::uint_fast16_t;

using ::std::tr1::int32_t;
using ::std::tr1::uint32_t;
using ::std::tr1::int_least32_t;
using ::std::tr1::uint_least32_t;
using ::std::tr1::int_fast32_t;
using ::std::tr1::uint_fast32_t;

using ::std::tr1::int64_t;
using ::std::tr1::uint64_t;
using ::std::tr1::int_least64_t;
using ::std::tr1::uint_least64_t;
using ::std::tr1::int_fast64_t;
using ::std::tr1::uint_fast64_t;

#if _INTEGRAL_MAX_BITS >= 128
using ::std::tr1::int128_t;
using ::std::tr1::uint128_t;
#endif

using ::std::tr1::intmax_t;
using ::std::tr1::uintmax_t;

using ::std::tr1::intptr_t;
using ::std::tr1::uintptr_t;

#endif
