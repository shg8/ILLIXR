#pragma once

/**
 * Copyright (c) 2017-present, Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "cuda_decl.h"

#include <stdint.h>
#include <string.h>


#ifdef _MSC_VER
# include <intrin.h>
#else
#include <bitset>
#endif

namespace hvvr {

#ifdef __GNUG__ // gcc or clang
# define ALIGN(alignment) __attribute__((aligned(alignment)))
# define ALIGNOF(type) __alignof__(type)
#elif defined(_MSC_VER)
# define ALIGN(alignment) __declspec(align(alignment))
# define ALIGNOF(type) __alignof(type)
#else
# error "unknown compiler"
#endif

inline uint8_t _bittest(const uint32_t* Base, uint32_t Offset) {
    std::bitset<sizeof(uint32_t)> bitset(*Base); // initializer list doesn't allow narrowing conversion
    return bitset.test(Offset);
}

inline uint8_t _BitScanForward(uint32_t* Index, uint32_t Mask) {
#if _WIN32
    return ::_BitScanForward((unsigned long*)Index, (unsigned long)Mask);
#else
    auto index =  __builtin_ffsl(Mask); // gcc ffs is different from msvc _BitScanForward; it returns index + 1 for non-zero mask and 0 for zero mask
    if (index == 0) {
        return 0;
    } else {
        *Index = index - 1;
        return 1;
    }
#endif
}

inline uint32_t tzcnt(uint32_t mask) {
    uint32_t index;
    _BitScanForward(&index, mask);
    return index;
}

template <typename T> CHDI T min(T a, T b) { return a < b ? a : b; }
template <typename T> CHDI T max(T a, T b) { return a > b ? a : b; }
template <typename T> CHDI T min(T a, T b, T c) { return min(min(a, b), c); }
template <typename T> CHDI T max(T a, T b, T c) { return max(max(a, b), c); }
template <typename T> CHDI T min(T a, T b, T c, T d) { return min(min(min(a, b), c), d); }
template <typename T> CHDI T max(T a, T b, T c, T d) { return max(max(max(a, b), c), d); }

template <typename T> CHDI T clamp(T x, T mn, T mx) { return min(mx, max(mn, x));  }

template <typename T> CHDI T lerp(T a, T b, T t) { return a + (b - a) * t; }

// Useful to correctly convert between float/int interpretations of
// bits without running afoul of strict aliasing rules.
//
// Reading memory through a pointer of one type after writing that
// memory through a pointer to a different type is undefined behavior.
// The one allowed exception is that pointers to type "char" (note:
// not even "unsigned char") are allowed to alias anything else.
// Memcpy uses a pointer to char to do the copy and all modern
// compilers optimize away obvious calls to memcpy, so memcpy is the
// preferred way to "convert" between interpretations.
//
template<typename T, typename F>
inline T alias_cast(const F& raw_data)
{
    static_assert(sizeof(T) == sizeof(F), "argument and result must be the same size");
    T result;
    memcpy(&result, &raw_data, sizeof(T));
    return result;
}

} // namespace hvvr
