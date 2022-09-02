#pragma once
#include <cstdint>
#include <cstddef>
#include <sys/types.h>

using i8  = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

#if _WIN32
using ssize_t = SSIZE_T;
#endif

using usize = size_t;
using isize = ssize_t;

using f32 = float;
using f64 = double;

using c_string = char const*;
