#ifndef NEWO_COMMON_H
#define NEWO_COMMON_H

#include <stdint.h>

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef float f32;
typedef double f64;

#define global_variable static
#define local_persist static
#define internal static

#define Assert(Expression) if (!(Expression)) { *(int *) 0 = 0; }

#define ArrayCount(Array) (sizeof((Array)) / (sizeof((Array)[0])))

#define Kilobytes(Value) (         (Value) * 1024)
#define Megabytes(Value) (Kilobytes(Value) * 1024)
#define Gigabytes(Value) (Megabytes(Value) * 1024)
#define Terabytes(Value) (Gigabytes(Value) * 1024)

#define Max(X, Y) ((X) > (Y)) ? (X) : (Y)
#define Min(X, Y) ((X) < (Y)) ? (X) : (Y)

#endif
