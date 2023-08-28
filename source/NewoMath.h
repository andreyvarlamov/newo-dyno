#ifndef NEWO_MATH_H
#define NEWO_MATH_H

#include "NewoCommon.h"

#include <cmath>
#include <cfloat>

#define PI32 3.141592653f

internal inline f32
AbsF32(f32 Value)
{
    return ((Value >= 0.0f) ? Value : -Value);
}

internal inline f32
SqrtF32(f32 Value)
{
    return sqrtf(Value);
}

internal inline f32
SinF32(f32 Value)
{
    return sinf(Value);
}

internal inline f32
CosF32(f32 Value)
{
    return cosf(Value);
}

internal inline f32
TanF32(f32 Value)
{
    return tanf(Value);
}

internal inline f32
DegreesToRadians(f32 Degrees)
{
    return (Degrees * PI32 / 180.0f);
}

internal inline f32
ArcSinF32(f32 Value)
{
    return asinf(Value);
}

internal inline f32
ClampF32(f32 Value, f32 Min, f32 Max)
{
    if (Value < Min) return Min;
    if (Value > Max) return Max;
    return Value;
}

#endif