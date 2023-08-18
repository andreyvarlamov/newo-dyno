#ifndef NEWO_LINEAR_MATH_H
#define NEWO_LINEAR_MATH_H

#include "NewoCommon.h"
#include "NewoMath.h"

// NOTE: All mat functions assume Column-Major representation [Column][Row]

union vec2
{
    struct
    {
        f32 X, Y;
    };

    f32 D[2];
};

union vec3
{
    struct
    {
        f32 X, Y, Z;
    };

    f32 D[3];
};

union vec4
{
    struct
    {
        f32 X, Y, Z, W;
    };

    f32 D[4];
};

struct mat3
{
    f32 D[3][3];
};

struct mat4
{
    f32 D[4][4];
};

internal inline vec2
operator+(vec2 V0, vec2 V1)
{
    return vec2 { V0.X + V1.X, V0.Y + V1.Y };
}

internal inline vec2
operator-(vec2 V0, vec2 V1)
{
    return vec2 { V0.X - V1.X, V0.Y - V1.Y };
}

internal inline vec2
operator*(vec2 V, f32 S)
{
    return vec2 { V.X * S, V.Y * S };
}

internal inline  vec2
operator*(f32 S, vec2 V)
{
    return vec2 { V.X * S, V.Y * S };
}

internal inline vec2
operator/(vec2 V, f32 S)
{
    return vec2 { V.X / S, V.Y / S };
}

internal inline vec2 &
operator+=(vec2 &V0, vec2 V1)
{
    V0 = V0 + V1;
    return V0;
}

internal inline vec2 &
operator-=(vec2 &V0, vec2 V1)
{
    V0 = V0 - V1;
    return V0;
}

internal inline vec2 &
operator*=(vec2 &V, f32 S)
{
    V = V * S;
    return V;
}

internal inline vec2 &
operator/=(vec2 &V, f32 S)
{
    V = V / S;
    return V;
}

internal inline vec3
operator+(vec3 V0, vec3 V1)
{
    return vec3 { V0.X + V1.X, V0.Y + V1.Y, V0.Z + V1.Z };
}

internal inline vec3
operator-(vec3 V0, vec3 V1)
{
    return vec3 { V0.X - V1.X, V0.Y - V1.Y, V0.Z - V1.Z };
}

internal inline vec3
operator*(vec3 V, f32 S)
{
    return vec3 { V.X * S, V.Y * S, V.Z * S };
}

internal inline vec3
operator*(f32 S, vec3 V)
{
    return vec3 { V.X * S, V.Y * S, V.Z * S };
}

internal inline vec3
operator/(vec3 V, f32 S)
{
    return vec3 { V.X * S, V.Y * S, V.Z * S };
}

internal inline vec3 &
operator+=(vec3 &V0, vec3 V1)
{
    V0 = V0 + V1;
    return V0;
}

internal inline vec3 &
operator-=(vec3 &V0, vec3 V1)
{
    V0 = V0 - V1;
    return V0;
}

internal inline vec3 &
operator*=(vec3 &V, f32 S)
{
    V = V * S;
    return V;
}

internal inline vec3 &
operator/=(vec3 &V, f32 S)
{
    V = V / S;
    return V;
}
#if 0
internal inline vec4
operator+(vec4 V0, vec4 V1);

internal inline vec4
operator-(vec4 V0, vec4 V1);

internal inline vec4
operator*(vec4 V, f32 S);

internal inline vec4
operator*(f32 S, vec4 V);

internal inline vec4
operator/(vec4 V, f32 S);

internal inline vec4 &
operator+=(vec4 &V0, vec4 V1);

internal inline vec4 &
operator-=(vec4 &V0, vec4 V1);

internal inline vec4 &
operator*=(vec4 &V, f32 S);

internal inline vec4 &
operator/=(vec4 &V, f32 S);

internal inline mat3
operator*(mat3 M0, mat3 M1);

internal inline mat3
operator*(mat3 M, vec3 V);

internal inline mat4
operator*(mat4 M0, mat4 M1);

internal inline mat4
operator*(mat4 M, vec3 V);
#endif

internal inline f32
VecDotProduct(vec2 V0, vec2 V1)
{
    return (V0.X * V1.X + V0.Y * V1.Y);
}

internal inline f32
VecLengthSqr(vec2 V)
{
    return VecDotProduct(V, V);
}

internal inline f32
VecLength(vec2 V)
{
    return SqrtF32(VecLengthSqr(V));
}

internal inline vec2
VecNormalize(vec2 V)
{
    return V / VecLength(V);
}

internal inline f32
VecDotProduct(vec3 V0, vec3 V1)
{
    return (V0.X * V1.X + V0.Y * V1.Y + V0.Z * V1.Z);
}

internal inline f32
VecLengthSqr(vec3 V)
{
    return VecDotProduct(V, V);
}

internal inline f32
VecLength(vec3 V)
{
    return SqrtF32(VecLengthSqr(V));
}

internal inline vec3
VecNormalize(vec3 V)
{
    return V / VecLength(V);
}

internal inline vec3
CrossProduct(vec3 V0, vec3 V1)
{
    return vec3 { V0.Y * V1.Z - V0.Z * V1.Y,
        V0.X * V1.Z - V0.Z * V1.X,
        V0.X * V1.Y - V0.Y * V1.X };
}

#endif
