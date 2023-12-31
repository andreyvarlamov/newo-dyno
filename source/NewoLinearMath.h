#ifndef NEWO_LINEAR_MATH_H
#define NEWO_LINEAR_MATH_H

#include "NewoCommon.h"
#include "NewoMath.h"

// NOTE: All mat functions assume Column-Major representation [Column][Row]

// -------------------------------------------------------------------------------
// VECTOR 2 ----------------------------------------------------------------------
// -------------------------------------------------------------------------------

union vec2
{
    struct
    {
        f32 X, Y;
    };

    f32 D[2];
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
operator-(vec2 V0)
{
    return vec2 { -V0.X, -V0.Y };
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

internal inline f32
VecDot(vec2 V0, vec2 V1)
{
    return (V0.X * V1.X + V0.Y * V1.Y);
}

internal inline f32
VecLengthSq(vec2 V)
{
    return VecDot(V, V);
}

internal inline f32
VecLength(vec2 V)
{
    return SqrtF32(VecLengthSq(V));
}

internal inline vec2
VecNormalize(vec2 V)
{
    f32 Length = VecLength(V);
    return ((Length != 0.0f) ? (V / Length) : V);
}

// -------------------------------------------------------------------------------
// VECTOR 3 ----------------------------------------------------------------------
// -------------------------------------------------------------------------------

union vec3
{
    struct
    {
        f32 X, Y, Z;
    };

    f32 D[3];
};


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
operator-(vec3 V0)
{
    return vec3 { -V0.X, -V0.Y, -V0.Z };
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
    return vec3 { V.X / S, V.Y / S, V.Z / S };
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

internal inline f32
VecDot(vec3 V0, vec3 V1)
{
    return (V0.X * V1.X + V0.Y * V1.Y + V0.Z * V1.Z);
}

internal inline f32
VecLengthSq(vec3 V)
{
    return VecDot(V, V);
}

internal inline f32
VecLength(vec3 V)
{
    return SqrtF32(VecLengthSq(V));
}

internal inline vec3
VecNormalize(vec3 V)
{
    f32 Length = VecLength(V);
    return ((Length != 0.0f) ? (V / Length) : V);
}

internal inline vec3
VecCross(vec3 V0, vec3 V1)
{
    return vec3 { V0.Y * V1.Z - V0.Z * V1.Y,
        V0.Z * V1.X - V0.X * V1.Z,
        V0.X * V1.Y - V0.Y * V1.X };
}

internal inline f32
VecScalarTriple(vec3 A, vec3 B, vec3 C)
{
    return VecDot(A, VecCross(B, C));
}

internal inline bool
IsZeroVector(vec3 Vector)
{
    return ((Vector.X <= FLT_EPSILON) &&
            (Vector.Y <= FLT_EPSILON) &&
            (Vector.Z <= FLT_EPSILON));
}

// -------------------------------------------------------------------------------
// VECTOR 4 ----------------------------------------------------------------------
// -------------------------------------------------------------------------------

union vec4
{
    struct
    {
        f32 X, Y, Z, W;
    };

    f32 D[4];
};

internal inline vec4
operator+(vec4 V0, vec4 V1);

internal inline vec4
operator-(vec4 V0, vec4 V1);

internal inline vec4
operator-(vec4 V0);

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

// -------------------------------------------------------------------------------
// MATRIX 3 ----------------------------------------------------------------------
// -------------------------------------------------------------------------------

struct mat3
{
    f32 D[3][3];
};

internal inline mat3
Mat3Identity()
{
    mat3 Result = {};

    Result.D[0][0] = 1.0f;
    Result.D[1][1] = 1.0f;
    Result.D[2][2] = 1.0f;

    return Result;
}

internal inline mat3
Mat3Transpose(mat3 M)
{
    mat3 Result = {};

    for (u32 Column = 0; Column < 3; ++Column)
    {
        for (u32 Row = 0; Row < 3; ++Row)
        {
            Result.D[Column][Row] = M.D[Row][Column];
        }
    }

    return Result;
}

internal inline mat3
operator*(mat3 M0, mat3 M1)
{
    mat3 Result = {};

    for (i32 I = 0; I < 3; ++I)
    {
        for (i32 J = 0; J < 3; ++J)
        {
            Result.D[J][I] = (M0.D[0][I]*M1.D[J][0] +
                              M0.D[1][I]*M1.D[J][1] +
                              M0.D[2][I]*M1.D[J][2]);
        }
    }

    return Result;
}

internal inline vec3
operator*(mat3 M, vec3 V)
{
    vec3 Result = {};

    Result.X = M.D[0][0] * V.X + M.D[1][0] * V.Y + M.D[2][0] * V.Z;
    Result.Y = M.D[0][1] * V.X + M.D[1][1] * V.Y + M.D[2][1] * V.Z;
    Result.Z = M.D[0][2] * V.X + M.D[1][2] * V.Y + M.D[2][2] * V.Z;

    return Result;
}

internal inline mat3
Mat3FromCols(vec3 A, vec3 B, vec3 C)
{
    mat3 Result = {};

    Result.D[0][0] = A.X;
    Result.D[0][1] = A.Y;
    Result.D[0][2] = A.Z;

    Result.D[1][0] = B.X;
    Result.D[1][1] = B.Y;
    Result.D[1][2] = B.Z;

    Result.D[2][0] = C.X;
    Result.D[2][1] = C.Y;
    Result.D[2][2] = C.Z;

    return Result;
}

internal inline mat3
Mat3FromCols(vec3 *Cols)
{
    Assert(Cols);

    mat3 Result = {};

    Result.D[0][0] = Cols[0].X;
    Result.D[0][1] = Cols[0].Y;
    Result.D[0][2] = Cols[0].Z;

    Result.D[1][0] = Cols[1].X;
    Result.D[1][1] = Cols[1].Y;
    Result.D[1][2] = Cols[1].Z;

    Result.D[2][0] = Cols[2].X;
    Result.D[2][1] = Cols[2].Y;
    Result.D[2][2] = Cols[2].Z;

    return Result;
}

internal inline mat3
Mat3GetRotationAroundAxis(vec3 Axis, f32 Angle)
{
    vec3 WorldNegZRotated = { -SinF32(Angle), 0.0f, -CosF32(Angle) };
    vec3 Up = VecNormalize(Axis);
    vec3 Right = VecNormalize(VecCross(WorldNegZRotated, Up));
    vec3 Back = VecNormalize(VecCross(Right, Up));

    mat3 Result = Mat3FromCols(Right, Up, Back);
    return Result;
}

internal inline vec3
Mat3GetCol(mat3 M, u32 ColumnIndex)
{
    Assert(ColumnIndex < 3);

    vec3 Result;

    for (u32 RowIndex = 0; RowIndex < 3; ++RowIndex)
    {
        Result.D[RowIndex] = M.D[ColumnIndex][RowIndex];
    }

    return Result;
}

internal inline void
Mat3GetCols(mat3 M, vec3 *Out_Cols)
{
    Assert(Out_Cols);

    for (u32 ColIndex = 0; ColIndex < 3; ++ColIndex)
    {
        for (u32 RowIndex = 0; RowIndex < 3; ++RowIndex)
        {
            Out_Cols[ColIndex].D[RowIndex] = M.D[ColIndex][RowIndex];
        }
    }
}

// -------------------------------------------------------------------------------
// MATRIX 4 ----------------------------------------------------------------------
// -------------------------------------------------------------------------------

struct mat4
{
    f32 D[4][4];
};

internal inline mat4
Mat4Identity()
{
    mat4 Result = {};

    Result.D[0][0] = 1.0f;
    Result.D[1][1] = 1.0f;
    Result.D[2][2] = 1.0f;
    Result.D[3][3] = 1.0f;

    return Result;
}

internal inline mat4
operator*(mat4 M0, mat4 M1)
{
    mat4 Result = {};

    for (i32 I = 0; I < 4; ++I)
    {
        for (i32 J = 0; J < 4; ++J)
        {
            Result.D[J][I] = (M0.D[0][I]*M1.D[J][0] +
                              M0.D[1][I]*M1.D[J][1] +
                              M0.D[2][I]*M1.D[J][2] +
                              M0.D[3][I]*M1.D[J][3]);
        }
    }

    return Result;
}

internal inline vec4
operator*(mat4 M, vec4 V)
{
    vec4 Result = {};

    Result.X = M.D[0][0] * V.X + M.D[1][0] * V.Y + M.D[2][0] * V.Z + M.D[3][0] * V.W;
    Result.Y = M.D[0][1] * V.X + M.D[1][1] * V.Y + M.D[2][1] * V.Z + M.D[3][1] * V.W;
    Result.Z = M.D[0][2] * V.X + M.D[1][2] * V.Y + M.D[2][2] * V.Z + M.D[3][2] * V.W;
    Result.W = M.D[0][2] * V.X + M.D[1][2] * V.Y + M.D[2][2] * V.Z + M.D[3][3] * V.W;

    return Result;
}

#endif
