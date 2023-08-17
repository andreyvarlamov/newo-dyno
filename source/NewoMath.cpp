#include "NewoMath.h"

#include <cmath>
#include <cfloat>

// TODO: These functions are untested

f32
AbsF32(f32 Value)
{
    return ((Value >= 0.0f) ? Value : -Value);
}

f32
SqrtF32(f32 Value)
{
    return sqrtf(Value);
}

f32
SinF32(f32 Value)
{
    return sinf(Value);
}

f32
CosF32(f32 Value)
{
    return cosf(Value);
}

vec3
operator+(vec3 V0, vec3 V1)
{
    return vec3 { V0.X + V1.X, V0.Y + V1.Y, V0.Z + V1.Z };
}

vec3
operator-(vec3 V0, vec3 V1)
{
    return vec3 { V0.X - V1.X, V0.Y - V1.Y, V0.Z - V1.Z };
}

vec2
operator+(vec2 V0, vec2 V1)
{
    return vec2 { V0.X + V1.X, V0.Y + V1.Y };
}

vec2
operator-(vec2 V0, vec2 V1)
{
    return vec2 { V0.X - V1.X, V0.Y - V1.Y };
}

f32
LengthVec3(vec3 V)
{
    return SqrtF32(DotProduct(V, V));
}

vec3
NormalizeVec3(vec3 V)
{
    f32 Length = LengthVec3(V);
    return vec3 { V.X / Length, V.Y / Length, V.Z / Length };
}

f32
DotProduct2D(vec2 V0, vec2 V1)
{
    return (V0.X * V1.X + V0.Y * V1.Y);
}

f32
DotProduct(vec3 V0, vec3 V1)
{
    return (V0.X * V1.X + V0.Y * V1.Y + V0.Z * V1.Z);
}

vec3
CrossProduct(vec3 V0, vec3 V1)
{
    return vec3 { V0.Y * V1.Z - V0.Z * V1.Y,
                  V0.X * V1.Z - V0.Z * V1.X,
                  V0.X * V1.Y - V0.Y * V1.X };
}

mat4
GetProjectionMat4(f32 AspectRatio, f32 HalfFOV, f32 Near, f32 Far)
{
    mat4 ProjectionMat { };

    //ProjectionMat

    return ProjectionMat;
}

mat4
GetLookAtMat4(vec3 Position, vec3 Front, vec3 Up)
{
    Front = NormalizeVec3(Front - Position);
    Up = NormalizeVec3(Up);
    vec3 Right = NormalizeVec3(CrossProduct(Front, Up));

    mat4 LookAtMat { };

    LookAtMat.D[0][0] = Right.X;
    LookAtMat.D[0][1] = Right.Y;
    LookAtMat.D[0][2] = Right.Z;
    LookAtMat.D[1][0] = Up.X;
    LookAtMat.D[1][1] = Up.Y;
    LookAtMat.D[1][2] = Up.Z;
    LookAtMat.D[2][0] = Front.X;
    LookAtMat.D[2][1] = Front.Y;
    LookAtMat.D[2][2] = Front.Z;

    LookAtMat.D[3][0] = Position.X;
    LookAtMat.D[3][1] = Position.Y;
    LookAtMat.D[3][2] = Position.Z;

    return LookAtMat;
}

f32
TriDoubleSignedArea(vec3 A, vec3 B, vec3 C)
{
    vec3 Cross = CrossProduct(B - A, C - A);
    return LengthVec3(Cross);
}

f32
TriDoubleArea2D(f32 X1, f32 Y1, f32 X2, f32 Y2, f32 X3, f32 Y3)
{
    return (X1 - X2) * (Y2 - Y3) - (X2 - X3) * (Y1 - Y2);
}

void
BarycentricCoordsCramer(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W)
{
    vec3 V0 = B - A;
    vec3 V1 = C - A;
    vec3 V2 = P - A;

    // NOTE: This could be cached per triangle ABC
    f32 Dot00 = DotProduct(V0, V0);
    f32 Dot01 = DotProduct(V0, V1);
    f32 Dot11 = DotProduct(V1, V1);
    f32 Denominator = Dot00 * Dot11 - Dot01 * Dot01;
    //

    f32 Dot20 = DotProduct(V2, V0);
    f32 Dot21 = DotProduct(V2, V1);

    f32 VCoord = (Dot20 * Dot11 - Dot01 * Dot21) / Denominator;
    f32 WCoord = (Dot00 * Dot21 - Dot20 * Dot01) / Denominator;

    if (V) *V = VCoord;
    if (W) *W = WCoord;
    if (U) *U = 1.0f - VCoord - WCoord;
}

void
BarycentricCoordsAreas(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W)
{
    f32 DoubleSignedAreaABC = TriDoubleSignedArea(A, B, C);
    f32 UCoord = TriDoubleSignedArea(P, B, C) / DoubleSignedAreaABC;
    f32 VCoord = TriDoubleSignedArea(P, C, A) / DoubleSignedAreaABC;
    f32 WCoord = TriDoubleSignedArea(P, A, B) / DoubleSignedAreaABC;

    if (U) *U = UCoord;
    if (V) *V = VCoord;
    if (W) *W = WCoord;
}

void
BarycentricCoordsProjectedAreas(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W)
{
    vec3 M = CrossProduct(B - A, C - A); // Unnormalzied triangle normal
    
    f32 X = AbsF32(M.X);
    f32 Y = AbsF32(M.Y);
    f32 Z = AbsF32(M.Z);

    f32 NU; // Nominator for U
    f32 NV; // Nominator for V
    f32 OOD; // One over denominator for U/V ratio
    if (X >= Y && X >= Z)
    {
        // X is largest, project to the YZ plane
        NU = TriDoubleArea2D(P.Y, P.Z, B.Y, B.Z, C.Y, C.Z); // Area of PBC in YZ plane
        NV = TriDoubleArea2D(P.Y, P.Z, C.Y, C.Z, A.Y, A.Z); // Area of PCA in YZ plane
        OOD = 1.0f / M.X; // 1 / (2 * Area of ABC in YZ plane)
    }
    else if (Y >= X && Y >= Z)
    {
        // Y is largest, project to the XZ plane
        NU = TriDoubleArea2D(P.X, P.Z, B.X, B.Z, C.X, C.Z); // Area of PBC in XZ plane
        NV = TriDoubleArea2D(P.X, P.Z, C.X, C.Z, A.X, A.Z); // Area of PCA in XZ plane
        OOD = 1.0f / -M.Y; // 1 / (2 * Area of ABC in XZ plane)
    }
    else
    {
        // Z is largest, project to the XY plane
        NU = TriDoubleArea2D(P.X, P.Y, B.X, B.Y, C.X, C.Y); // Area of PBC in XY plane
        NV = TriDoubleArea2D(P.X, P.Y, C.X, C.Y, A.X, A.Y); // Area of PCA in XY plane
        OOD = 1.0f / M.Z; // 1 / (2 * Area of ABC in XY plane)
    }

    f32 UCoord = NU * OOD;
    f32 VCoord = NV * OOD;
    f32 WCoord = 1.0f - UCoord - VCoord;

    if (U) *U = UCoord;
    if (V) *V = VCoord;
    if (W) *W = WCoord;
}

bool
TestPointTriangle(vec3 P, vec3 A, vec3 B, vec3 C)
{
    f32 V;
    f32 W;
    BarycentricCoordsProjectedAreas(P, A, B, C, NULL, &V, &W);
    return ((V >= 0.0f) && (W >= 0.0f) && ((V + W) <= 1.0f));
}

plane
ComputePlane(vec3 A, vec3 B, vec3 C)
{
    plane Result;

    Result.Normal = CrossProduct(B - A, C - A); // CCW
    Result.Distance = DotProduct(Result.Normal, A);

    return Result;
}

bool
IsQuadConvex(vec3 A, vec3 B, vec3 C, vec3 D)
{
    vec3 NormalBDA = CrossProduct(D - B, A - B);
    vec3 NormalBDC = CrossProduct(D - B, C - B);
    if (DotProduct(NormalBDA, NormalBDC) >= 0.0f)
    {
        return false;
    }

    vec3 NormalACD = CrossProduct(C - A, D - A);
    vec3 NormalACB = CrossProduct(C - A, B - A);
    return (DotProduct(NormalACD, NormalACB) < 0.0f);
}

i32
PointFarthestFromEdge(vec2 A, vec2 B, vec2 *Points, i32 PointCount)
{
    vec2 Edge = B - A;
    vec2 EPerp = vec2 { -Edge.Y, Edge.X };  // CCW Perpendicular

    i32 BestIndex = -1;
    f32 MaxValue = -FLT_MAX;
    f32 RightMostValue = -FLT_MAX;

    // NOTE: This was 1 in the example (I think that's wrong)
    for (i32 PointIndex = 0; PointIndex < PointCount; ++PointIndex)
    {
        f32 Distance = DotProduct2D(Points[PointIndex] - A, EPerp);
        f32 R = DotProduct2D(Points[PointIndex] - A, Edge);
        // If there are multiple points with the same distance to edge, take the right-most value
        if (Distance > MaxValue || (Distance == MaxValue || R > RightMostValue))
        {
            BestIndex = PointIndex;
            MaxValue = Distance;
            RightMostValue = R;
        }
    }

    return BestIndex;
}
