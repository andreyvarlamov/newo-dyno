#include "NewoGeometry.h"

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

f32
TriDoubleSignedArea(vec3 A, vec3 B, vec3 C)
{
    vec3 Cross = VecCross(B - A, C - A);
    return VecLength(Cross);
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
    f32 Dot00 = VecDot(V0, V0);
    f32 Dot01 = VecDot(V0, V1);
    f32 Dot11 = VecDot(V1, V1);
    f32 Denominator = Dot00 * Dot11 - Dot01 * Dot01;
    //

    f32 Dot20 = VecDot(V2, V0);
    f32 Dot21 = VecDot(V2, V1);

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
    vec3 M = VecCross(B - A, C - A); // Unnormalzied triangle normal

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

    Result.Normal = VecCross(B - A, C - A); // CCW
    Result.Distance = VecDot(Result.Normal, A);

    return Result;
}

bool
IsQuadConvex(vec3 A, vec3 B, vec3 C, vec3 D)
{
    vec3 NormalBDA = VecCross(D - B, A - B);
    vec3 NormalBDC = VecCross(D - B, C - B);
    if (VecDot(NormalBDA, NormalBDC) >= 0.0f)
    {
        return false;
    }

    vec3 NormalACD = VecCross(C - A, D - A);
    vec3 NormalACB = VecCross(C - A, B - A);
    return (VecDot(NormalACD, NormalACB) < 0.0f);
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
        f32 Distance = VecDot(Points[PointIndex] - A, EPerp);
        f32 R = VecDot(Points[PointIndex] - A, Edge);
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

bool
TestAABBAABB(aabb A, aabb B)
{
    if (AbsF32(A.Center.X - B.Center.X) > (A.Extents.X + B.Extents.X))
    {
        return false;
    }

    if (AbsF32(A.Center.Y - B.Center.Y) > (A.Extents.Y + B.Extents.Y))
    {
        return false;
    }

    if (AbsF32(A.Center.Z - B.Center.Z) > (A.Extents.Z + B.Extents.Z))
    {
        return false;
    }

    return true;
}

void
ExtremePointsAlongDirection(vec3 Direction, vec3 *Points, u32 PointCount, u32 *Out_MinIndex, u32 *Out_MaxIndex)
{
    f32 MinProjection = FLT_MAX;
    f32 MaxProjection = -FLT_MAX;
    u32 MinIndex = 0;
    u32 MaxIndex = 0;
    for (u32 PointIndex = 0; PointIndex < PointCount; ++PointIndex)
    {
        f32 Projection = VecDot(Points[PointIndex], Direction);

        if (Projection < MinProjection)
        {
            MinProjection = Projection;
            MinIndex = PointIndex;
        }
        if (Projection > MaxProjection)
        {
            MaxProjection = Projection;
            MaxIndex = PointIndex;
        }
    }

    if (Out_MinIndex) *Out_MinIndex = MinIndex;
    if (Out_MaxIndex) *Out_MaxIndex = MaxIndex;
}

aabb
GetAABBForPointSet(vec3 *Points, u32 PointCount)
{
    aabb Result = {};

    for (u32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
    {
        vec3 Direction = {};
        Direction.D[AxisIndex] = 1.0f;

        u32 MinIndex;
        u32 MaxIndex;
        ExtremePointsAlongDirection(Direction, Points, PointCount, &MinIndex, &MaxIndex);

        f32 MinOnAxis = Points[MinIndex].D[AxisIndex];
        f32 MaxOnAxis = Points[MaxIndex].D[AxisIndex];

        Result.Extents.D[AxisIndex] = (MaxOnAxis - MinOnAxis) / 2.0f;
        Result.Center.D[AxisIndex] = MinOnAxis + Result.Extents.D[AxisIndex];
    }

    return Result;
}

void
UpdateAABB(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B)
{
    // NOTE: This is equivalent to finding extreme points along the 3 cardinal directions
    // for the 8 vertices of the transformed/rotated A
    Assert(Out_B);

    for (i32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
    {
        Out_B->Center.D[AxisIndex] = Translation.D[AxisIndex];
        Out_B->Extents.D[AxisIndex] = 0.0f;
        for (i32 MatColIndex = 0; MatColIndex < 3; ++MatColIndex)
        {
            // Transform the center through normal matrix multiplication
            Out_B->Center.D[AxisIndex] += Transform.D[MatColIndex][AxisIndex] * A.Center.D[MatColIndex];
            // Transform the extents by multiplying with abs of the matrix
            Out_B->Extents.D[AxisIndex] += AbsF32(Transform.D[MatColIndex][AxisIndex]) * A.Extents.D[MatColIndex];
        }
    }
}

bool
TestSphereSphere(sphere A, sphere B)
{
    vec3 D = A.Center - B.Center;
    f32 DistanceSq = VecLengthSq(D);
    f32 RadiusSum = A.Radius + B.Radius;

    return DistanceSq <= RadiusSum * RadiusSum;
}
