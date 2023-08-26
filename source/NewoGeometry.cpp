#include "NewoGeometry.h"

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"
#include "DynoDraw.h"

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

u32
PointFarthestFromEdge(vec2 A, vec2 B, vec2 *Points, u32 PointCount)
{
    Assert(Points);

    vec2 Edge = B - A;
    vec2 EPerp = vec2 { -Edge.Y, Edge.X };  // CCW Perpendicular

    u32 BestIndex = 0;
    f32 MaxValue = -FLT_MAX;
    f32 RightMostValue = -FLT_MAX;

    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
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
    Assert(Points);

    f32 MinProjection = FLT_MAX;
    f32 MaxProjection = -FLT_MAX;
    u32 MinIndex = 0;
    u32 MaxIndex = 0;
    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
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
    Assert(Points);

    aabb Result = {};

    for (u32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
    {
        vec3 Direction = {};
        Direction.D[AxisIndex] = 1.0f;

        u32 MinIndex;
        u32 MaxIndex;
        ExtremePointsAlongDirection(Direction, Points, PointCount, &MinIndex, &MaxIndex);

        //Assert(MinIndex > 0 && MaxIndex > 0);
        {
            f32 MinOnAxis = Points[MinIndex].D[AxisIndex];
            f32 MaxOnAxis = Points[MaxIndex].D[AxisIndex];

            Result.Extents.D[AxisIndex] = (MaxOnAxis - MinOnAxis) * 0.5f;
            Result.Center.D[AxisIndex] = MinOnAxis + Result.Extents.D[AxisIndex];
        }
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

void
MostSeparatedPointsOnAABB(vec3 *Points, u32 PointCount, u32 *Out_MinIndex, u32 *Out_MaxIndex)
{
    Assert(Points);

    u32 I_MinX = 0, I_MaxX = 0, I_MinY = 0, I_MaxY = 0, I_MinZ = 0, I_MaxZ = 0;
    f32 MinX = FLT_MAX, MinY = FLT_MAX, MinZ = FLT_MAX;
    f32 MaxX = -FLT_MAX, MaxY = -FLT_MAX, MaxZ = -FLT_MAX;
    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        if (Points[PointIndex].X < MinX) { I_MinX = PointIndex; MinX = Points[PointIndex].X; }
        if (Points[PointIndex].X > MaxX) { I_MaxX = PointIndex; MaxX = Points[PointIndex].X; }
        if (Points[PointIndex].Y < MinY) { I_MinY = PointIndex; MinY = Points[PointIndex].Y; }
        if (Points[PointIndex].Y > MaxY) { I_MaxY = PointIndex; MaxY = Points[PointIndex].Y; }
        if (Points[PointIndex].Z < MinZ) { I_MinZ = PointIndex; MinZ = Points[PointIndex].Z; }
        if (Points[PointIndex].Z > MaxZ) { I_MaxZ = PointIndex; MaxZ = Points[PointIndex].Z; }
    }

    u32 MinIndex = 0;
    u32 MaxIndex = 0;
    //Assert(I_MinX > 0 && I_MaxX > 0 && I_MinY > 0 && I_MaxY > 0 && I_MinZ > 0 && I_MaxZ > 0);
    {
        f32 DistSqX = VecLengthSq(Points[I_MaxX] - Points[I_MinX]);
        f32 DistSqY = VecLengthSq(Points[I_MaxY] - Points[I_MinY]);
        f32 DistSqZ = VecLengthSq(Points[I_MaxZ] - Points[I_MinZ]);

        MinIndex = I_MinX;
        MaxIndex = I_MaxX;

        if (DistSqY > DistSqX && DistSqY >= DistSqZ)
        {
            MinIndex = I_MinY;
            MaxIndex = I_MaxY;
        }
        if (DistSqZ > DistSqX && DistSqZ > DistSqY)
        {
            MinIndex = I_MinZ;
            MaxIndex = I_MaxZ;
        }
    }

    if (Out_MinIndex) *Out_MinIndex = MinIndex;
    if (Out_MaxIndex) *Out_MaxIndex = MaxIndex;
}

sphere
SphereFromMostSeparatedPoints(vec3 *Points, u32 PointCount)
{
    Assert(Points);

    u32 MinIndex, MaxIndex;
    MostSeparatedPointsOnAABB(Points, PointCount, &MinIndex, &MaxIndex);

    sphere Result = {};
    Result.Center = (Points[MinIndex] + Points[MaxIndex]) * 0.5f;
    Result.Radius = VecLength(Points[MaxIndex] - Result.Center);
    return Result;
}

sphere
SphereEncompassingSphereAndPoint(sphere Sphere, vec3 Point)
{
    sphere Result = Sphere;

    vec3 D = Point - Sphere.Center;
    f32 DistSq = VecLengthSq(D);

    if (DistSq > Sphere.Radius * Sphere.Radius)
    {
        f32 Dist = SqrtF32(DistSq);
        Result.Radius = (Sphere.Radius + Dist) * 0.5f;
        f32 NewCenterOffset = (Result.Radius - Sphere.Radius) / Dist;
        Result.Center += NewCenterOffset * D;
    }

    return Result;
}

sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount)
{
    Assert(Points);

    sphere Result = SphereFromMostSeparatedPoints(Points, PointCount );

    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        Result = SphereEncompassingSphereAndPoint(Result, Points[PointIndex]);
    }

    return Result;
}

f32
VarianceForF32Set(f32 *Values, u32 ValueCount)
{
    f32 Mean = 0.0f;
    for (u32 ValueIndex = 0; ValueIndex < ValueCount; ++ValueIndex)
    {
        Mean += Values[ValueIndex];
    }
    Mean /= (f32) ValueCount;

    f32 VarianceSq = 0.0f;
    for (u32 ValueIndex = 0; ValueIndex < ValueCount; ++ValueIndex)
    {
        VarianceSq += (Values[ValueIndex] - Mean) * (Values[ValueIndex] - Mean);
    }
    VarianceSq /= (f32) ValueCount;

    return VarianceSq;
}

mat3
CovarianceMatrixForPointSet(vec3 *Points, u32 PointCount)
{
    Assert(Points);
    Assert(PointCount > 1);

    f32 OneOverCount = 1.0f / (f32) (PointCount - 1);

    vec3 CenterOfMass = { 0.0f, 0.0f, 0.0f };
    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        CenterOfMass += Points[PointIndex];
    }
    CenterOfMass *= OneOverCount;

    f32 E00, E11, E22, E01, E02, E12; E00 = E11 = E22 = E01 = E02 = E12 = 0.0f;

    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        vec3 Point = Points[PointIndex] - CenterOfMass;
        E00 += Point.X * Point.X;
        E11 += Point.Y * Point.Y;
        E22 += Point.Z * Point.Z;
        E01 += Point.X * Point.Y;
        E02 += Point.X * Point.Z;
        E12 += Point.Y * Point.Z;
    }

    mat3 Result;
    Result.D[0][0] = E00 * OneOverCount;
    Result.D[1][1] = E11 * OneOverCount;
    Result.D[2][2] = E22 * OneOverCount;
    Result.D[0][1] = Result.D[1][0] = E01 * OneOverCount;
    Result.D[0][2] = Result.D[2][0] = E02 * OneOverCount;
    Result.D[1][2] = Result.D[2][1] = E12 * OneOverCount;
    return Result;
}

void
SymmetricSchur2Decomposition(mat3 A, u32 P, u32 Q, f32 *Out_C, f32 *Out_S)
{
    f32 C, S;

    if (AbsF32(A.D[P][Q]) > 0.0001f)
    {
        f32 R = (A.D[Q][Q] - A.D[P][P]) / (2.0f * A.D[P][Q]);
        f32 T;
        if (R >= 0.0f)
        {
            T = 1.0f / (R + SqrtF32(1.0f + R * R));
        }
        else
        {
            T = -1.0f / (-R + SqrtF32(1.0f + R * R));
        }
        C = 1.0f / SqrtF32(1.0f + T * T);
        S = T * C;
    }
    else
    {
        C = 1.0f;
        S = 0.0f;
    }

    if (Out_C) *Out_C = C;
    if (Out_S) *Out_S = S;
}

#define JACOBI_MAX_ITERATIONS 50
void
JacobiEigenvalues(mat3 *A, mat3 *V)
{
    Assert(A);
    Assert(V);

    *V = Mat3Identity();

    f32 PrevNorm = FLT_MAX;

    for (u32 JacobiIteration = 0; JacobiIteration < JACOBI_MAX_ITERATIONS; ++JacobiIteration)
    {
        // Find largest off-diagonal absolute element A[P][Q]
        u32 P = 0;
        u32 Q = 1;
        for (u32 Column = 0; Column < 3; ++Column)
        {
            for (u32 Row = 0; Row < 3; ++Row)
            {
                if (Column == Row) continue;
                if (AbsF32(A->D[Column][Row]) > AbsF32(A->D[P][Q]))
                {
                    P = Column;
                    Q = Row;
                }
            }
        }

        // Compute the Jacobi rotation matrix J(p, q, theta)
        // (This code can be optimized for the three different cases of rotation)
        f32 C, S;
        SymmetricSchur2Decomposition(*A, P, Q, &C, &S);
        mat3 J = Mat3Identity();
        J.D[P][P] = C;
        J.D[P][Q] = S;
        J.D[Q][P] = -S;
        J.D[Q][Q] = C;

        // Cumulate rotations into what will contain the eigenvectors
        *V = *V * J;

        // Make A more diagonal, until just eigenvalues remain on diagonal
        *A = (Mat3Transpose(J) * *A) * J;

        // Compute norm of off-diagonal elements
        f32 Norm = 0.0f;
        for (u32 Column = 0; Column < 3; ++Column)
        {
            for (u32 Row = 0; Row < 3; ++Row)
            {
                if (Column == Row) continue;
                Norm += A->D[Column][Row] * A->D[Column][Row];
            }
        }
        // NOTE: No need to take sqrt, as we're just comparing values

        // Stop when norm is no longer decreasing
        if (JacobiIteration > 2 && Norm >= PrevNorm)
        {
            return;
        }

        PrevNorm = Norm;
    }
}

sphere
SphereFromMaximumSpreadEigen(vec3 *Points, u32 PointCount)
{
    // NOTE: For the particular 3 × 3 matrix used here, instead of applying a general approach
    // such as the Jacobi method the eigenvalues could be directly computed from a simple
    // cubic equation. The eigenvectors could then easily be found through, for example,
    // Gaussian elimination. Such an approach is described in [Cromwell94].

    mat3 Covariance = CovarianceMatrixForPointSet(Points, PointCount);
    mat3 EigenvaluesM = Covariance;
    mat3 EigenvectorsM;
    JacobiEigenvalues(&EigenvaluesM, &EigenvectorsM);

    // Find the largest magnitude eigenvalue (largest spread).
    // (Find which diagonal element of EigenvaluesM is the largest magnitude.)
#if 0
    u32 MaxComponent = 0;
    f32 MaxTemp;
    f32 MaxEigenvalue = AbsF32(EigenvaluesM.D[0][0]);
    if ((MaxTemp = AbsF32(EigenvaluesM.D[1][1])) > MaxEigenvalue) MaxComponent = 1, MaxEigenvalue = MaxTemp;
    if ((MaxTemp = AbsF32(EigenvaluesM.D[2][2])) > MaxEigenvalue) MaxComponent = 2, MaxEigenvalue = MaxTemp;
#else
    u32 MaxComponent = 0;
    if (AbsF32(EigenvaluesM.D[1][1]) > AbsF32(EigenvaluesM.D[0][0])) MaxComponent = 1;
    if (AbsF32(EigenvaluesM.D[2][2]) > AbsF32(EigenvaluesM.D[1][1])) MaxComponent = 2;
#endif

    // Get the corresponding eigenvector from EigenvectorsM (nth column)
    vec3 MaxEigenvector = { EigenvectorsM.D[MaxComponent][0],
                            EigenvectorsM.D[MaxComponent][1],
                            EigenvectorsM.D[MaxComponent][2] };

    // Find the most extreme points along direction of MaxEigenvector
    u32 MinIndex, MaxIndex;
    ExtremePointsAlongDirection(MaxEigenvector, Points, PointCount, &MinIndex, &MaxIndex);
    vec3 MinPoint = Points[MinIndex];
    vec3 MaxPoint = Points[MaxIndex];

    f32 Distance = VecLength(MaxPoint - MinPoint);

    sphere Result;
    Result.Radius = Distance * 0.5f;
    Result.Center = (MinPoint + MaxPoint) * 0.5f;
    return Result;
}

sphere
GetBoundingSphereForPointSetRitterEigen(vec3 *Points, u32 PointCount)
{
    sphere Result = SphereFromMaximumSpreadEigen(Points, PointCount);

    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        Result = SphereEncompassingSphereAndPoint(Result, Points[PointIndex]);
    }

    return Result;
}
