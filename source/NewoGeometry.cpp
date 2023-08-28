#include "NewoGeometry.h"

#include <cfloat>
#include <cstdlib>

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"
#include "DynoDraw.h"

#pragma warning(push)
#pragma warning(disable: 4505)

//
// VISUALIZATION --------------------------------------------------------------
//

internal void
AddVizDot(debug_viz_data *VizData, vec3 Position, vec3 Color)
{
    if (VizData != DEBUG_VIZ_NONE)
    {
        Assert(VizData->DotsUsed < DEBUG_VIZ_DOTS);
        VizData->Dots[VizData->DotsUsed] = Position;
        VizData->DotColors[VizData->DotsUsed] = Color;
        VizData->DotsUsed++;
    }
}

internal void
AddVizVector(debug_viz_data *VizData, vec3 Start, vec3 End, vec3 Color)
{
    if (VizData != DEBUG_VIZ_NONE)
    {
        Assert(VizData->VectorsUsed < DEBUG_VIZ_VECTORS);
        VizData->VectorStarts[VizData->VectorsUsed] = Start;
        VizData->VectorEnds[VizData->VectorsUsed] = End;
        VizData->VectorColors[VizData->VectorsUsed] = Color;
        VizData->VectorsUsed++;
    }
}

//
// TRIANGLE/BARYCENTRIC (Ericson05 Ch. 3.4) ----------------------------------------
//

internal f32
TriDoubleSignedArea(vec3 A, vec3 B, vec3 C)
{
    vec3 Cross = VecCross(B - A, C - A);
    return VecLength(Cross);
}

internal f32
TriDoubleArea2D(f32 X1, f32 Y1, f32 X2, f32 Y2, f32 X3, f32 Y3)
{
    return (X1 - X2) * (Y2 - Y3) - (X2 - X3) * (Y1 - Y2);
}

internal void
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

internal void
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

internal void
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

internal bool
TestPointTriangle(vec3 P, vec3 A, vec3 B, vec3 C)
{
    f32 V;
    f32 W;
    BarycentricCoordsProjectedAreas(P, A, B, C, NULL, &V, &W);
    return ((V >= 0.0f) && (W >= 0.0f) && ((V + W) <= 1.0f));
}

//
// MISC GEOMETRY (Ericson05 - Ch. 3.5 - 3.6) --------------------------------------
//

// Assumes points are non-colinear and CCW
internal plane
ComputePlane(vec3 A, vec3 B, vec3 C)
{
    plane Result;

    Result.Normal = VecCross(B - A, C - A); // CCW
    Result.Distance = VecDot(Result.Normal, A);

    return Result;
}

internal bool
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

internal u32
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

//
// AABB TEST/CONSTRUCTION (Ericson05 Ch. 4.2) --------------------------------------------
// -- Internal functions
//

internal void
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

//
// AABB TEST/CONSTRUCTION (Ericson05 Ch. 4.2) --------------------------------------------
// -- External functions
//

bool
TestAABBAABB(aabb A, aabb B, debug_viz_data *VizData)
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

aabb
GetAABBForPointSet(vec3 *Points, u32 PointCount, debug_viz_data *VizData)
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
GetAABBForOrientedBox(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B, debug_viz_data *VizData)
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

//
// SPHERE TEST/CONSTRUCTION (Ericson05 - Ch. 4.3) --------------------------------------------------
// -- Internal functions
//

// Calculate which 2 extreme points of a point set constitute the largest separation on AABB
// bounding the point set
internal void
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

// Calculate a sphere from the 2 most separated points on AABB bounding a set of points
internal sphere
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

// Minimal sphere encompassing a sphere and a point (possibly) outside the sphere
internal sphere
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

internal f32
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

internal mat3
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

// 2-by-2 Symmetric Schur Decomposition. Given by an n-by-n symmetric matrix A
// and indices P, Q, such that 1 <= P < Q <= N. Computes a sine-cosine pair (S, C)
// that will serve to form a Jacobi rotation matrix.
// Golub96 - p.428
internal void
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

// Compute the eigenvectors and eigenvalues of the symmetric matrix A using
// the classic Jacobi method of iteratively updating A as A = J^T * A * J,
// where J = J(P, Q, Theta) is the Jacobi rotation matrix.
// V will contain the eigenvectors and the diagonal elements of A are the corresponding
// eigenvalues.
// Golub96 - p.428
#define JACOBI_MAX_ITERATIONS 50
internal void
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

// Calculate the sphere by finding the axis of the largest spread of a set of points
// by using a covariance matrix and finding its eigenvalues and eigenvectors by using the Jacobi algorithm.
internal sphere
SphereFromMaximumSpreadEigen(vec3 *Points, u32 PointCount, debug_viz_data *VizData)
{
    // NOTE: For the particular 3 � 3 matrix used here, instead of applying a general approach
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
    AddVizDot(VizData, MinPoint, vec3 { 1.0f, 1.0f, 1.0f });
    AddVizDot(VizData, MaxPoint, vec3 { 1.0f, 1.0f, 1.0f });

    f32 Distance = VecLength(MaxPoint - MinPoint);

    sphere Result;
    Result.Radius = Distance * 0.5f;
    Result.Center = (MinPoint + MaxPoint) * 0.5f;
    AddVizVector(VizData, Result.Center - 10.0f * MaxEigenvector, Result.Center + 10.0f * MaxEigenvector, vec3 { 1.0f, 1.0f, 1.0f });
    return Result;
}

//
// SPHERE TEST/CONSTRUCTION (Ericson05 - Ch. 4.3) --------------------------------------------------
// -- External functions
//

bool
TestSphereSphere(sphere A, sphere B, debug_viz_data *VizData)
{
    vec3 D = A.Center - B.Center;
    f32 DistanceSq = VecLengthSq(D);
    f32 RadiusSum = A.Radius + B.Radius;

    return DistanceSq <= RadiusSum * RadiusSum;
}

sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount, debug_viz_data *VizData)
{
    Assert(Points);

    sphere Result = SphereFromMostSeparatedPoints(Points, PointCount );

    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        Result = SphereEncompassingSphereAndPoint(Result, Points[PointIndex]);
    }

    return Result;
}

sphere
GetBoundingSphereForPointSetRitterEigen(vec3 *Points, u32 PointCount, debug_viz_data *VizData)
{
    sphere Result = SphereFromMaximumSpreadEigen(Points, PointCount, VizData);

    for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
    {
        Result = SphereEncompassingSphereAndPoint(Result, Points[PointIndex]);
    }

    return Result;
}

#define RITTER_ITERATIVE_MAX_ITERATIONS 8
sphere
GetBoundingSphereForPointSetRitterIterative(vec3 *Points, u32 PointCount, debug_viz_data *VizData)
{
    sphere Result = GetBoundingSphereForPointSetRitter(Points, PointCount, VizData);
    
    sphere BetterResult = Result;

    for (u32 IterationIndex = 0; IterationIndex < RITTER_ITERATIVE_MAX_ITERATIONS; ++IterationIndex)
    {
        BetterResult.Radius = BetterResult.Radius * 0.95f;

        for (u32 PointIndex = 1; PointIndex < PointCount; ++PointIndex)
        {
            // Random swap of points
            if (PointIndex < PointCount - 1)
            {
                i32 RandomRange = PointCount - (PointIndex + 1);
                u32 IndexToSwap = PointIndex + 1;
                if (RandomRange > 0)
                {
                    IndexToSwap += (rand() % RandomRange);
                }
                vec3 Temp = Points[PointIndex];
                Points[PointIndex] = Points[IndexToSwap];
                Points[IndexToSwap] = Temp;
            }

            BetterResult = SphereEncompassingSphereAndPoint(BetterResult, Points[PointIndex]);

        }

        if (BetterResult.Radius < Result.Radius)
        {
            Result = BetterResult;
        }
    }

    return Result;
}


//
// OBB TEST/CONSTRUCTION (Ericson05 - Ch. 4.4) --------------------------------------------------
// -- Internal functions
//


//
// OBB TEST/CONSTRUCTION (Ericson05 - Ch. 4.4) --------------------------------------------------
// -- External functions
//

bool
TestOBBOBB(obb A, obb B, debug_viz_data *VizData)
{
    // Rotation matrix expressing B in A's coordinate frame
    mat3 Rotation;
    for (u32 Column = 0; Column < 3; ++Column)
    {
        for (u32 Row = 0; Row < 3; ++Row)
        {
            Rotation.D[Column][Row] = VecDot(A.Axes[Row], B.Axes[Column]);
        }
    }

    // Translation vector from A to B
    vec3 Translation = B.Center - A.Center;
    // Bring it into A's coordinate frame
    Translation = vec3 { VecDot(Translation, A.Axes[0]),
                         VecDot(Translation, A.Axes[1]),
                         VecDot(Translation, A.Axes[2]) };

    // Compute common subexpressions. Add in an epsilon term to counteract arithmetic errors
    // when two edges are parallel and their cross product is (near) 0.
    mat3 AbsRotation;
    for (u32 Column = 0; Column < 3; ++Column)
    {
        for (u32 Row = 0; Row < 3; ++Row)
        {
            AbsRotation.D[Column][Row] = AbsF32(Rotation.D[Column][Row]) + FLT_EPSILON;
        }
    }

    f32 RadiusA, RadiusB;

    // NOTE: The 15 axes to be tested defined in Ericson05 - p.103, Table 4.1

    // Test axes L = A0, L = A1, L = A2
    for (u32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
    {
        RadiusA = A.Extents.D[AxisIndex];
        RadiusB = (B.Extents.D[0] * AbsRotation.D[0][AxisIndex] +
                   B.Extents.D[1] * AbsRotation.D[1][AxisIndex] +
                   B.Extents.D[2] * AbsRotation.D[2][AxisIndex]);
        if (AbsF32(Translation.D[AxisIndex]) > RadiusA + RadiusB)
        {
            return false;
        }
    }

    // Test axes L = B0, L = B1, L = B2
    for (u32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
    {
        RadiusA = (A.Extents.D[0] * AbsRotation.D[AxisIndex][0] +
                   A.Extents.D[1] * AbsRotation.D[AxisIndex][1] +
                   A.Extents.D[2] * AbsRotation.D[AxisIndex][2]);
        RadiusB = B.Extents.D[AxisIndex];
        f32 ProjT = (Translation.D[0] * Rotation.D[AxisIndex][0] +
                     Translation.D[1] * Rotation.D[AxisIndex][1] +
                     Translation.D[2] * Rotation.D[AxisIndex][2]);
        if (AbsF32(ProjT) > RadiusA + RadiusB)
        {
            return false;
        }
    }

    // Test axis L = A0 x B0
    RadiusA = A.Extents.D[1] * AbsRotation.D[0][2] + A.Extents.D[2] * AbsRotation.D[0][1];
    RadiusB = B.Extents.D[1] * AbsRotation.D[2][0] + B.Extents.D[2] * AbsRotation.D[1][0];
    if (AbsF32(Translation.D[2] * Rotation.D[0][1] - Translation.D[1] * Rotation.D[0][2]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A0 x B1
    RadiusA = A.Extents.D[1] * AbsRotation.D[1][2] + A.Extents.D[2] * AbsRotation.D[1][1];
    RadiusB = B.Extents.D[0] * AbsRotation.D[2][0] + B.Extents.D[2] * AbsRotation.D[0][0];
    if (AbsF32(Translation.D[2] * Rotation.D[1][1] - Translation.D[1] * Rotation.D[1][2]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A0 x B2
    RadiusA = A.Extents.D[1] * AbsRotation.D[2][2] + A.Extents.D[2] * AbsRotation.D[2][1];
    RadiusB = B.Extents.D[0] * AbsRotation.D[1][0] + B.Extents.D[1] * AbsRotation.D[0][0];
    if (AbsF32(Translation.D[2] * Rotation.D[2][1] - Translation.D[1] * Rotation.D[2][2]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A1 x B0
    RadiusA = A.Extents.D[0] * AbsRotation.D[0][2] + A.Extents.D[2] * AbsRotation.D[0][0];
    RadiusB = B.Extents.D[1] * AbsRotation.D[2][1] + B.Extents.D[2] * AbsRotation.D[1][1];
    if (AbsF32(Translation.D[0] * Rotation.D[0][2] - Translation.D[2] * Rotation.D[0][0]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A1 x B1
    RadiusA = A.Extents.D[0] * AbsRotation.D[1][2] + A.Extents.D[2] * AbsRotation.D[1][0];
    RadiusB = B.Extents.D[0] * AbsRotation.D[2][1] + B.Extents.D[2] * AbsRotation.D[0][1];
    if (AbsF32(Translation.D[0] * Rotation.D[1][2] - Translation.D[2] * Rotation.D[1][0]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A1 x B2
    RadiusA = A.Extents.D[0] * AbsRotation.D[2][2] + A.Extents.D[2] * AbsRotation.D[2][0];
    RadiusB = B.Extents.D[0] * AbsRotation.D[1][1] + B.Extents.D[1] * AbsRotation.D[0][1];
    if (AbsF32(Translation.D[0] * Rotation.D[2][2] - Translation.D[2] * Rotation.D[2][0]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A2 x B0
    RadiusA = A.Extents.D[0] * AbsRotation.D[0][1] + A.Extents.D[1] * AbsRotation.D[0][0];
    RadiusB = B.Extents.D[1] * AbsRotation.D[2][2] + B.Extents.D[2] * AbsRotation.D[1][2];
    if (AbsF32(Translation.D[1] * Rotation.D[0][0] - Translation.D[0] * Rotation.D[0][1]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A2 x B1
    RadiusA = A.Extents.D[0] * AbsRotation.D[1][1] + A.Extents.D[1] * AbsRotation.D[1][0];
    RadiusB = B.Extents.D[0] * AbsRotation.D[2][2] + B.Extents.D[2] * AbsRotation.D[0][2];
    if (AbsF32(Translation.D[1] * Rotation.D[1][0] - Translation.D[0] * Rotation.D[1][1]) > RadiusA + RadiusB)
    {
        return false;
    }

    // Test axis L = A2 x B2
    RadiusA = A.Extents.D[0] * AbsRotation.D[2][1] + A.Extents.D[1] * AbsRotation.D[2][0];
    RadiusB = B.Extents.D[0] * AbsRotation.D[1][2] + B.Extents.D[1] * AbsRotation.D[0][2];
    if (AbsF32(Translation.D[1] * Rotation.D[2][0] - Translation.D[0] * Rotation.D[2][1]) > RadiusA + RadiusB)
    {
        return false;
    }

    // No separating axis found, must be intersecting
    return true;
}

//
// NOTE: SSV (Sphere-swept volumes) Tests
// -- Internal functions
//

f32 SegmentPointDistSq(vec3 Start, vec3 End, vec3 Point)
{
    vec3 AB = End - Start;
    vec3 AC = Point - Start;
    vec3 BC = Point - End;

    f32 Proj = VecDot(AC, AB);
    if (Proj <= 0.0f) return VecLengthSq(AC);
    f32 SegmentLengthSq = VecLengthSq(AB);
    if (Proj >= SegmentLengthSq) return VecLengthSq(BC);
    f32 DistSq = VecLengthSq(AC) - Proj * Proj / SegmentLengthSq;
    return DistSq;
}

f32 SegmentSegmentClosestPoint(vec3 AStart, vec3 AEnd, vec3 BStart, vec3 BEnd,
                               f32 *Out_S, f32 *Out_T, vec3 *Out_PointOnA, vec3 *Out_PointOnB)
{
    vec3 ADir = AEnd - AStart;
    vec3 BDir = BEnd - BStart;
    f32 ALengthSq = VecLengthSq(ADir);
    f32 BLengthSq = VecLengthSq(BDir);

    f32 S, T;

    if (ALengthSq <= FLT_EPSILON && BLengthSq <= FLT_EPSILON)
    {
        // NOTE: Both lines degenerate -> Distance between points
        if (Out_S) *Out_S = 0.0f;
        if (Out_T) *Out_T = 0.0f;
        if (Out_PointOnA) *Out_PointOnA = AStart;
        if (Out_PointOnB) *Out_PointOnB = BStart;
        return VecLengthSq(AStart - BStart);
    }
    else
    {
        vec3 R = AStart - BStart;
        f32 ProjOnB = VecDot(R, BDir);
        if (ALengthSq <= FLT_EPSILON)
        {
            // NOTE: Line A degenerate -> Distance between point A and line B
            S = 0.0f;
            T = ClampF32(ProjOnB / BLengthSq, 0.0f, 1.0f);
        }
        else
        {
            f32 ProjOnA = VecDot(R, ADir);
            if (BLengthSq <= FLT_EPSILON)
            {
                // NOTE: Line B degenerate -> Distance between line A and point B
                S = ClampF32(-ProjOnA / ALengthSq, 0.0f, 1.0f);
                T = 0.0f;
            }
            else
            {
                // NOTE: General non-degenerate case
                f32 Proj = VecDot(ADir, BDir);
                f32 Denom = ALengthSq * BLengthSq - Proj * Proj;

                // If segments are parallel pick any S, else compute closest point on line A to line B
                // and clamp to the segment A.
                if (Denom <= FLT_EPSILON)
                {
                    S = 0.0f;
                }
                else
                {
                    S = ClampF32((Proj * ProjOnB - ProjOnA * BLengthSq) / Denom, 0.0f, 1.0f);
                }

                // Compute point on line B closest to segment A
                T = (Proj * S + ProjOnB) / BLengthSq;

                // If T in [0, 1] -> done. Else clamp T and recompute S
                if (T < 0.0f)
                {
                    S = ClampF32(-ProjOnA / ALengthSq, 0.0f, 1.0f);
                    T = 0.0f;
                }
                else if (T > 1.0f)
                {
                    S = ClampF32((Proj - ProjOnA) / ALengthSq, 0.0f, 1.0f);
                    T = 1.0f;
                }
            }
        }
    }

    vec3 PointOnA = AStart + ADir * S;
    vec3 PointOnB = BStart + BDir * T;
    if (Out_S) *Out_S = S;
    if (Out_T) *Out_T = T;
    if (Out_PointOnA) *Out_PointOnA = PointOnA;
    if (Out_PointOnB) *Out_PointOnB = PointOnB;
    return VecLengthSq(PointOnA - PointOnB);
}

//
// NOTE: SSV (Sphere-swept volumes) Tests
// -- External functions
//

bool
TestSphereCapsule(sphere S, capsule C, debug_viz_data *VizData)
{
    f32 DistSq = SegmentPointDistSq(C.Start, C.End, S.Center);
    f32 Radius = S.Radius + C.Radius;
    bool Result = (DistSq <= Radius * Radius);
    return Result;
}

bool
TestCapsuleCapsule(capsule A, capsule B, debug_viz_data *VizData)
{
    f32 DistSq = SegmentSegmentClosestPoint(A.Start, A.End, B.Start, B.End, NULL, NULL, NULL, NULL);
    f32 Radius = A.Radius + B.Radius;
    bool Result = (DistSq <= Radius * Radius);
    return Result;
}

#pragma warning(pop)
