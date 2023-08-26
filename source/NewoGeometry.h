#ifndef NEWO_GEOMETRY_H
#define NEWO_GEOMETRY_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"
#include "DynoDraw.h"

struct plane
{
    vec3 Normal; // Plane normal. Points X on the plane satisfy DotProduct(Normal, X) = Distance
    f32 Distance; // Distance from origin; Distance = DotProduct(Normal, P) for a given point P on the plane
};

struct aabb
{
    vec3 Center;
    vec3 Extents;
};

struct sphere
{
    vec3 Center;
    f32 Radius;
};

f32
TriDoubleSignedArea(vec3 A, vec3 B, vec3 C);

f32
TriDoubleArea2D(f32 X1, f32 Y1, f32 X2, f32 Y2, f32 X3, f32 Y3);

void
BarycentricCoordsCramer(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W);

void
BarycentricCoordsAreas(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W);

void
BarycentricCoordsProjectedAreas(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W);

bool
TestPointTriangle(vec3 P, vec3 A, vec3 B, vec3 C);

// Assumes points are non-colinear and CCW
plane
ComputePlane(vec3 A, vec3 B, vec3 C);

bool
IsQuadConvex(vec3 A, vec3 B, vec3 C, vec3 D);

u32
PointFarthestFromEdge(vec2 A, vec2 B, vec2 *Points, u32 PointCount);

internal inline vec3
VecSphericalToCartesian(f32 Theta, f32 Phi)
{
    // NOTE: Using coordinate system ZXY; Theta = Angle from axis Z in direction of X (CCW);
    // Phi = Angle from axis Y in direction ZX plane (CW, from top to down)
    vec3 Result = {};

    f32 ThetaRads = DegreesToRadians(Theta);
    f32 PhiRads = DegreesToRadians(Phi);

    Result.X = SinF32(PhiRads) * SinF32(ThetaRads);
    Result.Y = CosF32(PhiRads);
    Result.Z = SinF32(PhiRads) * CosF32(ThetaRads);

    return Result;
}

bool
TestAABBAABB(aabb A, aabb B);

void
ExtremePointsAlongDirection(vec3 Direction, vec3 *Points, u32 PointCount, u32 *Out_MinIndex, u32 *Out_MaxIndex);

aabb
GetAABBForPointSet(vec3 *Points, u32 PointCount);

// Transform A by Transform and Translation, calculate new axis-aligned
// extents and store in B
void
UpdateAABB(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B);

bool
TestSphereSphere(sphere A, sphere B);

// Calculate which 2 extreme points of a point set constitute the largest separation on AABB
// bounding the point set
void
MostSeparatedPointsOnAABB(vec3 *Points, u32 PointCount, u32 *Out_MinIndex, u32 *Out_MaxIndex);

// Calculate a sphere from the 2 most separated points on AABB bounding a set of points
sphere
SphereFromMostSeparatedPoints(vec3 *Points, u32 PointCount);

// Minimal sphere encompassing a sphere and a point (possibly) outside the sphere
sphere
SphereEncompassingSphereAndPoint(sphere Sphere, vec3 Point);

// Calculate bounding sphere for a point set by finding a sphere from the 2 most separated
// points on AABB bounding a set of points and expanding it until it envelops all points in the set
// Ritter90 -- Ericson05 - p. 128
sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount);

f32
VarianceForF32Set(f32 *Values, u32 ValueCount);

mat3
CovarianceMatrixForPointSet(vec3 *Points, u32 PointCount);

// 2-by-2 Symmetric Schur Decomposition. Given by an n-by-n symmetric matrix A
// and indices P, Q, such that 1 <= P < Q <= N. Computes a sine-cosine pair (S, C)
// that will serve to form a Jacobi rotation matrix.
// Golub96 - p.428
void
SymmetricSchur2Decomposition(mat3 A, u32 P, u32 Q, f32 *Out_C, f32 *Out_S);

// Compute the eigenvectors and eigenvalues of the symmetric matrix A using
// the classic Jacobi method of iteratively updating A as A = J^T * A * J,
// where J = J(P, Q, Theta) is the Jacobi rotation matrix.
// V will contain the eigenvectors and the diagonal elements of A are the corresponding
// eigenvalues.
// Golub96 - p.428
void
JacobiEigenvalues(mat3 *A, mat3 *V);

// Calculate the sphere by finding the axis of the largest spread of a set of points
// by using a covariance matrix and finding its eigenvalues and eigenvectors by using the Jacobi algorithm.
sphere
SphereFromMaximumSpreadEigen(vec3 *Points, u32 PointCount);

// Calculate the bounding sphere for a point set by finding the axis of maximum spread using eigenvalues.
// Constructing a sphere on thathen expanding it until it envelops all points in the set.
// Ericson05 - p.93
// NOTE: Further research on PCA (principal component analysis) - Jolliffe02
sphere
GetBoundingSphereForPointSetRitterEigen(vec3 *Points, u32 PointCount);

#endif
