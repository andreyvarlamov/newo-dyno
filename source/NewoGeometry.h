#ifndef NEWO_GEOMETRY_H
#define NEWO_GEOMETRY_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"
#include "DynoDraw.h"

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

bool
TestAABBAABB(aabb A, aabb B);

aabb
GetAABBForPointSet(vec3 *Points, u32 PointCount);

// Transform A by Transform and Translation, calculate new axis-aligned
// extents and store in B
void
GetAABBForOrientedBox(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B);

bool
TestSphereSphere(sphere A, sphere B);

// Calculate bounding sphere for a point set by finding a sphere from the 2 most separated
// points on AABB bounding a set of points and expanding it until it envelops all points in the set
// Ritter90 -- Ericson05 - p. 128
sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount);

// Calculate the bounding sphere for a point set by finding the axis of maximum spread using eigenvalues.
// Constructing a sphere on thathen expanding it until it envelops all points in the set.
// Ericson05 - p.93
// NOTE: Further research on PCA (principal component analysis) - Jolliffe02
sphere
GetBoundingSphereForPointSetRitterEigen(vec3 *Points, u32 PointCount);

#endif
