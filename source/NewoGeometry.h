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

struct obb
{
    vec3 Center;
    vec3 Axes[3];
    vec3 Extents;
};

struct capsule
{
    vec3 Start;
    vec3 End;
    f32 Radius;
};

struct lozenge
{
    vec3 Origin;
    vec3 Edges[2];
    f32 Radius;
};

#define DEBUG_VIZ_DOTS 64
#define DEBUG_VIZ_VECTORS 64
// Don't know if this is a good idea
#define DEBUG_VIZ_NONE 0
struct debug_viz_data
{
    vec3 Dots[DEBUG_VIZ_DOTS];
    vec3 DotColors[DEBUG_VIZ_DOTS];
    u32 DotsUsed;

    vec3 VectorStarts[DEBUG_VIZ_VECTORS];
    vec3 VectorEnds[DEBUG_VIZ_VECTORS];
    vec3 VectorColors[DEBUG_VIZ_VECTORS];
    u32 VectorsUsed;
};

bool
TestAABBAABB(aabb A, aabb B, debug_viz_data *VizData);

aabb
GetAABBForPointSet(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Transform A by Transform and Translation, calculate new axis-aligned
// extents and store in B
void
GetAABBForOrientedBox(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B, debug_viz_data *VizData);

bool
TestSphereSphere(sphere A, sphere B, debug_viz_data *VizData);

// Calculate bounding sphere for a point set by finding a sphere from the 2 most separated
// points on AABB bounding a set of points and expanding it until it envelops all points in the set
// Ritter90 -- Ericson05 - p. 128
sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Calculate the bounding sphere for a point set by finding the axis of maximum spread using eigenvalues.
// Constructing a sphere on thathen expanding it until it envelops all points in the set.
// Aka Wu's algorithm. Also something more that I found: https://ep.liu.se/ecp/034/009/ecp083409.pdf
// Ericson05 - p.93
// NOTE: Further research on PCA (principal component analysis) - Jolliffe02
sphere
GetBoundingSphereForPointSetRitterEigen(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Calculate bounding sphere for a point set by using the ritter method, then trying to minimize it by shrinking
// the sphere and re-extending it to fit all points in a number of iterations
// Ericson05 - p.99
sphere
GetBoundingSphereForPointSetRitterIterative(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Exact minimum sphere using Welzl algorithm.
// Not implementing it now, as I don't know how to compute exact sphere for 0-4 support points.
// And the algorithm doesn't seem that useful for real-time game applications.
// Ericson05 - p.100
// sphere
// WelzlSphere(vec3 *Points, u32 PointCount, vec3 SetOfSupportPoints, u32 SetOfSupportPointCount);

bool
TestOBBOBB(obb A, obb B, debug_viz_data *VizData);

// NOTE: For more info on computing tight OBBs about polyhedra, other references, see Ericson05 p. 108-112

f32 SegmentSegmentClosestPoint(vec3 AStart, vec3 AEnd, vec3 BStart, vec3 BEnd,
                               f32 *Out_S, f32 *Out_T, vec3 *Out_PointOnA, vec3 *Out_PointOnB);

bool
TestSphereCapsule(sphere S, capsule C);

bool
TestCapsuleCapsule(capsule A, capsule B);

#endif
