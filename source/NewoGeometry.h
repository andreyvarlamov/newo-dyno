#ifndef NEWO_GEOMETRY_H
#define NEWO_GEOMETRY_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

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

// NOTE: Transform A by Transform and Translation, calculate new axis-aligned
// extents and store in B
void
UpdateAABB(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B);

bool
TestSphereSphere(sphere A, sphere B);

void
MostSeparatedPointsOnAABB(vec3 *Points, u32 PointCount, u32 *Out_MinIndex, u32 *Out_MaxIndex);

sphere
SphereFromMostSeparatedPoints(vec3 *Points, u32 PointCount);

sphere
SphereEncompassingSphereAndPoint(sphere Sphere, vec3 Point);

sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount);

#endif
