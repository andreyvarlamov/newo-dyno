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

i32
PointFarthestFromEdge(vec2 A, vec2 B, vec2 *Points, i32 PointCount);

#endif
