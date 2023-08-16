#ifndef NEWO_MATH_H
#define NEWO_MATH_H

#include "NewoCommon.h"

#define PI32 3.141592653f

union vec2
{
    struct
    {
        f32 X, Y;
    };

    f32 D[2];
};

union vec3
{
    struct
    {
        f32 X, Y, Z;
    };

    f32 D[3];
};

//union mat3
//{
//    struct
//    {
//        vec3 A, B, C;
//    };
//
//    vec3 D[3];
//};

//typedef f32 vec3[3];
//typedef f32 vec4[4];
typedef f32 mat3[3][3];
//typedef f32 mat4[4][4];

struct plane
{
    vec3 Normal; // Plane normal. Points X on the plane satisfy DotProduct(Normal, X) = Distance
    f32 Distance; // Distance from origin; Distance = DotProduct(Normal, P) for a given point P on the plane
};

f32
AbsF32(f32 Value);

f32
SqrtF32(f32 Value);

f32
SinF32(f32 Value);

f32
CosF32(f32 Value);

vec3
operator+(vec3 V0, vec3 V1);

vec3
operator-(vec3 V0, vec3 V1);

vec2
operator+(vec2 V0, vec2 V1);

vec2
operator-(vec2 V0, vec2 V1);

f32
LengthVec3(vec3 V);

f32
DotProduct2D(vec2 V0, vec2 V1);

f32
DotProduct(vec3 V0, vec3 V1);

vec3
CrossProduct(vec3 V0, vec3 V1);

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