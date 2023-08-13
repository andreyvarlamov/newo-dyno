#ifndef NEWO_MATH_H
#define NEWO_MATH_H

#include "NewoCommon.h"

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

inline f32
AbsF32(f32 Value);

inline vec3
operator+(vec3 V0, vec3 V1);

inline vec3
operator-(vec3 V0, vec3 V1);

inline f32
LengthVec3(vec3 V);

inline f32
DotProduct(vec3 V0, vec3 V1);

inline vec3
CrossProduct(vec3 V0, vec3 V1);

inline f32
TriDoubleSignedArea(vec3 A, vec3 B, vec3 C);

inline f32
TriDoubleArea2D(f32 X1, f32 Y1, f32 X2, f32 Y2, f32 X3, f32 Y3);

void
BarycentricCoordsCramer(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W);

void
BarycentricCoordsAreas(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W);

void
BarycentricCoordsProjectedAreas(vec3 P, vec3 A, vec3 B, vec3 C, f32 *U, f32 *V, f32 *W);

bool
TestPointTriangle(vec3 P, vec3 A, vec3 B, vec3 C);

#endif