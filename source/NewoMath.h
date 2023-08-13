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

#endif