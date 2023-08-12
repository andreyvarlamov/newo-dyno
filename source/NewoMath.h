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

#endif