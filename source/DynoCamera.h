#ifndef DYNO_CAMERA_H
#define DYNO_CAMERA_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

struct dyno_camera
{
    vec3 Position;
    f32 Yaw;
    f32 Pitch;
    
    vec3 _Front;
    vec3 _Right;
    vec3 _Up;
};

dyno_camera
InitializeCamera(vec3 Position, f32 Yaw, f32 Pitch);

void
UpdateCameraPosition(dyno_camera *Camera, f32 DeltaYaw, f32 DeltaPitch, vec3 LocalDeltaPosition);

mat4
GetCameraViewMat(dyno_camera *Camera);

mat4
GetPerspecitveProjectionMat(f32 FovY_Degrees, f32 AspectRatio, f32 Near, f32 Far);

// TODO: Orthographic projection
mat4
GetOrthographicProjectionMat();

mat4
GetViewMat(vec3 Position, vec3 Front, vec3 Up);

#endif