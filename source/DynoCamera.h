#ifndef DYNO_CAMERA_H
#define DYNO_CAMERA_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

struct dyno_camera
{
    vec3 Position;
    f32 Radius;
    f32 Theta;
    f32 Phi;
    bool IsLookAround;
};

dyno_camera
InitializeCamera(vec3 Position, f32 Radius, f32 Theta, f32 Phi, bool IsLookAround);

void
UpdateCameraOrientation(dyno_camera *Camera, f32 DeltaRadius, f32 DeltaTheta, f32 DeltaPhi);

void
UpdateCameraPosition(dyno_camera *Camera, vec3 DeltaPositionLocal);

mat4
GetCameraViewMat(dyno_camera *Camera);

vec3
GetCameraTarget(dyno_camera *Camera);

vec3
GetCameraLookDirection(dyno_camera *Camera);

mat4
GetViewMat(vec3 Position, vec3 Front, vec3 Up);

mat4
GetPerspecitveProjectionMat(f32 FovY_Degrees, f32 AspectRatio, f32 Near, f32 Far);

// TODO: Orthographic projection
mat4
GetOrthographicProjectionMat();

#endif