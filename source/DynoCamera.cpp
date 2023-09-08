#include "DynoCamera.h"

#include <cstdio>

#include "NewoCommon.h"
#include "NewoGeometry.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

dyno_camera
InitializeCamera(vec3 Position, f32 Radius, f32 Theta, f32 Phi, bool IsLookAround)
{
    dyno_camera Result = {};

    Result.Position = Position;
    Result.Radius = ((Radius > 1.0f) ? Radius : 1.0f);
    Result.Theta = Theta;
    Result.Phi = Phi;
    Result.IsLookAround = IsLookAround;

    return Result;
}

void
UpdateCameraOrientation(dyno_camera *Camera, f32 DeltaRadius, f32 DeltaTheta, f32 DeltaPhi)
{
    f32 OldTheta = Camera->Theta;
    Camera->Theta += DeltaTheta;
    if (Camera->Theta < 0.0f)
    {
        Camera->Theta += 360.0f;
    }
    else if (Camera->Theta > 360.0f)
    {
        Camera->Theta -= 360.0f;
    }

    f32 OldPhi = Camera->Phi;
    Camera->Phi += DeltaPhi;
    if (Camera->Phi > 179.0f)
    {
        Camera->Phi = 179.0f;
    }
    else if (Camera->Phi < 1.0f)
    {
        Camera->Phi = 1.0f;
    }

    f32 OldRadius = Camera->Radius;
    Camera->Radius += DeltaRadius;
    if (Camera->Radius < 1.0f)
    {
        Camera->Radius = 1.0f;
    }
    
    if (!Camera->IsLookAround)
    {
        vec3 TranslationFromOldPositionToTarget = OldRadius * -VecSphericalToCartesian(OldTheta, OldPhi);
        vec3 TranslationFromTargetToNewPosition = OldRadius * VecSphericalToCartesian(Camera->Theta, Camera->Phi);
        Camera->Position += TranslationFromOldPositionToTarget + TranslationFromTargetToNewPosition;
    }
}

void
UpdateCameraPosition(dyno_camera *Camera, vec3 DeltaPositionLocal)
{
    vec3 Front = -VecSphericalToCartesian(Camera->Theta, Camera->Phi);
    vec3 Right = VecCross(Front, vec3 { 0.0f, 1.0f, 0.0f });
    vec3 Up = VecCross(Right, Front);
    mat3 LocalToWorld = Mat3FromCols(Right, Up, Front);
    Camera->Position += LocalToWorld * DeltaPositionLocal;
}

mat4
GetCameraViewMat(dyno_camera *Camera)
{
    vec3 Front = -VecSphericalToCartesian(Camera->Theta, Camera->Phi);
    vec3 CameraTarget;
    if (Camera->IsLookAround)
    {
        CameraTarget = Camera->Position + Front;
    }
    else
    {
        CameraTarget = Camera->Position + Camera->Radius * Front;
    }

    mat4 Result = GetViewMat(Camera->Position, CameraTarget, vec3 { 0.0f, 1.0f, 0.0f });
    return Result;
}

vec3
GetCameraTarget(dyno_camera *Camera)
{
    vec3 Result = Camera->Position;

    if (!Camera->IsLookAround)
    {
        vec3 TranslationToTarget = Camera->Radius * -VecSphericalToCartesian(Camera->Theta, Camera->Phi);
        Result += TranslationToTarget;
    }

    return Result;
}

vec3
GetCameraLookDirection(dyno_camera *Camera)
{
    vec3 Front = -VecSphericalToCartesian(Camera->Theta, Camera->Phi);
    return Front;
}

mat4
GetViewMat(vec3 EyePosition, vec3 TargetPoint, vec3 WorldUp)
{
    // NOTE: General formula: https://registry.khronos.org/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml
    // Translation optimization by dot product from GLM

    vec3 Front = VecNormalize(TargetPoint - EyePosition);
    vec3 Right = VecNormalize(VecCross(Front, VecNormalize(WorldUp)));
    vec3 Up = VecCross(Right, Front);

    mat4 Result = Mat4Identity();
    Result.D[0][0] =  Right.X;
    Result.D[0][1] =  Up.X;
    Result.D[0][2] = -Front.X;
    Result.D[1][0] =  Right.Y;
    Result.D[1][1] =  Up.Y;
    Result.D[1][2] = -Front.Y;
    Result.D[2][0] =  Right.Z;
    Result.D[2][1] =  Up.Z;
    Result.D[2][2] = -Front.Z;
    Result.D[3][0] = -VecDot(Right, EyePosition);
    Result.D[3][1] = -VecDot(Up, EyePosition);
    Result.D[3][2] =  VecDot(Front, EyePosition);

    return Result;
}

mat4
GetPerspecitveProjectionMat(f32 FovY_Degrees, f32 AspectRatio, f32 Near, f32 Far)
{
    // NOTE: http://www.songho.ca/opengl/gl_projectionmatrix.html
    mat4 Result = {};

    f32 HalfHeight = Near * TanF32(DegreesToRadians(FovY_Degrees) / 2.0f);
    f32 HalfWidth = HalfHeight * AspectRatio;

    Result.D[0][0] = Near / HalfWidth;
    Result.D[1][1] = Near / HalfHeight;
    Result.D[2][2] = -(Far + Near) / (Far - Near);
    Result.D[2][3] = -1.0f;
    Result.D[3][2] = -2.0f * Far * Near / (Far - Near);

    return Result;
}
