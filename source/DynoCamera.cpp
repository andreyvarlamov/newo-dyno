#include "DynoCamera.h"

#include <cstdio>

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

dyno_camera
InitializeCamera(vec3 Target, f32 Radius, f32 Theta, f32 Phi)
{
    dyno_camera Result = {};

    Result.Target = Target;
    Result.Radius = Radius;
    Result.Theta = Theta;
    Result.Phi = Phi;

    return Result;
}

void
UpdateCameraPosition(dyno_camera *Camera, vec3 DeltaPositionLocal, f32 DeltaRadius, f32 DeltaTheta, f32 DeltaPhi)
{
    Camera->Theta += DeltaTheta;
    if (Camera->Theta < 0.0f)
    {
        Camera->Theta += 360.0f;
    }
    else if (Camera->Theta > 360.0f)
    {
        Camera->Theta -= 360.0f;
    }

    Camera->Phi += DeltaPhi;
    if (Camera->Phi > 179.0f)
    {
        Camera->Phi = 179.0f;
    }
    else if (Camera->Phi < 1.0f)
    {
        Camera->Phi = 1.0f;
    }

    Camera->Radius += DeltaRadius;
    if (Camera->Radius < 0.0f)
    {
        Camera->Radius = 0.0f;
    }

    if (Camera->Radius > 0.0f)
    {
        vec3 Front = -VecSphericalToCartesian(Camera->Theta, Camera->Phi);
        vec3 Right = VecCrossProduct(Front, vec3 { 0.0f, 1.0f, 0.0f });
        vec3 Up = VecCrossProduct(Right, Front);

        mat3 LocalToWorld = Mat3FromVec3Columns(Right, Up, Front);

        Camera->Target += LocalToWorld * DeltaPositionLocal;
    }
    else
    {
        Assert(false);
    }
}

mat4
GetCameraViewMat(dyno_camera *Camera)
{
    vec3 CartesianDirection = VecSphericalToCartesian(Camera->Theta, Camera->Phi);
    vec3 CameraTranslationFromTarget = Camera->Radius * CartesianDirection;
    vec3 CameraPosition = Camera->Target + CameraTranslationFromTarget;

    mat4 Result = GetViewMat(CameraPosition, Camera->Target, vec3 { 0.0f, 1.0f, 0.0f });
    return Result;
}

mat4
GetViewMat(vec3 EyePosition, vec3 TargetPoint, vec3 WorldUp)
{
    // NOTE: General formula: https://registry.khronos.org/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml
    // Translation optimization by dot product from GLM

    vec3 Front = VecNormalize(TargetPoint - EyePosition);
    vec3 Right = VecNormalize(VecCrossProduct(Front, VecNormalize(WorldUp)));
    vec3 Up = VecCrossProduct(Right, Front);

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
    Result.D[3][0] = -VecDotProduct(Right, EyePosition);
    Result.D[3][1] = -VecDotProduct(Up, EyePosition);
    Result.D[3][2] =  VecDotProduct(Front, EyePosition);

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
