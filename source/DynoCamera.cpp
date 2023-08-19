#include "DynoCamera.h"

#include <cstdio>

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

internal inline mat4
GetViewMatRaw(vec3 Translation, vec3 Back, vec3 Right, vec3 Up);

internal inline void
CalculateCameraInternals(dyno_camera *Camera);

dyno_camera
InitializeCamera(vec3 Position, f32 Yaw, f32 Pitch)
{
    dyno_camera Result = {};

    Result.Position = Position;
    Result.Yaw = Yaw;
    Result.Pitch = Pitch;

    CalculateCameraInternals(&Result);

    return Result;
}

void
UpdateCameraPosition(dyno_camera *Camera, f32 DeltaYaw, f32 DeltaPitch, vec3 LocalDeltaPosition)
{
    Camera->Yaw += DeltaYaw;
    if (Camera->Yaw < 0.0f)
    {
        Camera->Yaw += 360.0f;
    }
    else if (Camera->Yaw > 360.0f)
    {
        Camera->Yaw -= 360.0f;
    }

    Camera->Pitch += DeltaPitch;
    if (Camera->Pitch > 89.0f)
    {
        Camera->Pitch = 89.0f;
    }
    else if (Camera->Pitch < -89.0f)
    {
        Camera->Pitch = -89.0f;
    }

    CalculateCameraInternals(Camera);

    printf("Yaw: %.3f, Pitch: %.3f, Front: {%.3f, %.3f, %.3f}, Right: {%.3f, %.3f, %.3f}, Up: {%.3f, %.3f, %.3f}\n",
           Camera->Yaw, Camera->Pitch,
           Camera->_Front.X, Camera->_Front.Y, Camera->_Front.Z,
           Camera->_Right.X, Camera->_Right.Y, Camera->_Right.Z,
           Camera->_Up.X, Camera->_Up.Y, Camera->_Up.Z);

    mat3 LocalToWorld = Mat3FromVec3Columns(Camera->_Right, Camera->_Up, Camera->_Front);
    
    Camera->Position += LocalToWorld * LocalDeltaPosition;
}

mat4
GetCameraViewMat(dyno_camera *Camera)
{
    // NOTE: Front vector is flipped, because the view matrix expects "back" vector, because the coordinate system
    // we're using for the world is right-handed. Only the camera's internal coordinate system is left-handed.
    mat4 Result = GetViewMatRaw(Camera->Position, -Camera->_Front, Camera->_Right, Camera->_Up);
    return Result;
}

mat4
GetPerspecitveProjectionMat(f32 FovY_Degrees, f32 AspectRatio, f32 Near, f32 Far)
{
    // NOTE: http://www.songho.ca/opengl/gl_projectionmatrix.html
    mat4 Result = Mat4Identity();

    f32 HalfHeight = Near * TanF32(DegreesToRadians(FovY_Degrees) / 2.0f);
    f32 HalfWidth = HalfHeight * AspectRatio;

#if 0
    f32 Left = -HalfWidth;
    f32 Right = HalfWidth;
    f32 Top = HalfHeight;
    f32 Bottom = -HalfHeight;

    Result.D[0][0] = 2.0f * Near / (Right - Left);

    Result.D[1][1] = 2.0f * Near / (Top - Bottom);

    Result.D[2][0] = (Right + Left) / (Right - Left);
    Result.D[2][1] = (Top + Bottom) / (Top - Bottom);
    Result.D[2][2] = -(Far + Near) / (Far - Near);
    Result.D[2][3] = -1.0f;

    Result.D[3][2] = -2.0f * Far * Near / (Far - Near);
#else
    // For symmetrical frustum
    Result.D[0][0] = Near / HalfWidth;

    Result.D[1][1] = Near / HalfHeight;

    Result.D[2][2] = -(Far + Near) / (Far - Near);
    Result.D[2][3] = -1.0f;

    Result.D[3][2] = -2.0f * Far * Near / (Far - Near);
#endif

    return Result;
}

mat4
GetViewMat(vec3 EyePosition, vec3 TargetPoint, vec3 WorldUp)
{
    vec3 Back = VecNormalize(EyePosition - TargetPoint);
    vec3 Right = VecNormalize(VecCrossProduct(VecNormalize(WorldUp), Back));
    vec3 Up = VecCrossProduct(Back, Right);

    mat4 Result = GetViewMatRaw(EyePosition, Back, Right, Up);
    return Result;
}

internal inline mat4
GetViewMatRaw(vec3 Translation, vec3 Back, vec3 Right, vec3 Up)
{
    mat4 Result = Mat4Identity();

#if 0
    Result.D[0][0] = Right.X;
    Result.D[0][1] = Up.X;
    Result.D[0][2] = Back.X;

    Result.D[1][0] = Right.Y;
    Result.D[1][1] = Up.Y;
    Result.D[1][2] = Back.Y;

    Result.D[2][0] = Right.Z;
    Result.D[2][1] = Up.Z;
    Result.D[2][2] = Back.Z;
#endif

    Result.D[0][0] = Right.X;
    Result.D[1][0] = Up.X;
    Result.D[2][0] = Back.X;

    Result.D[0][1] = Right.Y;
    Result.D[1][1] = Up.Y;
    Result.D[2][1] = Back.Y;

    Result.D[0][2] = Right.Z;
    Result.D[1][2] = Up.Z;
    Result.D[2][2] = Back.Z;


    Result.D[3][0] = -Translation.X;
    Result.D[3][1] = -Translation.Y;
    Result.D[3][2] = -Translation.Z;

    return Result;
}

internal inline void
CalculateCameraInternals(dyno_camera *Camera)
{
    Camera->_Front = vec3 { 0.0f, 0.0f, 0.0f };
    f32 PitchRad = DegreesToRadians(Camera->Pitch);
    f32 YawRad = DegreesToRadians(Camera->Yaw);
    Camera->_Front.X = CosF32(PitchRad) * CosF32(YawRad);
    Camera->_Front.Y = SinF32(PitchRad);
    Camera->_Front.Z = CosF32(PitchRad) * SinF32(YawRad);
    // NOTE: For the calculation of the camera position, flip Z-axis and make the coordinate system left-handed
    // as opposed to right-handed. Because we want the camera to move in the direction of the front vector
    // when going forward. But in right-handed system, in which the world is, forward, into the screen, is negative Z.
    Camera->_Right = VecNormalize(VecCrossProduct(Camera->_Front, vec3 { 0.0f, 1.0f, 0.0f }));
    Camera->_Up = VecCrossProduct(Camera->_Right, Camera->_Front);
}
