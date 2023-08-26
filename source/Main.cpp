#include <cstdio>
#include <cstdlib>
#include <ctime>

#include <glad/glad.h>
#include <sdl2/SDL.h>
#include <sdl2/SDL_ttf.h>

#include "NewoCommon.h"
#include "NewoGeometry.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"
#include "DynoCamera.h"
#include "DynoDraw.h"
#include "DynoUI.h"

#define QUIT_ON_ESC 0

enum test_case
{
    TEST_CASE_AABBS_INTERSECTION,
    TEST_CASE_POINT_SET_AABB,
    TEST_CASE_OBB_BOUNDS,
    TEST_CASE_SPHERES_INTERSECTION,
    TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER,
    TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTEREIGEN,
    TEST_CASE_COUNT
};

global_variable test_case CurrentTestCase = TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTEREIGEN;

void
ProcessPointSetUpdate(const u8 *CurrentKeyStates, u8 *KeyWasDown, vec3 *PointSet, u32 *PointsUsed, u32 PointBufferCount);

void
DrawDebugVizData(dd_render_data *DDRenderData, debug_viz_data *VizData);

int
main(int Argc, char *Argv[])
{
    Assert(SDL_Init(SDL_INIT_VIDEO) >= 0);
    SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_Window *Window = SDL_CreateWindow("Newo Dyno",
                                          SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                          1920, 1080,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    Assert(Window);
    SDL_GLContext MainContext = SDL_GL_CreateContext(Window);
    Assert(MainContext);

    gladLoadGLLoader(SDL_GL_GetProcAddress);
    printf("OpenGL loaded\n");
    printf("Vendor: %s\n", glGetString(GL_VENDOR));
    printf("Renderer: %s\n", glGetString(GL_RENDERER));
    printf("Version: %s\n", glGetString(GL_VERSION));

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    Assert(TTF_Init() != -1);

    //if (SDL_SetRelativeMouseMode(SDL_TRUE) != 0)
    //{
    //    fprintf(stderr, "SDL Could not set mouse relative mode: %s\n", SDL_GetError());
    //}

    i32 ScreenWidth;
    i32 ScreenHeight;
    SDL_GetWindowSize(Window, &ScreenWidth, &ScreenHeight);
    glViewport(0, 0, ScreenWidth, ScreenHeight);

    f32 MonitorRefreshRate = 164.836f; // Hardcode from my display settings for now
    f32 DeltaTime = 1.0f / MonitorRefreshRate;

    size_t ApplicationMemorySize = Megabytes(64);
    void *ApplicationMemory = calloc(1, ApplicationMemorySize);
    //void *ApplicationMemoryCopy = calloc(1, ApplicatioMemorySize);

    memory_arena DDArena{};
    size_t DDArenaSize = Megabytes(1);
    Assert(DDArenaSize >= sizeof(dd_render_data));
    InitializeMemoryArena(&DDArena, DDArenaSize, (u8 *) ApplicationMemory);
    dd_render_data *DDRenderData = DD_InitializeRenderData(&DDArena);

    dyno_camera Camera = InitializeCamera(vec3 { 0.0f, 0.0f, 10.0f }, 10.0f, 0.0f, 90.0f, false);

    u8 KeyWasDown[SDL_NUM_SCANCODES] = {};

    bool CursorGotSwitched = true;
    bool CurrentArrow = true;

    SDL_Cursor *ArrowCursor = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_ARROW);
    SDL_Cursor *SizeAllCursor = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_SIZEALL);

    vec3 ControlledPosition = {};
    vec3 ControlledPosition2 = {};

    f32 ControlledAngle = 0.0f;

    u32 PointsUsed = 1;
    vec3 PointSet[32];
    PointSet[0] = {};

    //srand((u32) time(0));
    srand(123);
    
    //DUI_LoadFontFromFile("resource/font/FragmentMono-Italic.ttf", 18);
    //DUI_LoadFontFromFile("resource/font/PoltawskiNowy-Italic.ttf", 18);

    SDL_Event SdlEvent;
    bool ShouldQuit = false;
    while (!ShouldQuit)
    {
        while (SDL_PollEvent(&SdlEvent))
        {
            switch (SdlEvent.type)
            {
            case SDL_QUIT:
                ShouldQuit = true;
                break;
            case SDL_WINDOWEVENT:
                if (SdlEvent.window.event == SDL_WINDOWEVENT_RESIZED)
                {
                    ScreenWidth = SdlEvent.window.data1;
                    ScreenHeight = SdlEvent.window.data2;
                    glViewport(0, 0, ScreenWidth, ScreenHeight);
                }
                break;
            }
        }

        const u8 *CurrentKeyStates = SDL_GetKeyboardState(0);
#if QUIT_ON_ESC
        if (CurrentKeyStates[SDL_SCANCODE_ESCAPE])
        {
            ShouldQuit = true;
        }
#endif

        i32 MouseDeltaX;
        i32 MouseDeltaY;
        u32 MouseButtonState = SDL_GetRelativeMouseState(&MouseDeltaX, &MouseDeltaY);

        i32 MouseX;
        i32 MouseY;
        SDL_GetMouseState(&MouseX, &MouseY);

        f32 CameraRotationSensitivity = 0.1f;
        f32 CameraTranslationSensitivity = 0.01f;

        vec3 CameraTranslation = {};
        f32 CameraDeltaRadius = 0.0f;
        f32 CameraDeltaTheta = 0.0f;
        f32 CameraDeltaPhi = 0.0f;

        bool MouseMoved = (MouseDeltaX != 0 || MouseDeltaY != 0);
        bool LeftButtonPressed = (SDL_BUTTON(1) & MouseButtonState);
        bool MiddleButtonPressed = (SDL_BUTTON(2) & MouseButtonState);
        bool RightButtonPressed = (SDL_BUTTON(3) & MouseButtonState);

        bool ShouldShowCameraTarget = (CurrentKeyStates[SDL_SCANCODE_LSHIFT] ||
                                       CurrentKeyStates[SDL_SCANCODE_LCTRL] ||
                                       MiddleButtonPressed);
        if (MiddleButtonPressed && MouseMoved)
        {
            if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] && CurrentKeyStates[SDL_SCANCODE_LCTRL])
            {
                CameraDeltaRadius = (f32) -MouseDeltaY * CameraTranslationSensitivity;
            }
            else if (CurrentKeyStates[SDL_SCANCODE_LSHIFT])
            {
                CameraTranslation.X = (f32) -MouseDeltaX * CameraTranslationSensitivity;
                CameraTranslation.Y = (f32) MouseDeltaY * CameraTranslationSensitivity;
            }
            else if (CurrentKeyStates[SDL_SCANCODE_LCTRL])
            {
                CameraDeltaRadius = (f32) MouseDeltaY * CameraTranslationSensitivity;
                CameraTranslation.Z = (f32) -MouseDeltaY * CameraTranslationSensitivity;
            }
            else
            {
                CameraDeltaTheta = (f32) -MouseDeltaX * CameraRotationSensitivity;
                CameraDeltaPhi = (f32) -MouseDeltaY * CameraRotationSensitivity;
            }
        }

        if (LeftButtonPressed && MouseMoved)
        {
            ControlledAngle += (f32) MouseDeltaX * CameraRotationSensitivity;
            if (ControlledAngle > 360.0f)
            {
                ControlledAngle -= 360.0f;
            }
            else if (ControlledAngle < 0.0f)
            {
                ControlledAngle += 360.0f;
            }
        }

        UpdateCameraOrientation(&Camera, CameraDeltaRadius, CameraDeltaTheta, CameraDeltaPhi);
        UpdateCameraPosition(&Camera, CameraTranslation);

        if (CurrentKeyStates[SDL_SCANCODE_HOME] && !KeyWasDown[SDL_SCANCODE_HOME])
        {
            Camera.IsLookAround = !Camera.IsLookAround;
            KeyWasDown[SDL_SCANCODE_HOME] = true;
        }
        else if (!CurrentKeyStates[SDL_SCANCODE_HOME])
        {
            KeyWasDown[SDL_SCANCODE_HOME] = false;
        }

        if ((MouseX > ((ScreenWidth / 2) - 5)) &&
            (MouseX < ((ScreenWidth / 2) + 5)) &&
            (MouseY > ((ScreenHeight / 2) - 5)) &&
            (MouseY < ((ScreenHeight / 2) + 5)))
        {
            SDL_SetCursor(SizeAllCursor);
            CursorGotSwitched = true;
        }
        else if (CursorGotSwitched)
        {
            SDL_SetCursor(ArrowCursor);
            CursorGotSwitched = false;
        }

        vec3 ControlledVelocity = {};
        vec3 ControlledVelocity2 = {};
        f32 ControlledSpeed = 3.0f;
        vec3 *ControlledPositionPtr = &ControlledPosition;
        vec3 *ControlledVelocityPtr = &ControlledVelocity;
        if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
        {
            ControlledVelocityPtr = &ControlledVelocity2;
            ControlledPositionPtr = &ControlledPosition2;
        }

        if (CurrentKeyStates[SDL_SCANCODE_UP])
        {
            ControlledVelocityPtr->Z -= 1.0f;
        }
        if (CurrentKeyStates[SDL_SCANCODE_DOWN])
        {
            ControlledVelocityPtr->Z += 1.0f;
        }
        if (CurrentKeyStates[SDL_SCANCODE_LEFT])
        {
            ControlledVelocityPtr->X -= 1.0f;
        }
        if (CurrentKeyStates[SDL_SCANCODE_RIGHT])
        {
            ControlledVelocityPtr->X += 1.0f;
        }
        if (CurrentKeyStates[SDL_SCANCODE_PAGEUP])
        {
            ControlledVelocityPtr->Y += 1.0f;
        }
        if (CurrentKeyStates[SDL_SCANCODE_PAGEDOWN])
        {
            ControlledVelocityPtr->Y -= 1.0f;
        }
        *ControlledVelocityPtr = VecNormalize(*ControlledVelocityPtr);
        *ControlledPositionPtr += DeltaTime * ControlledSpeed * (*ControlledVelocityPtr);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Coordinate system
        DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, vec3 { 0.0f, 0.0f, 0.0f }, vec3 { 5.0f, 0.0f, 0.0f }, vec3 { 1.0f, 0.0f, 0.0f });
        DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, vec3 { 0.0f, 0.0f, 0.0f }, vec3 { 0.0f, 5.0f, 0.0f }, vec3 { 0.0f, 1.0f, 0.0f });
        DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, vec3 { 0.0f, 0.0f, 0.0f }, vec3 { 0.0f, 0.0f, 5.0f }, vec3 { 0.0f, 0.0f, 1.0f });

        // Camera data
        if (ShouldShowCameraTarget && !Camera.IsLookAround)
        {
            vec3 CameraTargetPosition = GetCameraTarget(&Camera);
            vec3 ProjTargetPosition = vec3 { CameraTargetPosition.X, 0.0f, CameraTargetPosition.Z };
            DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, vec3 { 0.0f, 0.0f, 0.0f }, ProjTargetPosition, vec3 { 0.3f, 0.3f, 0.3f });
            DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, CameraTargetPosition, vec3 { 0.3f, 0.3f, 0.3f });
        }

        // Tests
        switch (CurrentTestCase)
        {
            case TEST_CASE_AABBS_INTERSECTION:
            {
                //
                // NOTE: AABBs intersection test
                //
                aabb A = {};
                A.Center = ControlledPosition;
                A.Extents = vec3 { 0.5f, 0.5f, 0.5f };

                aabb B = {};
                B.Center = vec3 { 2.0f, 0.0f, 0.0f };
                B.Extents = vec3 { 0.5f, 0.5f, 0.5f };

                bool AreIntersecting = TestAABBAABB(A, B, DEBUG_VIZ_NONE);

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                if (AreIntersecting)
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }
                DD_DrawAABox(DDRenderData, PRIM_STYLE_WIREFRAME, A.Center, A.Extents, Color);
                DD_DrawAABox(DDRenderData, PRIM_STYLE_WIREFRAME, B.Center, B.Extents, Color);
            } break;
            case TEST_CASE_POINT_SET_AABB:
            {
                //
                // NOTE: AABB for an arbitrary set of points
                //
                glDisable(GL_CULL_FACE);

                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet));
                for (u32 PointIndex = 1; PointIndex < PointsUsed; ++PointIndex)
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_DEPTHTEST, PointSet[PointIndex], vec3 { 1.0f, 0.0f, 0.0f });
                }

                if (PointsUsed > 3)
                {
                    aabb AABB = GetAABBForPointSet(PointSet, PointsUsed, DEBUG_VIZ_NONE);

                    if (AABB.Extents.X > 0.0f && AABB.Extents.Y > 0.0f && AABB.Extents.Z > 0.0f)
                    {
                        DD_DrawAABox(DDRenderData, PRIM_STYLE_WIREFRAME, AABB.Center, AABB.Extents, vec3 { 1.0f, 1.0f, 0.0f });
                    }
                }
            } break;
            case TEST_CASE_OBB_BOUNDS:
            {
                //
                // NOTE: Rotating a box and recalculating AABB around it
                //
                glDisable(GL_CULL_FACE);

                aabb OrientedBox = {}; // Used just as an oriented box together with the BoxOrientation matrix
                OrientedBox.Center = {};
                OrientedBox.Extents = vec3 { 0.5f, 0.5f, 0.5f };
                vec3 Axis = VecNormalize(ControlledPosition2 + vec3 { 0.0f, 1.0f, 0.0f } - OrientedBox.Center);
                f32 Angle = ControlledAngle;
                mat3 BoxOrientation = Mat3GetRotationAroundAxis(Axis, DegreesToRadians(Angle));

                aabb OrientedBoxBounds;
                GetAABBForOrientedBox(OrientedBox, BoxOrientation, ControlledPosition, &OrientedBoxBounds, DEBUG_VIZ_NONE);

                DD_VisualizeRotationMat(DDRenderData, VECTOR_STYLE_DEPTHTEST, BoxOrientation, 2.0f, ControlledPosition, vec3 { 1.0f, 0.5f, 0.0f });
                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_TRANSPARENT, ControlledPosition, OrientedBox.Extents, BoxOrientation, vec3 { 1.0f, 0.0f, 0.0f });
                DD_DrawAABox(DDRenderData, PRIM_STYLE_WIREFRAME, OrientedBoxBounds.Center, OrientedBoxBounds.Extents, vec3 { 1.0f, 1.0f, 0.0f });

            } break;
            case TEST_CASE_SPHERES_INTERSECTION:
            {
                //
                // NOTE: Test intersection of 2 spheres
                //
                glDisable(GL_CULL_FACE);

                sphere A = {};
                A.Center = ControlledPosition;
                A.Radius = 1.0f;

                sphere B = {};
                B.Center = { 4.0f, 0.0f, 0.0f };
                B.Radius = 2.0f;

                bool AreIntersecting = TestSphereSphere(A, B, DEBUG_VIZ_NONE);

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                if (AreIntersecting)
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }

                DD_DrawSphere(DDRenderData, PRIM_STYLE_WIREFRAME, A.Center, A.Radius, Color, 29, 30);
                DD_DrawSphere(DDRenderData, PRIM_STYLE_WIREFRAME, B.Center, B.Radius, Color, 29, 30);
            } break;
            case TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER:
            {
                glDisable(GL_CULL_FACE);

                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet));
                for (u32 PointIndex = 1; PointIndex < PointsUsed; ++PointIndex)
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_DEPTHTEST, PointSet[PointIndex], vec3 { 1.0f, 0.0f, 0.0f });
                }

                if (PointsUsed > 3)
                {
                    sphere BoundingSphere = GetBoundingSphereForPointSetRitter(PointSet, PointsUsed, DEBUG_VIZ_NONE);
                    aabb AABB = GetAABBForPointSet(PointSet, PointsUsed, DEBUG_VIZ_NONE);

                    if (BoundingSphere.Radius > 0.0f)
                    {
                        DD_DrawSphere(DDRenderData, PRIM_STYLE_WIREFRAME, BoundingSphere.Center, BoundingSphere.Radius, vec3 { 0.0f, 0.0f, 0.5f }, 29, 30);
                    }
                    if (AABB.Extents.X > 0.0f && AABB.Extents.Y > 0.0f && AABB.Extents.Z > 0.0f)
                    {
                        DD_DrawAABox(DDRenderData, PRIM_STYLE_WIREFRAME, AABB.Center, AABB.Extents, vec3 { 1.0f, 1.0f, 0.0f });
                    }
                }
            } break;
            case TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTEREIGEN:
            {
                glDisable(GL_CULL_FACE);

                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet));
                for (u32 PointIndex = 1; PointIndex < PointsUsed; ++PointIndex)
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_DEPTHTEST, PointSet[PointIndex], vec3 { 1.0f, 0.0f, 0.0f });
                }

                if (PointsUsed > 3)
                {
                    debug_viz_data VizData = {};
                    sphere BoundingSphere = GetBoundingSphereForPointSetRitter(PointSet, PointsUsed, DEBUG_VIZ_NONE);
                    sphere BoundingSphere2 = GetBoundingSphereForPointSetRitterEigen(PointSet, PointsUsed, &VizData);
                    aabb AABB = GetAABBForPointSet(PointSet, PointsUsed, DEBUG_VIZ_NONE);

                    if (BoundingSphere.Radius > 0.0f)
                    {
                        DD_DrawSphere(DDRenderData, PRIM_STYLE_WIREFRAME, BoundingSphere.Center, BoundingSphere.Radius, vec3 { 0.0f, 0.0f, 0.5f }, 29, 30);
                    }
                    if (BoundingSphere2.Radius > 0.0f)
                    {
                        DD_DrawSphere(DDRenderData, PRIM_STYLE_WIREFRAME, BoundingSphere2.Center, BoundingSphere2.Radius, vec3 { 0.5f, 0.5f, 0.0f }, 29, 30);
                    }
                    if (AABB.Extents.X > 0.0f && AABB.Extents.Y > 0.0f && AABB.Extents.Z > 0.0f)
                    {
                        DD_DrawAABox(DDRenderData, PRIM_STYLE_WIREFRAME, AABB.Center, AABB.Extents, vec3 { 1.0f, 1.0f, 0.0f });
                    }

                    DrawDebugVizData(DDRenderData, &VizData);
                }
            } break;
            default:
            {
                Assert(!"Unknown test case");
            } break;
        }
        
        mat4 ProjectionMat = GetPerspecitveProjectionMat(70.0f, (f32) ScreenWidth / (f32) ScreenHeight, 0.1f, 1000.0f);
        mat4 ViewMat = GetCameraViewMat(&Camera);
        DD_Render(DDRenderData, ProjectionMat, ViewMat);

        SDL_GL_SwapWindow(Window);
    }

    SDL_Quit();
    return 0;
}

void
ProcessPointSetUpdate(const u8 *CurrentKeyStates, u8 *KeyWasDown, vec3 *PointSet, u32 *PointsUsed, u32 PointBufferCount)
{
    Assert(PointBufferCount > 3);

    if (*PointsUsed <= 1)
    {
        PointSet[0] = {};
        for (u32 PointIndex = 1; PointIndex < 4; ++PointIndex)
        {
            for (u32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
            {
                PointSet[PointIndex].D[AxisIndex] = ((f32) (rand() % 255) * 10.0f / 255.0f) - 5.0f;
            }
        }
        *PointsUsed = 4;
    }

    if (CurrentKeyStates[SDL_SCANCODE_SPACE] && !KeyWasDown[SDL_SCANCODE_SPACE])
    {
        if (CurrentKeyStates[SDL_SCANCODE_LSHIFT])
        {
            PointSet[0] = {};
            *PointsUsed = 1;
        }
        else
        {
            if (*PointsUsed + 1 < PointBufferCount)
            {
                for (u32 AxisIndex = 0; AxisIndex < 3; ++AxisIndex)
                {
                    PointSet[*PointsUsed].D[AxisIndex] = ((f32) (rand() % 255) * 10.0f / 255.0f) - 5.0f;
                }
                (*PointsUsed)++;
            }
        }
        KeyWasDown[SDL_SCANCODE_SPACE] = true;
    }
    else if (!CurrentKeyStates[SDL_SCANCODE_SPACE])
    {
        KeyWasDown[SDL_SCANCODE_SPACE] = false;
    }
}

void
DrawDebugVizData(dd_render_data *DDRenderData, debug_viz_data *VizData)
{
    for (u32 DotIndex = 0; DotIndex < VizData->DotsUsed; ++DotIndex)
    {
        DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, VizData->Dots[DotIndex], VizData->DotColors[DotIndex]);
    }

    for (u32 VectorIndex = 0; VectorIndex < VizData->VectorsUsed; ++VectorIndex)
    {
        DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, VizData->VectorStarts[VectorIndex], VizData->VectorEnds[VectorIndex], VizData->VectorColors[VectorIndex]);
    }
}
