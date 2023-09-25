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
    TEST_CASE_INTERSECTION_AABBS,
    TEST_CASE_POINT_SET_AABB,
    TEST_CASE_OBB_BOUNDS,
    TEST_CASE_INTERSECTION_SPHERES,
    TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER,
    TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER_EIGEN,
    TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER_ITERATIVE,
    TEST_CASE_INTERSECTION_OBBS,
    TEST_CASE_INTERSECTION_SPHERE_CAPSULE,
    TEST_CASE_INTERSECTION_CAPSULE_CAPSULE,
    TEST_CASE_CLOSEST_POINT_POINT_SEGMENT,
    TEST_CASE_CLOSEST_POINT_POINT_AABB,
    TEST_CASE_CLOSEST_POINT_POINT_OBB,
    TEST_CASE_CLOSEST_POINT_POINT_RECT,
    TEST_CASE_CLOSEST_POINT_POINT_TRIANGLE,
    TEST_CASE_CLOSEST_POINT_POINT_TETRAHEDRON,
    TEST_CASE_CLOSEST_POINT_SEGMENT_SEGMENT,
    TEST_CASE_INTERSECTION_SPHERE_AABB,
    TEST_CASE_INTERSECTION_SPHERE_OBB,
    TEST_CASE_INTERSECTION_SPHERE_TRIANGLE,
    TEST_CASE_INTERSECTION_TRIANGLE_BOX,
    TEST_CASE_INTERSECTION_TRIANGLE_BOX_2,
    TEST_CASE_INTERSECTION_TRIANGLE_TRIANGLE,
    TEST_CASE_MOUSE_PICKING,
    TEST_CASE_COUNT
};

global_variable test_case CurrentTestCase = TEST_CASE_INTERSECTION_TRIANGLE_BOX_2;

void
ProcessPointSetUpdate(const u8 *CurrentKeyStates, u8 *KeyWasDown, vec3 *PointSet, u32 *PointsUsed, u32 PointBufferCount, bool *PointSetChanged);

#define MAX_POINT_SET_COUNT 32

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
    vec3 ControlledPosition3 = {};
    vec3 ControlledPosition4 = {};

    f32 ControlledAngle = 0.0f;
    f32 ControlledAngle2 = 0.0f;

    u32 PointsUsed = 1;
    vec3 PointSet[MAX_POINT_SET_COUNT];
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

        f32 *ControlledAnglePtr = &ControlledAngle;
        if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
        {
            ControlledAnglePtr = &ControlledAngle2;
        }

        if ((CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
            && LeftButtonPressed && MouseMoved)
        {
            *ControlledAnglePtr += (f32) MouseDeltaX * CameraRotationSensitivity;
            if (*ControlledAnglePtr > 360.0f)
            {
                *ControlledAnglePtr -= 360.0f;
            }
            else if (*ControlledAnglePtr < 0.0f)
            {
                *ControlledAnglePtr += 360.0f;
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
        vec3 ControlledVelocity3 = {};
        vec3 ControlledVelocity4 = {};
        f32 ControlledSpeed = 3.0f;
        vec3 *ControlledPositionPtr = &ControlledPosition;
        vec3 *ControlledVelocityPtr = &ControlledVelocity;
        if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
        {
            ControlledVelocityPtr = &ControlledVelocity3;
            ControlledPositionPtr = &ControlledPosition3;
        }
        if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
        {
            if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
            {
                ControlledVelocityPtr = &ControlledVelocity4;
                ControlledPositionPtr = &ControlledPosition4;
            }
            else
            {
                ControlledVelocityPtr = &ControlledVelocity2;
                ControlledPositionPtr = &ControlledPosition2;
            }
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
        if (Camera.IsLookAround)
        {
            vec3 CameraTarget = Camera.Position + GetCameraLookDirection(&Camera);
            DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, CameraTarget, vec3 { 1.0f, 1.0f, 1.0f });
        }

        // Tests
        switch (CurrentTestCase)
        {
            case TEST_CASE_INTERSECTION_AABBS:
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

                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet), NULL);
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
            case TEST_CASE_INTERSECTION_SPHERES:
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

                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet), NULL);
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
            case TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER_EIGEN:
            {
                glDisable(GL_CULL_FACE);

                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet), NULL);
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
            case TEST_CASE_POINT_SET_BOUNDING_SPHERE_RITTER_ITERATIVE:
            {
                glDisable(GL_CULL_FACE);

                bool PointSetChanged;
                ProcessPointSetUpdate(CurrentKeyStates, KeyWasDown, PointSet, &PointsUsed, ArrayCount(PointSet), &PointSetChanged);
                for (u32 PointIndex = 1; PointIndex < PointsUsed; ++PointIndex)
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_DEPTHTEST, PointSet[PointIndex], vec3 { 1.0f, 0.0f, 0.0f });
                }

                if (PointsUsed > 3)
                {
#if 0
                    sphere BoundingSphere = GetBoundingSphereForPointSetRitter(PointSet, PointsUsed, DEBUG_VIZ_NONE);
#else
                    sphere BoundingSphere = GetBoundingSphereForPointSetRitterEigen(PointSet, PointsUsed, DEBUG_VIZ_NONE);
#endif
                    vec3 TempPointSet[MAX_POINT_SET_COUNT] = {};
                    for (u32 PointIndex = 0; PointIndex < PointsUsed; ++PointIndex)
                    {
                        TempPointSet[PointIndex] = PointSet[PointIndex];
                    }
                    local_persist sphere BoundingSphere2;
                    if (PointSetChanged)
                    {
                        BoundingSphere2 = GetBoundingSphereForPointSetRitterIterative(TempPointSet, PointsUsed, DEBUG_VIZ_NONE);
                    }
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
                }

            } break;
            case TEST_CASE_INTERSECTION_OBBS:
            {
                //
                // NOTE: Test intersection of 2 oriented boxes using Separating Axis Theorem
                //

                glDisable(GL_CULL_FACE);

                obb A;
                A.Center = ControlledPosition;
                A.Extents = { 0.5f, 0.5f, 0.5f };
                vec3 OrientationAxisPoint = vec3 { 0.0f, 1.0f, 0.0f } + ControlledPosition2;
                Mat3GetCols(Mat3GetRotationAroundAxis(VecNormalize(OrientationAxisPoint), 
                                                      DegreesToRadians(ControlledAngle)),
                            A.Axes);
                obb B;
                B.Center = vec3 { 3.0f, 0.0f, 0.0f } + ControlledPosition3;
                B.Extents = { 1.0f, 0.5f, 0.5f };
                vec3 OrientationAxisPoint2 = vec3 { 0.0f, 1.0f, 0.0f } + ControlledPosition4;
                Mat3GetCols(Mat3GetRotationAroundAxis(VecNormalize(OrientationAxisPoint2),
                                                      DegreesToRadians(ControlledAngle2)),
                            B.Axes);
                vec3 Color = { 1.0f, 1.0f, 1.0f };
                if (TestOBBOBB(A, B, DEBUG_VIZ_NONE))
                {
                    Color = { 0.0f, 1.0f, 0.0f };
                }
                if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
                {
                    if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
                    {
                        DD_VisualizeRotationMat(DDRenderData, VECTOR_STYLE_DEPTHTEST, Mat3FromCols(B.Axes), 2.0f,
                                                B.Center, vec3 { 1.0f, 0.5f, 0.0f });
                        DD_DrawDot(DDRenderData, VECTOR_STYLE_DEPTHTEST, B.Center + OrientationAxisPoint2, vec3 { 0.0f, 1.0f, 0.0f });
                    }
                    else
                    {
                        DD_VisualizeRotationMat(DDRenderData, VECTOR_STYLE_DEPTHTEST, Mat3FromCols(A.Axes), 2.0f,
                                                ControlledPosition, vec3 { 1.0f, 0.5f, 0.0f });
                        DD_DrawDot(DDRenderData, VECTOR_STYLE_DEPTHTEST, A.Center + OrientationAxisPoint, vec3 { 0.0f, 1.0f, 0.0f });
                    }
                }

                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_WIREFRAME, A.Center, A.Extents, Mat3FromCols(A.Axes), Color);
                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_WIREFRAME, B.Center, B.Extents, Mat3FromCols(B.Axes), Color);
            } break;
            case TEST_CASE_INTERSECTION_SPHERE_CAPSULE:
            {
                sphere Sphere;
                Sphere.Center = vec3 { 0.0f, 0.0f, 0.0f } + ControlledPosition;
                Sphere.Radius = 1.0f;

                capsule Capsule;
                Capsule.Start = vec3 { 5.0f, 0.0f, 0.0f } + ControlledPosition3;
                Capsule.End = vec3 { 5.0f, 1.0f, 0.0f } + ControlledPosition3 + ControlledPosition4;
                Capsule.Radius = 0.8f;

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                if (TestSphereCapsule(Sphere, Capsule, DEBUG_VIZ_NONE))
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }

                DD_DrawVector(DDRenderData, VECTOR_STYLE_DEPTHTEST, Capsule.Start, Capsule.End, vec3 { 1.0f, 0.0f, 0.0f });

                DD_DrawSphere(DDRenderData, PRIM_STYLE_TRANSPARENT, Sphere.Center, Sphere.Radius, Color, 29, 30);
                DD_DrawCapsule(DDRenderData, PRIM_STYLE_TRANSPARENT, Capsule.Start, Capsule.End, Capsule.Radius, Color, 15, 30);
            } break;
            case TEST_CASE_INTERSECTION_CAPSULE_CAPSULE:
            {
                capsule A;
                A.Start = vec3 { 0.0f, 0.0f, 0.0f } + ControlledPosition;
                A.End = vec3 { 0.0f, 1.0f, 0.0f } + ControlledPosition + ControlledPosition2;
                A.Radius = 1.0f;

                capsule B;
                B.Start = vec3 { 5.0f, 0.0f, 0.0f } + ControlledPosition3;
                B.End = vec3 { 5.0f, 1.0f, 0.0f } + ControlledPosition3 + ControlledPosition4;
                B.Radius = 0.8f;

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                if (TestCapsuleCapsule(A, B, DEBUG_VIZ_NONE))
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }

                DD_DrawVector(DDRenderData, VECTOR_STYLE_DEPTHTEST, A.Start, A.End, vec3 { 1.0f, 0.0f, 0.0f });
                DD_DrawVector(DDRenderData, VECTOR_STYLE_DEPTHTEST, B.Start, B.End, vec3 { 1.0f, 0.0f, 0.0f });

                DD_DrawCapsule(DDRenderData, PRIM_STYLE_TRANSPARENT, A.Start, A.End, A.Radius, Color, 15, 30);
                DD_DrawCapsule(DDRenderData, PRIM_STYLE_TRANSPARENT, B.Start, B.End, B.Radius, Color, 15, 30);
            } break;
            case TEST_CASE_CLOSEST_POINT_POINT_SEGMENT:
            {
                vec3 Point = vec3 { -1.0f, 0.0f, 0.0f } + vec3 { ControlledPosition.X, -ControlledPosition.Z, 0.0f };
                vec3 Start = vec3 { 1.0f, 0.0f, 0.0f };
                vec3 End = vec3 { 1.0f, 1.0f, 0.0f };
                f32 T;
                vec3 ClosestPoint = ClosestPointPointSegment(Point, Start, End, &T);
                f32 DistSq = DistSqPointSegment(Point, Start, End);
                printf("T = %.3f; DistSq = %.3f\n", T, DistSq);

                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, Start, End, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });
                
                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint, vec3 { 0.0f, 1.0f, 0.0f });
            } break;
            case TEST_CASE_CLOSEST_POINT_POINT_AABB:
            {
                vec3 Point = vec3 { -1.0f, 0.0f, 0.0f } + ControlledPosition;
                aabb AABB;
                AABB.Center = vec3 { 1.0f, 0.0f, 0.0f };
                AABB.Extents = vec3 { 1.0f, 1.0f, 1.0f };
                
                vec3 ClosestPoint = ClosestPointPointAABB(Point, AABB);
                f32 DistSq = DistSqPointAABB(Point, AABB);
                printf("DistSq = %.3f\n", DistSq);

                DD_DrawAABox(DDRenderData, PRIM_STYLE_TRANSPARENT, AABB.Center, AABB.Extents, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });
                
                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint, vec3 { 0.0f, 1.0f, 0.0f });
            } break;
            case TEST_CASE_CLOSEST_POINT_POINT_OBB:
            {
                vec3 Point = vec3 { -1.0f, 0.0f, 0.0f } + ControlledPosition;
                obb OBB;
                OBB.Center = vec3 { 1.0f, 0.0f, 0.0f };
                mat3 Rotation = Mat3GetRotationAroundAxis(VecNormalize(vec3 { 1.0f, 1.0f, 1.0f}), DegreesToRadians(45.0f));
                Mat3GetCols(Rotation, OBB.Axes);
                OBB.Extents = vec3 { 1.0f, 1.0f, 1.0f };
                
                vec3 ClosestPoint = ClosestPointPointOBB(Point, OBB);
                f32 DistSq = DistSqPointOBB(Point, OBB);
                printf("DistSq = %.3f\n", DistSq);

                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_TRANSPARENT, OBB.Center, OBB.Extents, Rotation, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });
                
                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint, vec3 { 0.0f, 1.0f, 0.0f });
            } break;
            case TEST_CASE_CLOSEST_POINT_POINT_RECT:
            {
                glDisable(GL_CULL_FACE);

                vec3 Point = vec3 { -1.0f, 0.0f, 0.0f } + ControlledPosition;

                rect Rect;
                Rect.Center = vec3 { 2.0f, 1.0f, 0.0f };
                vec3 OrientationAxisPoint = vec3 { 0.0f, 1.0f, 0.0f } + ControlledPosition2;
                mat3 Rotation = Mat3GetRotationAroundAxis(VecNormalize(OrientationAxisPoint), DegreesToRadians(ControlledAngle2));
                Rect.Axes[0] = Mat3GetCol(Rotation, 0);
                Rect.Axes[1] = Mat3GetCol(Rotation, 1);
                Rect.Extents = vec2 { 1.0f, 0.5f };

                vec3 ClosestPoint = ClosestPointPointRect(Point, Rect);
                vec3 ClosestPoint2 = ClosestPointPointRect(Point,
                                                           Rect.Center - Rect.Extents.X * Rect.Axes[0] + Rect.Extents.Y * Rect.Axes[1],
                                                           Rect.Center - Rect.Extents.X * Rect.Axes[0] - Rect.Extents.Y * Rect.Axes[1],
                                                           Rect.Center + Rect.Extents.X * Rect.Axes[0] + Rect.Extents.Y * Rect.Axes[1]);

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint, vec3 { 0.0f, 1.0f, 0.0f });
                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint2, vec3 { 1.0f, 0.0f, 0.0f });

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Rect.Center, vec3 { 0.9f, 0.9f, 0.9f });
                DD_DrawRect(DDRenderData, PRIM_STYLE_TRANSPARENT, Rect.Center, Rect.Axes, Rect.Extents, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });

                DD_VisualizeRotationMat(DDRenderData, VECTOR_STYLE_OVERLAY, Rotation, 2.0f, Rect.Center, vec3 { 0.5f, 0.5f, 0.5f });
            } break;
            case TEST_CASE_CLOSEST_POINT_POINT_TRIANGLE:
            {
                glDisable(GL_CULL_FACE);

                vec3 Point = vec3 { -1.0f, 1.0f, 0.0f } + ControlledPosition;

                vec3 A = vec3 { 1.5f, 1.0f, 0.0f } + ControlledPosition2;
                vec3 B = vec3 { 2.5f, 1.0f, 0.0f } + ControlledPosition3;
                vec3 C = vec3 { 2.0f, 2.0f, 0.0f } + ControlledPosition4;

                vec3 Centroid = (A + B + C) / 3.0f;
                vec3 Normal = VecNormalize(VecCross(B - A, C - A));

                vec3 ClosestPoint = ClosestPointPointTriangle(Point, A, B, C);

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawTriangle(DDRenderData, PRIM_STYLE_TRANSPARENT, A, B, C, vec3 { 1.0f, 1.0f, 1.0f });
                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, Centroid, Centroid + 2.0f * Normal, vec3 { 0.0f, 0.0f, 1.0f });
                if ((CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT]) &&
                    (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT]))
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, C, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, A, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, B, vec3 { 1.0f, 0.0f, 1.0f });
                }

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint, vec3 { 0.0f, 1.0f, 0.0f });
            } break;
            case TEST_CASE_CLOSEST_POINT_POINT_TETRAHEDRON:
            {
                glDisable(GL_CULL_FACE);

                vec3 Point = vec3 { -1.0f, 1.0f, 0.0f } + ControlledPosition;

                vec3 A = vec3 { 2.0f, 2.0f, 0.0f };
                vec3 B = vec3 { 1.5f, 1.0f, 0.5f };
                vec3 C = vec3 { 2.5f, 1.0f, 0.5f };
                vec3 D = vec3 { 2.0f, 1.0f, -0.5f };

                vec3 ClosestPoint = ClosestPointPointTetrahedron(Point, A, B, C, D);

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawTetrahedron(DDRenderData, PRIM_STYLE_WIREFRAME, A, B, C, D, vec3 { 1.0f, 1.0f, 1.0f });

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, ClosestPoint, vec3 { 0.0f, 1.0f, 0.0f });
            } break;
            case TEST_CASE_CLOSEST_POINT_SEGMENT_SEGMENT:
            {
                vec3 AStart = vec3 { -1.0f, 0.0f, 0.0f } + ControlledPosition;
                vec3 AEnd = vec3 { -1.0f, 1.0f, 0.0f } + ControlledPosition2;
                vec3 BStart = vec3 { 1.0f, 0.0f, 0.0f } + ControlledPosition3;
                vec3 BEnd = vec3 { 1.0f, 1.0f, 0.0f } + ControlledPosition4;

                f32 S, T;
                vec3 PointOnA, PointOnB;
                f32 DistSq = ClosestPointSegmentSegment(AStart, AEnd, BStart, BEnd, &S, &T, &PointOnA, &PointOnB);    
                printf("DistSq = %.3f; S (on A) = %.3f; T (on B) = %.3f\n", DistSq, S, T);

                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, PointOnA, vec3 { 0.0f, 1.0f, 0.0f });
                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, PointOnB, vec3 { 0.0f, 1.0f, 0.0f });

                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, AStart, AEnd, vec3 { 1.0f, 1.0f, 1.0f });
                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, BStart, BEnd, vec3 { 1.0f, 1.0f, 1.0f });
            } break;
            case TEST_CASE_INTERSECTION_TRIANGLE_BOX:
            {
                glDisable(GL_CULL_FACE);

                vec3 Point = vec3 { -1.0f, 1.0f, 0.0f } + ControlledPosition;
                DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, Point, vec3 { 1.0f, 1.0f, 1.0f });

                obb Box;
                Box.Center = vec3 { -1.0f, 0.0f, 0.0f };
                Box.Extents = vec3 { 0.5f, 0.5f, 0.5f };
                mat3 Rotation = Mat3GetRotationAroundAxis(VecNormalize(Point - Box.Center), DegreesToRadians(45.0f));
                Mat3GetCols(Rotation, Box.Axes);

                vec3 A = vec3 { -0.8f, 0.0f, 0.0f } + ControlledPosition2;
                vec3 B = vec3 { 2.5f, 1.0f, 5.0f } + ControlledPosition3;
                vec3 C = vec3 { 2.0f, 2.0f, 0.0f } + ControlledPosition4;
                vec3 Centroid = (A + B + C) / 3.0f;
                vec3 Normal = VecNormalize(VecCross(B - A, C - A));

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                debug_viz_data VizData = {};
                if (TestTriangleBox(A, B, C, Box.Center, Box.Extents, Box.Axes, &VizData))
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }

                DD_DrawTriangle(DDRenderData, PRIM_STYLE_TRANSPARENT, A, B, C, Color);
                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_TRANSPARENT, Box.Center, Box.Extents, Rotation, Color);

                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, Centroid, Centroid + 2.0f * Normal, vec3 { 0.0f, 0.0f, 1.0f });
                if ((CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT]) &&
                    (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT]))
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, C, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, A, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, B, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else
                {
                    DD_VisualizeRotationMat(DDRenderData, VECTOR_STYLE_OVERLAY, Rotation, 2.0f, Box.Center, vec3 { 0.5f, 0.5f, 0.5f });
                }

                DrawDebugVizData(DDRenderData, &VizData);
            } break;
            case TEST_CASE_INTERSECTION_TRIANGLE_BOX_2:
            {
                glDisable(GL_CULL_FACE);

                obb Box;
                Box.Center = vec3 { 2.0f, 2.0f, 0.0f } + ControlledPosition;
                Box.Extents = vec3 { 0.5f, 2.0f, 0.5f };
                mat3 Rotation = Mat3FromCols(vec3 { 1.0f, 0.0f, 0.0f },
                                             vec3 { 0.0f, 1.0f, 0.0f },
                                             vec3 { 0.0f, 0.0f, 1.0f });
                Mat3GetCols(Rotation, Box.Axes);

                vec3 A = vec3 { -1.0f, 0.0f, 0.0f };
                vec3 B = vec3 { 1.0f, 0.0f, 0.0f };
                vec3 C = vec3 { 0.0f, 2.0f, 0.0f };
                vec3 Centroid = (A + B + C) / 3.0f;
                vec3 Normal = VecNormalize(VecCross(B - A, C - A));

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                debug_viz_data VizData = {};
                if (TestTriangleBox(A, B, C, Box.Center, Box.Extents, Box.Axes, &VizData))
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }

                DD_DrawTriangle(DDRenderData, PRIM_STYLE_TRANSPARENT, A, B, C, Color);
                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_TRANSPARENT, Box.Center, Box.Extents, Rotation, Color);

                DrawDebugVizData(DDRenderData, &VizData);
            } break;
            case TEST_CASE_INTERSECTION_TRIANGLE_TRIANGLE:
            {
                glDisable(GL_CULL_FACE);
                vec3 TriAPosition = vec3 { -1.5f, 0.0f, 0.0f } + ControlledPosition;
                mat3 Rotation = Mat3GetRotationAroundAxis(vec3 { 0.0f, 1.0f, 0.0f }, DegreesToRadians(45.0f + ControlledAngle));
                vec3 A = (Rotation * (vec3 { -1.0f, 0.0f, 0.0f })) + TriAPosition;
                vec3 B = (Rotation * (vec3 {  1.0f, 0.0f, 0.0f })) + TriAPosition;
                vec3 C = (Rotation * (vec3 {  0.0f, 1.0f, 0.0f })) + TriAPosition;
                vec3 CentroidA = (A + B + C) / 3.0f;
                vec3 NormalA = VecNormalize(VecCross(B - A, C - A));

                vec3 D = vec3 { 1.0f, 0.0f, 0.0f } + ControlledPosition2;
                vec3 E = vec3 { 2.0f, 0.0f, 0.0f } + ControlledPosition3;
                vec3 F = vec3 { 1.5f, 1.0f, 0.0f } + ControlledPosition4;
                vec3 CentroidD = (D + E + F) / 3.0f;
                vec3 NormalD = VecNormalize(VecCross(E - D, F - D));

                vec3 Color = vec3 { 1.0f, 1.0f, 1.0f };

                debug_viz_data VizData = {};
                if (TestTriangleTriangle(A, B, C, D, E, F, &VizData))
                {
                    Color = vec3 { 0.0f, 1.0f, 0.0f };
                }

                DD_DrawTriangle(DDRenderData, PRIM_STYLE_TRANSPARENT, A, B, C, Color);
                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, CentroidA, CentroidA + 2.0f * NormalA, vec3 { 0.0f, 0.0f, 1.0f });

                DD_DrawTriangle(DDRenderData, PRIM_STYLE_TRANSPARENT, D, E, F, Color);
                DD_DrawVector(DDRenderData, VECTOR_STYLE_OVERLAY, CentroidD, CentroidD + 2.0f * NormalD, vec3 { 0.0f, 0.0f, 1.0f });

                if ((CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT]) &&
                    (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT]))
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, F, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else if (CurrentKeyStates[SDL_SCANCODE_LSHIFT] || CurrentKeyStates[SDL_SCANCODE_RSHIFT])
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, D, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else if (CurrentKeyStates[SDL_SCANCODE_LALT] || CurrentKeyStates[SDL_SCANCODE_RALT])
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, E, vec3 { 1.0f, 0.0f, 1.0f });
                }
                else
                {
                    DD_DrawDot(DDRenderData, VECTOR_STYLE_OVERLAY, CentroidA, vec3 { 1.0f, 0.0f, 1.0f });
                }

                DrawDebugVizData(DDRenderData, &VizData);
            } break;
            case TEST_CASE_MOUSE_PICKING:
            {
                glDisable(GL_CULL_FACE);
                Camera.IsLookAround = true;
                
                vec3 CameraPosition = Camera.Position;
                vec3 CameraLookDirection = GetCameraLookDirection(&Camera);

                sphere Sphere;
                Sphere.Center = vec3 { -3.0f, 1.0f, -3.0f };
                Sphere.Radius = 1.5f;

                aabb AABB;
                AABB.Center = vec3 { -4.0f, 0.5f, 4.0f };
                AABB.Extents = vec3 { 2.0f, 0.5f, 0.8f };

                obb OBB;
                OBB.Center = vec3 { -0.5f, 0.5f, 1.0f };
                OBB.Extents = vec3 { 0.7f, 2.0f, 1.5f };
                mat3 OBBRotation = Mat3GetRotationAroundAxis(VecNormalize(vec3 { 1.0f, 1.0f, 0.0f }), DegreesToRadians(33.0f));
                Mat3GetCols(OBBRotation, OBB.Axes);

                vec3 TriA = vec3 { 2.0f, -1.0f, -1.0f };
                vec3 TriB = vec3 { 4.0f, -0.7f, -2.0f };
                vec3 TriC = vec3 { 3.0f,  5.0f, 1.5f };

                capsule Capsule;
                Capsule.Start = vec3 { 4.0f, 0.0f, 3.0f };
                Capsule.End = vec3 { 4.0f, 2.0f, 2.5f };
                Capsule.Radius = 0.9f;

                vec3 SphereColor = vec3 { 1.0f, 1.0f, 1.0f };
                vec3 AABBColor = vec3 { 1.0f, 1.0f, 1.0f };
                vec3 OBBColor = vec3 { 1.0f, 1.0f, 1.0f };
                vec3 TriColor = vec3 { 1.0f, 1.0f, 1.0f };
                vec3 CapsuleColor = vec3 { 1.0f, 1.0f, 1.0f };

                if (IntersectRaySphere(CameraPosition, CameraLookDirection, Sphere, NULL, NULL))
                {
                    SphereColor = vec3 { 0.0f, 1.0f, 0.0f };
                }

                if (IntersectRayAABB(CameraPosition, CameraLookDirection, AABB, NULL, NULL))
                {
                    AABBColor = vec3 { 0.0f, 1.0f, 0.0f };
                }

                /*
                if (IntersectRayOBB(CameraPosition, CameraLookDirection, OBB, NULL, NULL))
                {
                    OBBColor = vec3 { 0.0f, 1.0f, 0.0f };
                }
                */

                if (IntersectRayTriangle(CameraPosition, CameraLookDirection, TriA, TriB, TriC, NULL, NULL, NULL, NULL))
                {
                    TriColor = vec3 { 0.0f, 1.0f, 0.0f };
                }

                /*
                if (IntersectRayCapsule(CameraPosition, CameraLookDirection, Capsule, NULL, NULL))
                {
                    CapsuleColor = vec3 { 0.0f, 1.0f, 0.0f };
                }
                */

                DD_DrawSphere(DDRenderData, PRIM_STYLE_TRANSPARENT, Sphere.Center, Sphere.Radius, SphereColor, 29, 30);
                DD_DrawAABox(DDRenderData, PRIM_STYLE_TRANSPARENT, AABB.Center, AABB.Extents, AABBColor);
                DD_DrawOrientedBox(DDRenderData, PRIM_STYLE_TRANSPARENT, OBB.Center, OBB.Extents, OBBRotation, OBBColor);
                DD_DrawTriangle(DDRenderData, PRIM_STYLE_TRANSPARENT, TriA, TriB, TriC, TriColor);
                DD_DrawCapsule(DDRenderData, PRIM_STYLE_TRANSPARENT, Capsule.Start, Capsule.End, Capsule.Radius, CapsuleColor, 14, 30);
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
ProcessPointSetUpdate(const u8 *CurrentKeyStates, u8 *KeyWasDown, vec3 *PointSet, u32 *PointsUsed, u32 PointBufferCount, bool *PointSetChanged)
{
    Assert(PointBufferCount > 3);
    if (PointSetChanged) *PointSetChanged = false;

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
        if (PointSetChanged) *PointSetChanged = true;
    }

    if (CurrentKeyStates[SDL_SCANCODE_SPACE] && !KeyWasDown[SDL_SCANCODE_SPACE])
    {
        if (CurrentKeyStates[SDL_SCANCODE_LSHIFT])
        {
            PointSet[0] = {};
            *PointsUsed = 1;
            if (PointSetChanged) *PointSetChanged = true;
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
                if (PointSetChanged) *PointSetChanged = true;
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
