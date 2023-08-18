#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>
#include <sdl2/SDL.h>

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"
#include "DynoDraw.h"

mat4
GetPerspecitveProjectionMat(f32 FovY_Degrees, f32 AspectRatio, f32 Near, f32 Far);

mat4
GetLookAtMat(vec3 Position, vec3 Front, vec3 Up);

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

    i32 ScreenWidth;
    i32 ScreenHeight;
    SDL_GetWindowSize(Window, &ScreenWidth, &ScreenHeight);
    glViewport(0, 0, ScreenWidth, ScreenHeight);

    size_t ApplicationMemorySize = Megabytes(64);
    void *ApplicationMemory = calloc(1, ApplicationMemorySize);
    //void *ApplicationMemoryCopy = calloc(1, ApplicatioMemorySize);

    memory_arena DDArena{};
    size_t DDArenaSize = Kilobytes(512);
    Assert(DDArenaSize >= sizeof(dd_render_data));
    InitializeMemoryArena(&DDArena, DDArenaSize, (u8 *) ApplicationMemory);
    dd_render_data *DDRenderData = DD_InitializeRenderData(&DDArena);

    mat4 ProjectionMat = GetPerspecitveProjectionMat(90.0f, (f32) ScreenWidth / (f32) ScreenHeight, 0.1f, 1000.0f);

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
                    i32 Width = SdlEvent.window.data1;
                    i32 Height = SdlEvent.window.data2;
                    glViewport(0, 0, Width, Height);
                }
                break;
            }
        }

        const u8 *CurrentKeyStates = SDL_GetKeyboardState(0);
        if (CurrentKeyStates[SDL_SCANCODE_ESCAPE])
        {
            ShouldQuit = true;
        }

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mat4 ViewMat = GetLookAtMat(vec3 { 0.0f, 0.0f, 5.0f }, vec3 { 0.0f, 0.0f, 4.0f }, vec3 { 0.0f, 1.0f, 0.0f });

        DD_DrawSphere(DDRenderData, 1.0f, vec3 { 0.0f, 0.0f, 0.0f }, vec3 { 1.0f, 1.0f, 1.0f }, 7, 8);

        DD_Render(DDRenderData, ProjectionMat, ViewMat);

        SDL_GL_SwapWindow(Window);
    }

    SDL_Quit();
    return 0;
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
GetLookAtMat(vec3 EyePosition, vec3 TargetPoint, vec3 WorldUp)
{
    mat4 Result = Mat4Identity();
    
    vec3 Front = VecNormalize(TargetPoint - EyePosition);
    vec3 Right = VecNormalize(VecCrossProduct(Front, VecNormalize(WorldUp)));
    vec3 Up = VecCrossProduct(Right, Front);

    Result.D[0][0] = Right.X;
    Result.D[0][1] = Up.X;
    Result.D[0][2] = -Front.X;

    Result.D[1][0] = Right.Y;
    Result.D[1][1] = Up.Y;
    Result.D[1][2] = -Front.Y;

    Result.D[2][0] = Right.Z;
    Result.D[2][1] = Up.Z;
    Result.D[2][2] = -Front.Z;

    Result.D[3][0] = -EyePosition.X;
    Result.D[3][1] = -EyePosition.Y;
    Result.D[3][2] = -EyePosition.Z;

    return Result;
}
