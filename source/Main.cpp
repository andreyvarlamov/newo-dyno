#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>
#include <sdl2/SDL.h>

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"
#include "DynoDraw.h"

mat4
GetProjectionMat4(f32 AspectRatio, f32 HalfFOV, f32 Near, f32 Far);

mat4
GetLookAtMat4(vec3 Position, vec3 Front, vec3 Up);

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
    // glEnable(GL_CULL_FACE);

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

    mat4 ProjectionMat = GetProjectionMat4((f32) ScreenWidth / (f32) ScreenHeight, 45.0f, 0.001f, 1000.0f);

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

        mat4 ViewMat = GetLookAtMat4(vec3 { 0.0f, 0.0f, 5.0f }, vec3 { 0.0f, 0.0f, 4.0f }, vec3 { 0.0f, 1.0f, 0.0f });

        DD_DrawSphere(DDRenderData, 1.0f, vec3 { 0.0f, 0.0f, 0.0f }, vec3 { 1.0f, 1.0f, 1.0f }, 5, 4);

        DD_Render(DDRenderData, ProjectionMat, ViewMat);

        SDL_GL_SwapWindow(Window);
    }

    SDL_Quit();
    return 0;
}

mat4
GetProjectionMat4(f32 AspectRatio, f32 HalfFOV, f32 Near, f32 Far)
{
    mat4 ProjectionMat { };

    //ProjectionMat

    return ProjectionMat;
}

mat4
GetLookAtMat4(vec3 Position, vec3 Front, vec3 Up)
{
    Front = VecNormalize(Front - Position);
    Up = VecNormalize(Up);
    vec3 Right = VecNormalize(CrossProduct(Front, Up));

    mat4 LookAtMat { };

    LookAtMat.D[0][0] = Right.X;
    LookAtMat.D[0][1] = Right.Y;
    LookAtMat.D[0][2] = Right.Z;
    LookAtMat.D[1][0] = Up.X;
    LookAtMat.D[1][1] = Up.Y;
    LookAtMat.D[1][2] = Up.Z;
    LookAtMat.D[2][0] = Front.X;
    LookAtMat.D[2][1] = Front.Y;
    LookAtMat.D[2][2] = Front.Z;

    LookAtMat.D[3][0] = Position.X;
    LookAtMat.D[3][1] = Position.Y;
    LookAtMat.D[3][2] = Position.Z;

    return LookAtMat;
}
