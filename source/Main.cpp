#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>
#include <sdl2/SDL.h>

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"
#include "DynoCamera.h"
#include "DynoDraw.h"

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
    //glEnable(GL_CULL_FACE);

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
    size_t DDArenaSize = Kilobytes(512);
    Assert(DDArenaSize >= sizeof(dd_render_data));
    InitializeMemoryArena(&DDArena, DDArenaSize, (u8 *) ApplicationMemory);
    dd_render_data *DDRenderData = DD_InitializeRenderData(&DDArena);

    dyno_camera Camera = InitializeCamera(vec3 { 0.0f, 0.0f, 10.0f }, -90.0f, 0.0f);

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

        i32 MouseDeltaX;
        i32 MouseDeltaY;
        u32 MouseButtonState = SDL_GetRelativeMouseState(&MouseDeltaX, &MouseDeltaY);

        f32 CameraSpeed = 5.0f;
        f32 CameraRotationSensitivity = 0.1f;
        f32 CameraTranslationSensitivity = 0.01f;

        vec3 CameraTranslation = {};
        f32 CameraDeltaYaw = 0.0f;
        f32 CameraDeltaPitch = 0.0f;

        bool MouseMoved = (MouseDeltaX != 0 || MouseDeltaY != 0);
        bool LeftButtonPressed = (SDL_BUTTON(1) & MouseButtonState);
        bool MiddleButtonPressed = (SDL_BUTTON(2) & MouseButtonState);
        bool RightButtonPressed = (SDL_BUTTON(3) & MouseButtonState);

        if (RightButtonPressed && MouseMoved)
        {
            CameraDeltaYaw = (f32) MouseDeltaX * CameraRotationSensitivity;
            CameraDeltaPitch = (f32) -MouseDeltaY * CameraRotationSensitivity;
        }

        if (MiddleButtonPressed && MouseMoved)
        {
            if (CurrentKeyStates[SDL_SCANCODE_LSHIFT])
            {
                CameraTranslation.Z = (f32) -MouseDeltaY * CameraTranslationSensitivity;
            }
            else
            {
                CameraTranslation.X = (f32) -MouseDeltaX * CameraTranslationSensitivity;
                CameraTranslation.Y = (f32) MouseDeltaY * CameraTranslationSensitivity;
            }
        }
        
        UpdateCameraPosition(&Camera, CameraDeltaYaw, CameraDeltaPitch, CameraTranslation);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mat4 ViewMat = GetCameraViewMat(&Camera);

        DD_DrawSphere(DDRenderData, 4.0f, vec3 { 0.0f, 0.0f, 0.0f }, vec3 { 1.0f, 1.0f, 1.0f }, 9, 10);
        DD_DrawSphere(DDRenderData, 1.0f, vec3 { 5.0f, 0.0f, 0.0f }, vec3 { 1.0f, 0.0f, 0.0f }, 9, 10);

        DD_Render(DDRenderData, ProjectionMat, ViewMat);

        SDL_GL_SwapWindow(Window);
    }

    SDL_Quit();
    return 0;
}
