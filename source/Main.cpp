#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>
#include <sdl2/SDL.h>

#include "NewoCommon.h"
#include "DynoDraw.h"

struct memory_arena
{
    size_t Size;
    u8 *Base;
    size_t Used;

    size_t PrevUsed;
};

void
InitializeMemoryArena(memory_arena *Arena, size_t Size, u8 *Base)
{
    Arena->Size = Size;
    Arena->Base = Base;
    Arena->Used = 0;
    Arena->PrevUsed = 0;
}

#define PushStruct(Arena, type) (type *) PushStruct_(Arena, sizeof(type))
void *
PushStruct_(memory_arena *Arena, size_t Size)
{
    Assert((Arena->Used + Size) <= Arena->Size);
    void *Result = Arena->Base + Arena->Used;
    Arena->Used += Size;
    return Result;
}

void
BeginTempMemoryManagement(memory_arena *Arena)
{
    if (Arena->PrevUsed == 0)
    {
        Arena->PrevUsed = Arena->Used;
    }
}

void
EndTempMemoryManagement(memory_arena *Arena)
{
    if (Arena->PrevUsed != 0)
    {
        Arena->Used = Arena->PrevUsed;
    }
}

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

    i32 WindowWidth;
    i32 WindowHeight;
    SDL_GetWindowSize(Window, &WindowWidth, &WindowHeight);
    glViewport(0, 0, WindowWidth, WindowHeight);

    size_t ApplicationMemorySize = Megabytes(64);
    void *ApplicationMemory = malloc(ApplicationMemorySize);
    memory_arena TransientArena{};
    InitializeMemoryArena(&TransientArena, ApplicationMemorySize, (u8 *) ApplicationMemory);

    dd_render_data *DDRenderData = PushStruct(&TransientArena, dd_render_data);
    dd_render_data *DDRenderData2 = PushStruct(&TransientArena, dd_render_data);

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

        SDL_GL_SwapWindow(Window);
    }

    SDL_Quit();
    return 0;
}
