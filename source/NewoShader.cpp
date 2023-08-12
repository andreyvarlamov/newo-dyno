#include "NewoShader.h"

#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>

#include "NewoCommon.h"

u32
CompileShaderFromPath(const char *Path, u32 ShaderType);

u32
LinkShaders(u32 VertexShader, u32 FragmentShader);

char *
LoadFileText(const char *Path);

u32
BuildShaderProgram(const char *VertexPath, const char *FragmentPath)
{
    u32 VertexShader = CompileShaderFromPath(VertexPath, GL_VERTEX_SHADER);
    u32 FragmentShader = CompileShaderFromPath(VertexPath, GL_FRAGMENT_SHADER);

    u32 ShaderProgram = LinkShaders(VertexShader, FragmentShader);

    glDeleteShader(VertexShader);
    glDeleteShader(FragmentShader);

    return ShaderProgram;
}

u32
CompileShaderFromPath(const char *Path, u32 ShaderType)
{
    printf("Compiling shader at %s: ", Path);
    char *Source = LoadFileText(Path);
    u32 Shader = glCreateShader(ShaderType);
    glShaderSource(Shader, 1, &Source, 0);
    glCompileShader(Shader);
    free(Source);

    i32 Success = 0;
    glGetShaderiv(Shader, GL_COMPILE_STATUS, &Success);
    if (Success)
    {
        printf("Done.\n");
        
        return Shader;
    }
    else
    {
        char LogBuffer[512];
        i32 ReturnedSize;
        glGetShaderInfoLog(Shader, 512, &ReturnedSize, LogBuffer);
        Assert(ReturnedSize < 512);
        Assert(LogBuffer[ReturnedSize] == '\0');
        printf("ERROR:\n%s\n", LogBuffer);

        return 0;
    }
    
}

u32
LinkShaders(u32 VertexShader, u32 FragmentShader)
{
    u32 ShaderProgram = glCreateProgram();
    glAttachShader(ShaderProgram, VertexShader);
    glAttachShader(ShaderProgram, FragmentShader);
    glLinkProgram(ShaderProgram);

    i32 Success = 0;
    glGetProgramiv(ShaderProgram, GL_LINK_STATUS, &Success);
    if (Success)
    {
        return ShaderProgram;
    }
    else
    {
        char LogBuffer[512];
        i32 ReturnedSize;
        glGetProgramInfoLog(ShaderProgram, 512, &ReturnedSize, LogBuffer);
        Assert(ReturnedSize < 512);
        Assert(LogBuffer[ReturnedSize] == '\0');
        printf("ERROR when linking shaders:\n%s\n", LogBuffer);

        return 0;
    }
}

char *
LoadFileText(const char *Path)
{
    FILE *File;
    fopen_s(&File, Path, "rb");
    Assert(File);
    fseek(File, 0, SEEK_END);
    size_t FileSize = ftell(File);
    Assert(FileSize > 0);
    fseek(File, 0, SEEK_SET);

    char *Result = (char *) malloc(FileSize + 1);
    Assert(Result);
    size_t ElementsRead = fread(Result, FileSize, 1, File);
    Assert(ElementsRead == 1);
    Result[FileSize] = '\0';

    fclose(File);

    return Result;
}