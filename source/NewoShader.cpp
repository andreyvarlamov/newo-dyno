#include "NewoShader.h"

#include <cstdio>
#include <cstdlib>

#include <glad/glad.h>

#include "NewoCommon.h"
#include "NewoLinearMath.h"

internal u32
CompileShaderFromPath(const char *Path, u32 ShaderType);

internal u32
LinkShaders(u32 VertexShader, u32 FragmentShader);

internal char *   
LoadFileText(const char *Path);

u32
BuildShaderProgram(const char *VertexPath, const char *FragmentPath)
{
    u32 VertexShader = CompileShaderFromPath(VertexPath, GL_VERTEX_SHADER);
    u32 FragmentShader = CompileShaderFromPath(FragmentPath, GL_FRAGMENT_SHADER);

    u32 ShaderProgram = LinkShaders(VertexShader, FragmentShader);

    glDeleteShader(VertexShader);
    glDeleteShader(FragmentShader);

    Assert(ShaderProgram != 0);

    return ShaderProgram;
}

internal u32
CompileShaderFromPath(const char *Path, u32 ShaderType)
{
    printf("Compiling shader at %s: ", Path);
    char *Source = LoadFileText(Path);
    u32 Shader = glCreateShader(ShaderType);
    glShaderSource(Shader, 1, &Source, 0);
    glCompileShader(Shader);
    free(Source);
    Source = 0;

    i32 Success = 0;
    glGetShaderiv(Shader, GL_COMPILE_STATUS, &Success);
    if (Success)
    {
        printf("Done.\n");
        
        return Shader;
    }
    else
    {
        char LogBuffer[1024];
        i32 ReturnedSize;
        glGetShaderInfoLog(Shader, 1024, &ReturnedSize, LogBuffer);
        Assert(ReturnedSize < 1024);
        Assert(LogBuffer[ReturnedSize] == '\0');
        printf("ERROR:\n%s\n", LogBuffer);

        return 0;
    }
    
}

internal u32
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
        char LogBuffer[1024];
        i32 ReturnedSize;
        glGetProgramInfoLog(ShaderProgram, 1024, &ReturnedSize, LogBuffer);
        Assert(ReturnedSize < 1024);
        Assert(LogBuffer[ReturnedSize] == '\0');
        printf("ERROR when linking shaders:\n%s\n", LogBuffer);

        return 0;
    }
}

internal char *
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

bool
SetUniformMat4F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseProgram)
{
    if (UseProgram)
    {
        glUseProgram(ShaderID);
    }
    
    i32 UniformLocation = glGetUniformLocation(ShaderID, UniformName);

    if (UniformLocation == -1)
    {
        return false;
    }

    glUniformMatrix4fv(UniformLocation, 1, false, Value);
    return true;
}
