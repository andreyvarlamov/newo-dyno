#ifndef NEWO_SHADER_H
#define NEWO_SHADER_H

#include <glad/glad.h>

#include "NewoCommon.h"

u32
BuildShaderProgram(const char *VertexPath, const char *FragmentPath);

internal inline void
UseShader(u32 ShaderID)
{
    glUseProgram(ShaderID);
}

bool
SetUniformInt(u32 ShaderID, const char *UniformName, i32 Value, bool UseProgram);

bool
SetUniformVec3F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseProgram);

bool
SetUniformVec4F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseProgram);

bool
SetUniformMat3F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseProgram);

bool
SetUniformMat4F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseProgram);

#endif