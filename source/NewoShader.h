#ifndef NEWO_SHADER_H
#define NEWO_SHADER_H

#include "NewoCommon.h"

u32
BuildShaderProgram(const char *VertexPath, const char *FragmentPath);

void
UseShader(u32 ShaderID);

bool
SetUniformInt(u32 ShaderID, const char *UniformName, i32 Value, bool UseShader);

bool
SetUniformVec3F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseShader);

bool
SetUniformVec4F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseShader);

bool
SetUniformMat3F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseShader);

bool
SetUniformMat4F(u32 ShaderID, const char *UniformName, f32 *Value, bool UseShader);

#endif