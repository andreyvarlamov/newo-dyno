#include "DynoDraw.h"

#include <glad/glad.h>

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoMemory.h"
#include "NewoShader.h"

#define VERTEX_SHADER_PATH "resource/shader/DynoDraw.vs"
#define FRAGMENT_SHADER_PATH "resource/shader/DynoDraw.fs"

dd_render_data *
DD_InitializeRenderData(memory_arena *MemoryArena)
{
    Assert(MemoryArena->Used == 0);
    Assert(MemoryArena->Size >= sizeof(dd_render_data));

    dd_render_data *RenderData = PushStruct(MemoryArena, dd_render_data);
    RenderData->MemoryArena = MemoryArena;

    u32 VAO;
    glGenVertexArrays(1, &VAO);
    u32 VBO;
    glGenBuffers(1, &VBO);
    u32 EBO;
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, VERTEX_ATTRIBUTES * sizeof(vec3) * MAX_VERTEX_COUNT, 0, GL_DYNAMIC_DRAW);
    for (u32 AttributeIndex = 0; AttributeIndex < VERTEX_ATTRIBUTES; ++AttributeIndex)
    {
        size_t StrideBytes = sizeof(vec3);
        size_t OffsetBytes = AttributeIndex * sizeof(vec3) * MAX_VERTEX_COUNT;
        glVertexAttribPointer(AttributeIndex, 3, GL_FLOAT, GL_FALSE, (GLsizei) StrideBytes, (void *) OffsetBytes);
        glEnableVertexAttribArray(AttributeIndex);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_INDEX_COUNT * sizeof(i32), 0, GL_DYNAMIC_DRAW);

    RenderData->VAO = VAO;
    RenderData->VBO = VBO;
    RenderData->EBO = EBO;
    Assert(RenderData->VAO != 0);
    RenderData->Shader = BuildShaderProgram(VERTEX_SHADER_PATH, FRAGMENT_SHADER_PATH);
    Assert(RenderData->Shader != 0);

    return RenderData;
}

void
DD_DrawSphere(dd_render_data *RenderData, f32 Radius, vec3 Position, vec3 Color, i32 Rings, i32 Sectors)
{
    //vec3 *Positions = RenderData->Positions;
    //vec3 *Colors = RenderData->Positions;
    //vec3 *Normals = RenderData->Normals;

    Assert(RenderData->VerticesUsed + Rings * Sectors <= MAX_VERTEX_COUNT);
    Assert(RenderData->IndicesUsed + Rings * Sectors * 6 <= MAX_INDEX_COUNT);

}

void
DD_Render(dd_render_data *RenderData)
{
    glBindBuffer(GL_ARRAY_BUFFER, RenderData->VBO);
    size_t AttribUsedBytes = sizeof(vec3) * RenderData->VerticesUsed;
    size_t AttribMaxBytes = sizeof(vec3) * MAX_VERTEX_COUNT;
    glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &RenderData->Positions);
    glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &RenderData->Colors);
    glBufferSubData(GL_ARRAY_BUFFER, 2 * AttribMaxBytes, AttribUsedBytes, &RenderData->Normals);
    size_t IndicesUsedBytes = sizeof(i32) * RenderData->IndicesUsed;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, RenderData->EBO);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, IndicesUsedBytes, &RenderData->Indices);

    UseShader(RenderData->Shader);
    glBindVertexArray(RenderData->VAO);
    glDrawElements(GL_TRIANGLES, RenderData->IndicesUsed, GL_UNSIGNED_INT, 0);

    RenderData->VerticesUsed = 0;
    RenderData->IndicesUsed = 0;
}
