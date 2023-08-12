#ifndef DYNO_DRAW_H
#define DYNO_DRAW_H

#include <glad/glad.h>

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoShader.h"

#define MAX_VERTEX_COUNT 4096
#define MAX_INDEX_COUNT 16384

#define VERTEX_SHADER_PATH "resource/shaders/DynoDraw.vs"
#define FRAGMENT_SHADER_PATH "resource/shaders/DynoDraw.fs"

struct dd_render_data
{
    vec3 Vertices[MAX_VERTEX_COUNT];
    vec3 Colors[MAX_VERTEX_COUNT];
    //vec3 Normals[MAX_VERTEX_COUNT];
    i32 Indices[MAX_INDEX_COUNT];
    u32 VerticesUsed;
    i32 IndicesUsed;
    u32 VAO;
    u32 VBO;
    u32 EBO;
    u32 Shader;
};

void
DD_DrawSphere(dd_render_data *RenderData, f32 Radius, vec3 Position, vec3 Color)
{

}

void
DD_Render(dd_render_data *RenderData)
{
    if (RenderData->VAO == 0)
    {
        u32 VAO;
        glGenVertexArrays(1, &VAO);
        u32 VBO;
        glGenBuffers(1, &VBO);
        u32 EBO;
        glGenBuffers(1, &EBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(vec3) * MAX_VERTEX_COUNT, 0, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void *) 0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void *) (sizeof(vec3) * MAX_VERTEX_COUNT));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, MAX_INDEX_COUNT * sizeof(i32), 0, GL_DYNAMIC_DRAW);

        RenderData->VAO = VAO;
        RenderData->VBO = VBO;
        RenderData->EBO = EBO;
        Assert(RenderData->VAO != 0);
    }

    if (RenderData->Shader == 0)
    {
        RenderData->Shader = BuildShaderProgram(VERTEX_SHADER_PATH, FRAGMENT_SHADER_PATH);
        Assert(RenderData->Shader != 0);
    }

    glBindBuffer(GL_ARRAY_BUFFER, RenderData->VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vec3) * MAX_VERTEX_COUNT, &RenderData->Vertices);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(vec3) * MAX_VERTEX_COUNT, sizeof(vec3) * MAX_VERTEX_COUNT, &RenderData->Colors);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, RenderData->EBO);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(i32) * MAX_INDEX_COUNT, &RenderData->Indices);

    // Draw
    UseShader(RenderData->Shader);
    glBindVertexArray(RenderData->VAO);
    glDrawElements(GL_TRIANGLES, RenderData->IndicesUsed, GL_UNSIGNED_INT, 0);
}

#endif
