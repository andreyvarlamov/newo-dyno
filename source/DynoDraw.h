#ifndef DYNO_DRAW_H
#define DYNO_DRAW_H

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"

#define MAX_VERTEX_COUNT 4096
#define MAX_INDEX_COUNT 16384
#define VERTEX_ATTRIBUTES 3

struct dd_render_data
{
    memory_arena *MemoryArena;

    vec3 Positions[MAX_VERTEX_COUNT];
    vec3 Normals[MAX_VERTEX_COUNT];
    vec3 Colors[MAX_VERTEX_COUNT];
    i32 Indices[MAX_INDEX_COUNT];
    u32 VerticesUsed;
    u32 IndicesUsed;

    u32 VAO;
    u32 VBO;
    u32 EBO;
    u32 Shader;
};

dd_render_data *
DD_InitializeRenderData(memory_arena *MemoryArena);

void
DD_DrawSphere(dd_render_data *RenderData, f32 Radius, vec3 Position, vec3 Color, u32 RingCount, u32 SectorCount);

void
DD_Render(dd_render_data *RenderData, mat4 Projection, mat4 View);

#endif
