#ifndef DYNO_DRAW_H
#define DYNO_DRAW_H

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"

#define MAX_VERTEX_COUNT 4096
#define MAX_INDEX_COUNT 16384
#define VERTEX_ATTRIBUTES 3

#define MAX_DOT_COUNT 128
#define MAX_VECTOR_COUNT 128

struct dd_primitives_render_data
{
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

struct dd_dots_render_data
{
    vec3 Positions[MAX_DOT_COUNT];
    vec3 Colors[MAX_DOT_COUNT];
    u32 Used;

    u32 VAO;
    u32 VBO;
    u32 Shader;
};

struct dd_vectors_render_data
{
    vec3 Positions[MAX_VECTOR_COUNT];
    vec3 Colors[MAX_VECTOR_COUNT];
    u32 Used;

    u32 VAO;
    u32 VBO;
    u32 Shader;
};

struct dd_render_data
{
    memory_arena *MemoryArena;

    dd_primitives_render_data Primitives;
    dd_dots_render_data Dots;
    dd_dots_render_data OverlayDots;
    dd_vectors_render_data Vectors;
};

dd_render_data *
DD_InitializeRenderData(memory_arena *MemoryArena);

void
DD_DrawSphere(dd_render_data *RenderData, f32 Radius, vec3 Position, vec3 Color, u32 RingCount, u32 SectorCount);

void
DD_DrawAABox(dd_render_data *RenderData, vec3 Position, vec3 Extents, vec3 Color);

void
DD_DrawDot(dd_render_data *RenderData, vec3 Position, vec3 Color);

void
DD_DrawOverlayDot(dd_render_data *RenderData, vec3 Position, vec3 Color);

void
DD_DrawVector(dd_render_data *RenderData, vec3 Start, vec3 End, vec3 EndColor);

void
DD_Render(dd_render_data *RenderData, mat4 Projection, mat4 View);

#endif
