#ifndef DYNO_DRAW_H
#define DYNO_DRAW_H

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMemory.h"

#define MAX_VERTEX_COUNT 4096
#define MAX_INDEX_COUNT 16384
#define VERTEX_ATTRIBUTES 3

#define MAX_DOT_COUNT 128
#define MAX_VECTOR_COUNT 256

struct dd_prims_render_data
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

    dd_prims_render_data Prims;
    dd_prims_render_data WirePrims;
    dd_prims_render_data TranspPrims;
    dd_dots_render_data Dots;
    dd_dots_render_data OverlayDots;
    dd_vectors_render_data Vectors;
    dd_vectors_render_data OverlayVectors;
};

enum prim_style
{
    PRIM_STYLE_FILLED,
    PRIM_STYLE_WIREFRAME,
    PRIM_STYLE_TRANSPARENT
};

enum vector_style
{
    VECTOR_STYLE_DEPTHTEST,
    VECTOR_STYLE_OVERLAY
};

dd_render_data *
DD_InitializeRenderData(memory_arena *MemoryArena);

void
DD_DrawSphere(dd_render_data *RenderData, prim_style Style,
              f32 Radius, vec3 Position, vec3 Color, u32 RingCount, u32 SectorCount);

void
DD_DrawAABox(dd_render_data *RenderData, prim_style Style, vec3 Position, vec3 Extents, vec3 Color);

void
DD_DrawOrientedBox(dd_render_data *RenderData, prim_style Style, vec3 Position, vec3 Extents, mat3 Orientation, vec3 Color);

void
DD_DrawDot(dd_render_data *RenderData, vector_style Style, vec3 Position, vec3 Color);

void
DD_DrawVector(dd_render_data *RenderData, vector_style Style, vec3 Start, vec3 End, vec3 EndColor);

void
DD_VisualizeRotationMat(dd_render_data *RenderData, vector_style Style, mat3 RotationMat, f32 Scale, vec3 Position, vec3 Color);

void
DD_Render(dd_render_data *RenderData, mat4 Projection, mat4 View);

#endif
