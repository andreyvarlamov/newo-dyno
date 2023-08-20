#include "DynoDraw.h"

#include <cstring>

#include <glad/glad.h>

#include "NewoCommon.h"
#include "NewoLinearMath.h"
#include "NewoMath.h"
#include "NewoMemory.h"
#include "NewoShader.h"

#define SHADERP_BASIC_VERT "resource/shader/DynoDraw_Basic.vs"
#define SHADERP_BASIC_FRAG "resource/shader/DynoDraw_Basic.fs"
#define SHADERP_PRIMITIVES_VERT "resource/shader/DynoDraw_Primitives.vs"
#define SHADERP_PRIMITIVES_FRAG "resource/shader/DynoDraw_Primitives.fs"

internal void
PrepareGPUBuffers(u32 VertexCount, size_t *AttributeStrides, u32 *AttributeComponentCounts, u32 AttributeCount,
                  u32 IndexCount, size_t IndexSize,
                  u32 *Out_VAO, u32 *Out_VBO, u32 *Out_EBO);

dd_render_data *
DD_InitializeRenderData(memory_arena *MemoryArena)
{
    Assert(MemoryArena->Used == 0);
    Assert(MemoryArena->Size >= sizeof(dd_render_data));

    dd_render_data *RenderData = PushStruct(MemoryArena, dd_render_data);
    RenderData->MemoryArena = MemoryArena;

    size_t AttributeStrides[] = { sizeof(vec3), sizeof(vec3), sizeof(vec3) };
    u32 AttributeComponentCounts[] = { 3, 3, 3 };
    PrepareGPUBuffers(MAX_VERTEX_COUNT, AttributeStrides, AttributeComponentCounts, ArrayCount(AttributeStrides),
                      MAX_INDEX_COUNT, sizeof(i32),
                      &RenderData->Primitives.VAO, &RenderData->Primitives.VBO, &RenderData->Primitives.EBO);
    RenderData->Primitives.Shader = BuildShaderProgram(SHADERP_PRIMITIVES_VERT, SHADERP_PRIMITIVES_FRAG);

    size_t DotAttributeStrides[] = { sizeof(vec3), sizeof(vec3) };
    u32 DotAttributeComponentCounts[] = { 3, 3 };
    PrepareGPUBuffers(MAX_DOT_COUNT, DotAttributeStrides, DotAttributeComponentCounts, ArrayCount(DotAttributeStrides),
                      0, 0,
                      &RenderData->Dots.VAO, &RenderData->Dots.VBO, NULL);
    RenderData->Dots.Shader = BuildShaderProgram(SHADERP_BASIC_VERT, SHADERP_BASIC_FRAG);

    size_t ODotAttributeStrides[] = { sizeof(vec3), sizeof(vec3) };
    u32 ODotAttributeComponentCounts[] = { 3, 3 };
    PrepareGPUBuffers(MAX_DOT_COUNT, ODotAttributeStrides, ODotAttributeComponentCounts, ArrayCount(ODotAttributeStrides),
                      0, 0,
                      &RenderData->OverlayDots.VAO, &RenderData->OverlayDots.VBO, NULL);
    RenderData->OverlayDots.Shader = RenderData->Dots.Shader;

    size_t VectorAttributeStrides[] = { sizeof(vec3), sizeof(vec3) };
    u32 VectorAttributeComponentCounts[] = { 3, 3 };
    PrepareGPUBuffers(MAX_VECTOR_COUNT, VectorAttributeStrides, VectorAttributeComponentCounts, ArrayCount(VectorAttributeStrides),
                      0, 0,
                      &RenderData->Vectors.VAO, &RenderData->Vectors.VBO, NULL);
    RenderData->Vectors.Shader = RenderData->Dots.Shader;

    return RenderData;
}

void
DD_DrawSphere(dd_render_data *RenderData, f32 Radius, vec3 Position, vec3 Color, u32 RingCount, u32 SectorCount)
{
    Assert(RingCount > 2);
    Assert(SectorCount > 2);
    u32 VerticesUsed = RenderData->Primitives.VerticesUsed;
    u32 IndicesUsed = RenderData->Primitives.IndicesUsed;
    u32 ExpectedVertexCount = 2 + (RingCount - 2) * SectorCount;
    u32 ExpectedIndexCount = (RingCount - 2) * SectorCount * 6;
    Assert(VerticesUsed + ExpectedVertexCount <= MAX_VERTEX_COUNT);
    Assert(IndicesUsed + ExpectedIndexCount <= MAX_INDEX_COUNT);

    vec3 *Positions = &RenderData->Primitives.Positions[VerticesUsed];
    vec3 *Normals = &RenderData->Primitives.Normals[VerticesUsed];
    vec3 *Colors = &RenderData->Primitives.Colors[VerticesUsed];

    u32 VertexIndex = 0;

    f32 SphereRadius = Radius;

    for (u32 RingIndex = 0; RingIndex < RingCount; ++RingIndex)
    {
        if (RingIndex == 0)
        {
            // At RingIndex = 0 -> VerticalAngle = PI/2 -> cos = 0 -> 0 radius ring -> north pole
            Positions[VertexIndex] = vec3 { 0.0f, SphereRadius, 0.0f } + Position;
            Normals[VertexIndex] = vec3 { 0.0f, 1.0f, 0.0f };
            Colors[VertexIndex] = Color;
            VertexIndex++;
        }
        else if (RingIndex == (RingCount - 1))
        {
            // At RingIndex = RingCount - 1 -> VerticalAngle = -PI/2 -> cos = 0; 0 radius ring -> south pole
            Positions[VertexIndex] = vec3 { 0.0f, -SphereRadius, 0.0f } + Position;
            Normals[VertexIndex] = vec3 { 0.0f, -1.0f, 0.0f };
            Colors[VertexIndex] = Color;
            VertexIndex++;
        }
        else
        {
            f32 RingRatio = ((f32) RingIndex / (f32) (RingCount - 1));

            f32 VerticalAngle = (PI32 / 2.0f - (PI32 * RingRatio)); // PI/2 -> -PI/2 Range
            f32 RingRadius = SphereRadius * CosF32(VerticalAngle);
            f32 Y = SphereRadius * SinF32(VerticalAngle); // Radius -> -Radius range

            for (u32 SectorIndex = 0; SectorIndex < SectorCount; ++SectorIndex)
            {
                f32 SectorRatio = (f32) SectorIndex / (f32) SectorCount;
                f32 Theta = 2.0f * PI32 * SectorRatio;
                f32 X = RingRadius * SinF32(Theta);
                f32 Z = RingRadius * CosF32(Theta);

                vec3 VertPosition = vec3 { X, Y, Z };
                Positions[VertexIndex] = VertPosition + Position;  // Add the position of the sphere origin
                Normals[VertexIndex] = VecNormalize(VertPosition);
                Colors[VertexIndex] = Color;
                VertexIndex++;
            }
        }
    }
    Assert(VertexIndex == ExpectedVertexCount);

    i32 *Indices = &RenderData->Primitives.Indices[IndicesUsed];

    u32 IndexIndex = 0;

    for (u32 RingIndex = 0; RingIndex < RingCount - 1; ++RingIndex)
    {
        for (u32 SectorIndex = 0; SectorIndex < SectorCount; ++SectorIndex)
        {
            i32 NextSectorIndex = ((SectorIndex == (SectorCount - 1)) ?
                                   (0) :
                                   ((i32) SectorIndex + 1));

            if (RingIndex == 0)
            {
                // Sectors are triangles, add 3 indices

                i32 CurrentRingSector0 = 0;
                i32 NextRingSector0 = 1;

                Indices[IndexIndex++] = CurrentRingSector0 + (i32) VerticesUsed;
                Indices[IndexIndex++] = NextRingSector0 + (i32) SectorIndex + (i32) VerticesUsed;
                Indices[IndexIndex++] = NextRingSector0 + NextSectorIndex + (i32) VerticesUsed;
            }
            else
            {
                i32 CurrentRingSector0 = 1 + ((i32) RingIndex - 1) * (i32) SectorCount;
                i32 NextRingSector0 = 1 + (i32) RingIndex * (i32) SectorCount;

                if (RingIndex == (RingCount - 2))
                {
                    // Sectors are triangles, add 3 indices

                    Indices[IndexIndex++] = CurrentRingSector0 + (i32) SectorIndex + (i32) VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + (i32) VerticesUsed;
                    Indices[IndexIndex++] = CurrentRingSector0 + NextSectorIndex + (i32) VerticesUsed;
                }
                else
                {
                    // Sectors are quads, add 6 indices

                    Indices[IndexIndex++] = CurrentRingSector0 + (i32) SectorIndex + (i32) VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + (i32) SectorIndex + (i32) VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + NextSectorIndex + (i32) VerticesUsed;

                    Indices[IndexIndex++] = CurrentRingSector0 + (i32) SectorIndex + (i32) VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + NextSectorIndex + (i32) VerticesUsed;
                    Indices[IndexIndex++] = CurrentRingSector0 + NextSectorIndex + (i32) VerticesUsed;
                }
            }
        }
    }
    Assert(IndexIndex == ExpectedIndexCount);

    RenderData->Primitives.VerticesUsed += VertexIndex;
    RenderData->Primitives.IndicesUsed += IndexIndex;
}

void
DD_DrawAABox(dd_render_data *RenderData, vec3 Position, vec3 Extents, vec3 Color)
{
    u32 VerticesUsed = RenderData->Primitives.VerticesUsed;
    u32 IndicesUsed = RenderData->Primitives.IndicesUsed;
    Assert(VerticesUsed + 8 <= MAX_VERTEX_COUNT);
    Assert(IndicesUsed + 36 <= MAX_INDEX_COUNT);

    vec3 *Positions = &RenderData->Primitives.Positions[VerticesUsed];
    vec3 *Normals = &RenderData->Primitives.Normals[VerticesUsed];
    vec3 *Colors = &RenderData->Primitives.Colors[VerticesUsed];

    Positions[0] = Position + vec3 { -Extents.X,  Extents.Y,  Extents.Z };
    Positions[1] = Position + vec3 { -Extents.X, -Extents.Y,  Extents.Z };
    Positions[2] = Position + vec3 {  Extents.X, -Extents.Y,  Extents.Z };
    Positions[3] = Position + vec3 {  Extents.X,  Extents.Y,  Extents.Z };
    Positions[4] = Position + vec3 {  Extents.X,  Extents.Y, -Extents.Z };
    Positions[5] = Position + vec3 {  Extents.X, -Extents.Y, -Extents.Z };
    Positions[6] = Position + vec3 { -Extents.X, -Extents.Y, -Extents.Z };
    Positions[7] = Position + vec3 { -Extents.X,  Extents.Y, -Extents.Z };

    Normals[0] = VecNormalize(vec3 { -1.0f,  1.0f,  1.0f });
    Normals[1] = VecNormalize(vec3 { -1.0f, -1.0f,  1.0f });
    Normals[2] = VecNormalize(vec3 {  1.0f, -1.0f,  1.0f });
    Normals[3] = VecNormalize(vec3 {  1.0f,  1.0f,  1.0f });
    Normals[4] = VecNormalize(vec3 {  1.0f,  1.0f, -1.0f });
    Normals[5] = VecNormalize(vec3 {  1.0f, -1.0f, -1.0f });
    Normals[6] = VecNormalize(vec3 { -1.0f, -1.0f, -1.0f });
    Normals[7] = VecNormalize(vec3 { -1.0f,  1.0f, -1.0f });

    for (i32 I = 0; I < 8; ++I)
    {
        Colors[I] = Color;
    }

    i32 *Indices = &RenderData->Primitives.Indices[IndicesUsed];

    i32 IndicesToCopy[] = {
        0, 1, 3,  3, 1, 2, // front
        4, 5, 7,  7, 5, 6, // back
        7, 0, 4,  4, 0, 3, // top
        1, 6, 2,  2, 6, 5, // bottom
        7, 6, 0,  0, 6, 1, // left
        3, 2, 4,  4, 2, 5  // right
    };

    for (u32 IndexIndex = 0; IndexIndex < ArrayCount(IndicesToCopy); ++IndexIndex)
    {
        Indices[IndexIndex] = IndicesToCopy[IndexIndex] + VerticesUsed;
    }

    RenderData->Primitives.VerticesUsed += 8;
    RenderData->Primitives.IndicesUsed += 36;
}

void
DD_DrawDot(dd_render_data *RenderData, vec3 Position, vec3 Color)
{
    RenderData->Dots.Positions[RenderData->Dots.Used] = Position;
    RenderData->Dots.Colors[RenderData->Dots.Used] = Color;
    RenderData->Dots.Used++;
}

void
DD_DrawOverlayDot(dd_render_data *RenderData, vec3 Position, vec3 Color)
{
    RenderData->OverlayDots.Positions[RenderData->OverlayDots.Used] = Position;
    RenderData->OverlayDots.Colors[RenderData->OverlayDots.Used] = Color;
    RenderData->OverlayDots.Used++;
}

void
DD_DrawVector(dd_render_data *RenderData, vec3 Start, vec3 End, vec3 EndColor)
{
    RenderData->Vectors.Positions[RenderData->Vectors.Used] = Start;
    RenderData->Vectors.Colors[RenderData->Vectors.Used] = vec3 { 0.7f, 0.7f, 0.7f };
    RenderData->Vectors.Used++;
    RenderData->Vectors.Positions[RenderData->Vectors.Used] = End;
    RenderData->Vectors.Colors[RenderData->Vectors.Used] = EndColor;
    RenderData->Vectors.Used++;
}

void
DD_Render(dd_render_data *RenderData, mat4 Projection, mat4 View)
{
    // Primitives
    dd_primitives_render_data *Primitives = &RenderData->Primitives;
    glBindBuffer(GL_ARRAY_BUFFER, Primitives->VBO);
    size_t AttribUsedBytes = sizeof(vec3) * Primitives->VerticesUsed;
    size_t AttribMaxBytes = sizeof(vec3) * MAX_VERTEX_COUNT;
    glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &Primitives->Positions);
    glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &Primitives->Colors);
    glBufferSubData(GL_ARRAY_BUFFER, 2 * AttribMaxBytes, AttribUsedBytes, &Primitives->Normals);
    size_t IndicesUsedBytes = sizeof(i32) * Primitives->IndicesUsed;
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Primitives->EBO);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, IndicesUsedBytes, &Primitives->Indices);

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    UseShader(Primitives->Shader);
    SetUniformMat4F(Primitives->Shader, "Projection", (f32 *) &Projection, false);
    SetUniformMat4F(Primitives->Shader, "View", (f32 *) &View, false);
    glBindVertexArray(Primitives->VAO);
    glDrawElements(GL_TRIANGLES, Primitives->IndicesUsed, GL_UNSIGNED_INT, 0);

    Primitives->VerticesUsed = 0;
    Primitives->IndicesUsed = 0;

    // Dots
    dd_dots_render_data *Dots = &RenderData->Dots;
    glBindBuffer(GL_ARRAY_BUFFER, Dots->VBO);
    AttribUsedBytes = sizeof(vec3) * Dots->Used;
    AttribMaxBytes = sizeof(vec3) * MAX_DOT_COUNT;
    glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &Dots->Positions);
    glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &Dots->Colors);

    UseShader(Dots->Shader);
    SetUniformMat4F(Dots->Shader, "Projection", (f32 *) &Projection, false);
    SetUniformMat4F(Dots->Shader, "View", (f32 *) &View, false);
    glBindVertexArray(Dots->VAO);
    glPointSize(6);
    glDrawArrays(GL_POINTS, 0, Dots->Used);
    Dots->Used = 0;

    // Overlay dots
    dd_dots_render_data *OverlayDots = &RenderData->OverlayDots;
    glBindBuffer(GL_ARRAY_BUFFER, OverlayDots->VBO);
    AttribUsedBytes = sizeof(vec3) * OverlayDots->Used;
    AttribMaxBytes = sizeof(vec3) * MAX_DOT_COUNT;
    glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &OverlayDots->Positions);
    glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &OverlayDots->Colors);

    glBindVertexArray(OverlayDots->VAO);
    glDisable(GL_DEPTH_TEST);
    glDrawArrays(GL_POINTS, 0, OverlayDots->Used);
    glEnable(GL_DEPTH_TEST);
    OverlayDots->Used = 0;

    // Vectors
    dd_vectors_render_data *Vectors = &RenderData->Vectors;
    glBindBuffer(GL_ARRAY_BUFFER, Vectors->VBO);
    AttribUsedBytes = sizeof(vec3) * Vectors->Used;
    AttribMaxBytes = sizeof(vec3) * MAX_VECTOR_COUNT;
    glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &Vectors->Positions);
    glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &Vectors->Colors);

    glBindVertexArray(Vectors->VAO);
    glDisable(GL_DEPTH_TEST);
    glLineWidth(2);
    glDrawArrays(GL_LINES, 0, Vectors->Used);
    glEnable(GL_DEPTH_TEST);
    Vectors->Used = 0;
}

internal void
PrepareGPUBuffers(u32 VertexCount, size_t *AttributeStrides, u32 *AttributeComponentCounts, u32 AttributeCount,
                  u32 IndexCount, size_t IndexSize,
                  u32 *Out_VAO, u32 *Out_VBO, u32 *Out_EBO)
{
    Assert(Out_VAO);
    Assert(AttributeStrides);
    Assert(AttributeComponentCounts);
    Assert(AttributeCount > 0);

    glGenVertexArrays(1, Out_VAO);
    glBindVertexArray(*Out_VAO);
    Assert(*Out_VAO);

    u32 VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    size_t TotalVertexSize = 0;
    for (u32 AttributeIndex = 0; AttributeIndex < AttributeCount; ++AttributeIndex)
    {
        TotalVertexSize += AttributeStrides[AttributeIndex];
    }
    glBufferData(GL_ARRAY_BUFFER, TotalVertexSize * VertexCount, 0, GL_DYNAMIC_DRAW);
    for (u32 AttributeIndex = 0; AttributeIndex < AttributeCount; ++AttributeIndex)
    {
        size_t StrideSize = AttributeStrides[AttributeIndex];
        size_t OffsetSize = 0;
        for (u32 Index = 0; Index < AttributeIndex; ++Index)
        {
            OffsetSize += AttributeStrides[Index] * VertexCount;
        }
        glVertexAttribPointer(AttributeIndex, AttributeComponentCounts[AttributeIndex], GL_FLOAT, GL_FALSE,
                              (GLsizei) StrideSize, (void *) OffsetSize);
        glEnableVertexAttribArray(AttributeIndex);
    }
    Assert(VBO);
    if (Out_VBO) *Out_VBO = VBO;

    if (IndexCount > 0 && IndexSize > 0)
    {
        u32 EBO;
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, IndexCount * IndexSize, 0, GL_DYNAMIC_DRAW);
        Assert(EBO);
        if (Out_EBO) *Out_EBO = EBO;
    }
}
