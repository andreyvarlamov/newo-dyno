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
PrepareGPUBuffers(u32 VertexCount, size_t *AttribStrides, u32 *AttribComponentCounts, u32 AttribCount,
                  u32 IndexCount, size_t IndexSize,
                  u32 *Out_VAO, u32 *Out_VBO, u32 *Out_EBO);

internal dd_prims_render_data *
GetRenderDataForPrimStyle(dd_render_data *RenderData, prim_style Style);

dd_render_data *
DD_InitializeRenderData(memory_arena *MemoryArena)
{
    Assert(MemoryArena->Used == 0);
    Assert(MemoryArena->Size >= sizeof(dd_render_data));

    dd_render_data *RenderData = PushStruct(MemoryArena, dd_render_data);
    RenderData->MemoryArena = MemoryArena;

    size_t Prim_AttribS[] = { sizeof(vec3), sizeof(vec3), sizeof(vec3) }; u32 Prim_AttribCC[] = { 3, 3, 3 };
    PrepareGPUBuffers(MAX_VERTEX_COUNT, Prim_AttribS, Prim_AttribCC, ArrayCount(Prim_AttribS), MAX_INDEX_COUNT, sizeof(i32),
                      &RenderData->Prims.VAO, &RenderData->Prims.VBO, &RenderData->Prims.EBO);
    RenderData->Prims.Shader = BuildShaderProgram(SHADERP_PRIMITIVES_VERT, SHADERP_PRIMITIVES_FRAG);

    PrepareGPUBuffers(MAX_VERTEX_COUNT, Prim_AttribS, Prim_AttribCC, ArrayCount(Prim_AttribS), MAX_INDEX_COUNT, sizeof(i32),
                      &RenderData->WirePrims.VAO, &RenderData->WirePrims.VBO, &RenderData->WirePrims.EBO);
    RenderData->WirePrims.Shader = RenderData->Prims.Shader;

    PrepareGPUBuffers(MAX_VERTEX_COUNT, Prim_AttribS, Prim_AttribCC, ArrayCount(Prim_AttribS), MAX_INDEX_COUNT, sizeof(i32),
                      &RenderData->TranspPrims.VAO, &RenderData->TranspPrims.VBO, &RenderData->TranspPrims.EBO);
    RenderData->TranspPrims.Shader = RenderData->Prims.Shader;

    size_t Dots_AttribS[] = { sizeof(vec3), sizeof(vec3) }; u32 Dots_AttribCC[] = { 3, 3 };
    PrepareGPUBuffers(MAX_DOT_COUNT, Dots_AttribS, Dots_AttribCC, ArrayCount(Dots_AttribS), 0, 0,
                      &RenderData->Dots.VAO, &RenderData->Dots.VBO, NULL);
    RenderData->Dots.Shader = BuildShaderProgram(SHADERP_BASIC_VERT, SHADERP_BASIC_FRAG);

    PrepareGPUBuffers(MAX_DOT_COUNT, Dots_AttribS, Dots_AttribCC, ArrayCount(Dots_AttribS), 0, 0,
                      &RenderData->OverlayDots.VAO, &RenderData->OverlayDots.VBO, NULL);
    RenderData->OverlayDots.Shader = RenderData->Dots.Shader;

    size_t Vec_AttribS[] = { sizeof(vec3), sizeof(vec3) }; u32 Vec_AttribCC[] = { 3, 3 };
    PrepareGPUBuffers(MAX_VECTOR_COUNT, Vec_AttribS, Vec_AttribCC, ArrayCount(Vec_AttribS), 0, 0,
                      &RenderData->Vectors.VAO, &RenderData->Vectors.VBO, NULL);
    RenderData->Vectors.Shader = RenderData->Dots.Shader;

    size_t OVec_AttribS[] = { sizeof(vec3), sizeof(vec3) }; u32 OVec_AttribCC[] = { 3, 3 };
    PrepareGPUBuffers(MAX_VECTOR_COUNT, OVec_AttribS, OVec_AttribCC, ArrayCount(OVec_AttribS), 0, 0,
                      &RenderData->OverlayVectors.VAO, &RenderData->OverlayVectors.VBO, NULL);
    RenderData->OverlayVectors.Shader = RenderData->Dots.Shader;


    return RenderData;
}

void
DD_DrawSphere(dd_render_data *RenderData, prim_style Style,
              f32 Radius, vec3 Position, vec3 Color, u32 RingCount, u32 SectorCount)
{
    Assert(RingCount > 2);
    Assert(SectorCount > 2);

    dd_prims_render_data *Prims = GetRenderDataForPrimStyle(RenderData, Style);

    u32 VerticesUsed = Prims->VerticesUsed;
    u32 IndicesUsed = Prims->IndicesUsed;
    u32 ExpectedVertexCount = 2 + (RingCount - 2) * SectorCount;
    u32 ExpectedIndexCount = (RingCount - 2) * SectorCount * 6;
    Assert(VerticesUsed + ExpectedVertexCount <= MAX_VERTEX_COUNT);
    Assert(IndicesUsed + ExpectedIndexCount <= MAX_INDEX_COUNT);

    vec3 *Positions = &Prims->Positions[VerticesUsed];
    vec3 *Normals = &Prims->Normals[VerticesUsed];
    vec3 *Colors = &Prims->Colors[VerticesUsed];

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

    i32 *Indices = &Prims->Indices[IndicesUsed];

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

    Prims->VerticesUsed += VertexIndex;
    Prims->IndicesUsed += IndexIndex;
}

void
DD_DrawAABox(dd_render_data *RenderData, prim_style Style, vec3 Position, vec3 Extents, vec3 Color)
{
    dd_prims_render_data *Prims = GetRenderDataForPrimStyle(RenderData, Style);

    u32 VerticesUsed = Prims->VerticesUsed;
    u32 IndicesUsed = Prims->IndicesUsed;
    Assert(VerticesUsed + 8 <= MAX_VERTEX_COUNT);
    Assert(IndicesUsed + 36 <= MAX_INDEX_COUNT);

    vec3 *Positions = &Prims->Positions[VerticesUsed];
    vec3 *Normals = &Prims->Normals[VerticesUsed];
    vec3 *Colors = &Prims->Colors[VerticesUsed];

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

    i32 *Indices = &Prims->Indices[IndicesUsed];

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

    Prims->VerticesUsed += 8;
    Prims->IndicesUsed += 36;
}

void
DD_DrawOrientedBox(dd_render_data *RenderData, prim_style Style, vec3 Position, vec3 Extents, mat3 Orientation, vec3 Color)
{
    dd_prims_render_data *Prims = GetRenderDataForPrimStyle(RenderData, Style);

    u32 VerticesUsed = Prims->VerticesUsed;
    u32 IndicesUsed = Prims->IndicesUsed;
    Assert(VerticesUsed + 8 <= MAX_VERTEX_COUNT);
    Assert(IndicesUsed + 36 <= MAX_INDEX_COUNT);

    vec3 *Positions = &Prims->Positions[VerticesUsed];
    vec3 *Normals = &Prims->Normals[VerticesUsed];
    vec3 *Colors = &Prims->Colors[VerticesUsed];

    Positions[0] = Position + Orientation * vec3 { -Extents.X,  Extents.Y,  Extents.Z };
    Positions[1] = Position + Orientation * vec3 { -Extents.X, -Extents.Y,  Extents.Z };
    Positions[2] = Position + Orientation * vec3 {  Extents.X, -Extents.Y,  Extents.Z };
    Positions[3] = Position + Orientation * vec3 {  Extents.X,  Extents.Y,  Extents.Z };
    Positions[4] = Position + Orientation * vec3 {  Extents.X,  Extents.Y, -Extents.Z };
    Positions[5] = Position + Orientation * vec3 {  Extents.X, -Extents.Y, -Extents.Z };
    Positions[6] = Position + Orientation * vec3 { -Extents.X, -Extents.Y, -Extents.Z };
    Positions[7] = Position + Orientation * vec3 { -Extents.X,  Extents.Y, -Extents.Z };

    Normals[0] = VecNormalize(Orientation * vec3 { -1.0f,  1.0f,  1.0f });
    Normals[1] = VecNormalize(Orientation * vec3 { -1.0f, -1.0f,  1.0f });
    Normals[2] = VecNormalize(Orientation * vec3 {  1.0f, -1.0f,  1.0f });
    Normals[3] = VecNormalize(Orientation * vec3 {  1.0f,  1.0f,  1.0f });
    Normals[4] = VecNormalize(Orientation * vec3 {  1.0f,  1.0f, -1.0f });
    Normals[5] = VecNormalize(Orientation * vec3 {  1.0f, -1.0f, -1.0f });
    Normals[6] = VecNormalize(Orientation * vec3 { -1.0f, -1.0f, -1.0f });
    Normals[7] = VecNormalize(Orientation * vec3 { -1.0f,  1.0f, -1.0f });

    for (i32 I = 0; I < 8; ++I)
    {
        Colors[I] = Color;
    }

    i32 *Indices = &Prims->Indices[IndicesUsed];

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

    Prims->VerticesUsed += 8;
    Prims->IndicesUsed += 36;
}

void
DD_DrawDot(dd_render_data *RenderData, vector_style Style, vec3 Position, vec3 Color)
{
    dd_dots_render_data *Dots;
    switch (Style)
    {
        case VECTOR_STYLE_OVERLAY:
        {
            Dots = &RenderData->OverlayDots;
        } break;
        default:
        {
            Dots = &RenderData->Dots;
        } break;
    }

    Assert(Dots->Used + 1 <= MAX_DOT_COUNT);

    Dots->Positions[Dots->Used] = Position;
    Dots->Colors[Dots->Used] = Color;
    Dots->Used++;
}

void
DD_DrawVector(dd_render_data *RenderData, vector_style Style, vec3 Start, vec3 End, vec3 EndColor)
{
    dd_vectors_render_data *Vectors;
    switch (Style)
    {
        case VECTOR_STYLE_OVERLAY:
        {
            Vectors = &RenderData->OverlayVectors;
        } break;
        default:
        {
            Vectors = &RenderData->Vectors;
        } break;
    }

    Assert(Vectors->Used + 2 <= MAX_VECTOR_COUNT);

    Vectors->Positions[Vectors->Used] = Start;
    Vectors->Colors[Vectors->Used] = vec3 { 0.7f, 0.7f, 0.7f };
    Vectors->Used++;
    Vectors->Positions[Vectors->Used] = End;
    Vectors->Colors[Vectors->Used] = EndColor;
    Vectors->Used++;
}

void
DD_VisualizeRotationMat(dd_render_data *RenderData, vector_style Style, mat3 RotationMat, f32 Scale, vec3 Position, vec3 Color)
{
    dd_vectors_render_data *Vectors;
    switch (Style)
    {
        case VECTOR_STYLE_OVERLAY:
        {
            Vectors = &RenderData->OverlayVectors;
        } break;
        default:
        {
            Vectors = &RenderData->Vectors;
        } break;
    }

    Assert(Vectors->Used + 6 <= MAX_VECTOR_COUNT);

    Vectors->Positions[Vectors->Used] = Position;
    Vectors->Colors[Vectors->Used] = vec3 { 1.0f, 0.0f, 0.0f };
    Vectors->Used++;
    Vectors->Positions[Vectors->Used] = Position + Scale * Mat3GetCol(RotationMat, 0);
    Vectors->Colors[Vectors->Used] = Color;
    Vectors->Used++;
    Vectors->Positions[Vectors->Used] = Position;
    Vectors->Colors[Vectors->Used] = vec3 { 0.0f, 1.0f, 0.0f };
    Vectors->Used++;
    Vectors->Positions[Vectors->Used] = Position + Scale * Mat3GetCol(RotationMat, 1);
    Vectors->Colors[Vectors->Used] = Color;
    Vectors->Used++;
    Vectors->Positions[Vectors->Used] = Position;
    Vectors->Colors[Vectors->Used] = vec3 { 0.0f, 0.0f, 1.0f };
    Vectors->Used++;
    Vectors->Positions[Vectors->Used] = Position + Scale * Mat3GetCol(RotationMat, 2);
    Vectors->Colors[Vectors->Used] = Color;
    Vectors->Used++;
}

void
DD_Render(dd_render_data *RenderData, mat4 Projection, mat4 View)
{
    // Primitives
    dd_prims_render_data *Prims = &RenderData->Prims;
    UseShader(Prims->Shader);
    SetUniformMat4F(Prims->Shader, "Projection", (f32 *) &Projection, false);
    SetUniformMat4F(Prims->Shader, "View", (f32 *) &View, false);
    if (Prims->IndicesUsed > 0)
    {
        glBindVertexArray(Prims->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, Prims->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * Prims->VerticesUsed;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_VERTEX_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &Prims->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &Prims->Colors);
        glBufferSubData(GL_ARRAY_BUFFER, 2 * AttribMaxBytes, AttribUsedBytes, &Prims->Normals);
        size_t IndicesUsedBytes = sizeof(i32) * Prims->IndicesUsed;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, Prims->EBO);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, IndicesUsedBytes, &Prims->Indices);

        glDrawElements(GL_TRIANGLES, Prims->IndicesUsed, GL_UNSIGNED_INT, 0);

        Prims->VerticesUsed = 0;
        Prims->IndicesUsed = 0;
    }

    // Wireframe primitives
    dd_prims_render_data *WirePrims = &RenderData->WirePrims;
    if (WirePrims->IndicesUsed > 0)
    {
        glBindVertexArray(WirePrims->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, WirePrims->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * WirePrims->VerticesUsed;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_VERTEX_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &WirePrims->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &WirePrims->Colors);
        glBufferSubData(GL_ARRAY_BUFFER, 2 * AttribMaxBytes, AttribUsedBytes, &WirePrims->Normals);
        size_t IndicesUsedBytes = sizeof(i32) * WirePrims->IndicesUsed;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, WirePrims->EBO);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, IndicesUsedBytes, &WirePrims->Indices);

        glLineWidth(2);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, WirePrims->IndicesUsed, GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        WirePrims->VerticesUsed = 0;
        WirePrims->IndicesUsed = 0;
    }

    // Dots
    dd_dots_render_data *Dots = &RenderData->Dots;
    UseShader(Dots->Shader);
    SetUniformMat4F(Dots->Shader, "Projection", (f32 *) &Projection, false);
    SetUniformMat4F(Dots->Shader, "View", (f32 *) &View, false);
    if (Dots->Used > 0)
    {
        glBindVertexArray(Dots->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, Dots->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * Dots->Used;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_DOT_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &Dots->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &Dots->Colors);

        
        glPointSize(6);
        glDrawArrays(GL_POINTS, 0, Dots->Used);

        Dots->Used = 0;
    }

    // Overlay dots
    dd_dots_render_data *OverlayDots = &RenderData->OverlayDots;
    if (OverlayDots->Used > 0)
    {
        glBindVertexArray(OverlayDots->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, OverlayDots->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * OverlayDots->Used;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_DOT_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &OverlayDots->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &OverlayDots->Colors);

        glPointSize(6);
        glDisable(GL_DEPTH_TEST);
        glDrawArrays(GL_POINTS, 0, OverlayDots->Used);
        glEnable(GL_DEPTH_TEST);
        
        OverlayDots->Used = 0;
    }

    // Vectors
    dd_vectors_render_data *Vectors = &RenderData->Vectors;
    if (Vectors->Used > 0)
    {
        glBindVertexArray(Vectors->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, Vectors->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * Vectors->Used;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_VECTOR_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &Vectors->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &Vectors->Colors);

        glLineWidth(2);
        glDrawArrays(GL_LINES, 0, Vectors->Used);
        
        Vectors->Used = 0;
    }

    // Overlay vectors
    dd_vectors_render_data *OverlayVectors = &RenderData->OverlayVectors;
    if (OverlayVectors->Used > 0)
    {
        glBindVertexArray(OverlayVectors->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, OverlayVectors->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * OverlayVectors->Used;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_VECTOR_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &OverlayVectors->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &OverlayVectors->Colors);

        glLineWidth(2);
        glDisable(GL_DEPTH_TEST);
        glDrawArrays(GL_LINES, 0, OverlayVectors->Used);
        glEnable(GL_DEPTH_TEST);

        OverlayVectors->Used = 0;
    }

    // Transparent primitives
    dd_prims_render_data *TranspPrims = &RenderData->TranspPrims;
    UseShader(TranspPrims->Shader);
    SetUniformMat4F(TranspPrims->Shader, "Projection", (f32 *) &Projection, false);
    SetUniformMat4F(TranspPrims->Shader, "View", (f32 *) &View, false);
    if (TranspPrims->IndicesUsed > 0)
    {
        glBindVertexArray(TranspPrims->VAO);
        glBindBuffer(GL_ARRAY_BUFFER, TranspPrims->VBO);
        size_t AttribUsedBytes = sizeof(vec3) * TranspPrims->VerticesUsed;
        size_t AttribMaxBytes = sizeof(vec3) * MAX_VERTEX_COUNT;
        glBufferSubData(GL_ARRAY_BUFFER, 0 * AttribMaxBytes, AttribUsedBytes, &TranspPrims->Positions);
        glBufferSubData(GL_ARRAY_BUFFER, 1 * AttribMaxBytes, AttribUsedBytes, &TranspPrims->Colors);
        glBufferSubData(GL_ARRAY_BUFFER, 2 * AttribMaxBytes, AttribUsedBytes, &TranspPrims->Normals);
        size_t IndicesUsedBytes = sizeof(i32) * TranspPrims->IndicesUsed;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, TranspPrims->EBO);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, IndicesUsedBytes, &TranspPrims->Indices);

        SetUniformInt(TranspPrims->Shader, "IsTransparent", 1, false);
        glDrawElements(GL_TRIANGLES, TranspPrims->IndicesUsed, GL_UNSIGNED_INT, 0);
        SetUniformInt(TranspPrims->Shader, "IsTransparent", 0, false);

        TranspPrims->VerticesUsed = 0;
        TranspPrims->IndicesUsed = 0;
    }


}

internal void
PrepareGPUBuffers(u32 VertexCount, size_t *AttribStrides, u32 *AttribComponentCounts, u32 AttribCount,
                  u32 IndexCount, size_t IndexSize,
                  u32 *Out_VAO, u32 *Out_VBO, u32 *Out_EBO)
{
    Assert(Out_VAO);
    Assert(AttribStrides);
    Assert(AttribComponentCounts);
    Assert(AttribCount > 0);

    glGenVertexArrays(1, Out_VAO);
    glBindVertexArray(*Out_VAO);
    Assert(*Out_VAO);

    u32 VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    size_t TotalVertexSize = 0;
    for (u32 AttribIndex = 0; AttribIndex < AttribCount; ++AttribIndex)
    {
        TotalVertexSize += AttribStrides[AttribIndex];
    }
    glBufferData(GL_ARRAY_BUFFER, TotalVertexSize * VertexCount, 0, GL_DYNAMIC_DRAW);
    for (u32 AttribIndex = 0; AttribIndex < AttribCount; ++AttribIndex)
    {
        size_t StrideSize = AttribStrides[AttribIndex];
        size_t OffsetSize = 0;
        for (u32 Index = 0; Index < AttribIndex; ++Index)
        {
            OffsetSize += AttribStrides[Index] * VertexCount;
        }
        glVertexAttribPointer(AttribIndex, AttribComponentCounts[AttribIndex], GL_FLOAT, GL_FALSE,
                              (GLsizei) StrideSize, (void *) OffsetSize);
        glEnableVertexAttribArray(AttribIndex);
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

internal dd_prims_render_data *
GetRenderDataForPrimStyle(dd_render_data *RenderData, prim_style Style)
{
    dd_prims_render_data *Result;

    switch (Style)
    {
        case PRIM_STYLE_WIREFRAME:
        {
            Result = &RenderData->WirePrims;
        } break;
        case PRIM_STYLE_TRANSPARENT:
        {
            Result = &RenderData->TranspPrims;
        } break;
        default:
        {
            Result = &RenderData->Prims;
        } break;
    }

    return Result;
}
