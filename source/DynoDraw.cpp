#include "DynoDraw.h"

#include <glad/glad.h>

#include "NewoCommon.h"
#include "NewoLinearMath.h"
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
DD_DrawSphere(dd_render_data *RenderData, f32 Radius, vec3 Position, vec3 Color, u32 RingCount, u32 SectorCount)
{
    Assert(RingCount > 2);
    Assert(SectorCount > 2);
    u32 ExpectedVertexCount = 2 + (RingCount - 2) * SectorCount;
    u32 ExpectedIndexCount = (RingCount - 2) * SectorCount * 6;
    Assert(RenderData->VerticesUsed + ExpectedVertexCount <= MAX_VERTEX_COUNT);
    Assert(RenderData->IndicesUsed + ExpectedIndexCount <= MAX_INDEX_COUNT);

    vec3 *Positions = &RenderData->Positions[RenderData->VerticesUsed];
    vec3 *Normals = &RenderData->Normals[RenderData->VerticesUsed];
    vec3 *Colors = &RenderData->Colors[RenderData->VerticesUsed];

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

    i32 *Indices = &RenderData->Indices[RenderData->IndicesUsed];

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

                Indices[IndexIndex++] = CurrentRingSector0 + (i32) RenderData->VerticesUsed;
                Indices[IndexIndex++] = NextRingSector0 + (i32) SectorIndex + (i32) RenderData->VerticesUsed;
                Indices[IndexIndex++] = NextRingSector0 + NextSectorIndex + (i32) RenderData->VerticesUsed;
            }
            else
            {
                i32 CurrentRingSector0 = 1 + ((i32) RingIndex - 1) * (i32) SectorCount;
                i32 NextRingSector0 = 1 + (i32) RingIndex * (i32) SectorCount;

                if (RingIndex == (RingCount - 2))
                {
                    // Sectors are triangles, add 3 indices

                    Indices[IndexIndex++] = CurrentRingSector0 + (i32) SectorIndex + (i32) RenderData->VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + (i32) RenderData->VerticesUsed;
                    Indices[IndexIndex++] = CurrentRingSector0 + NextSectorIndex + (i32) RenderData->VerticesUsed;
                }
                else
                {
                    // Sectors are quads, add 6 indices

                    Indices[IndexIndex++] = CurrentRingSector0 + (i32) SectorIndex + (i32) RenderData->VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + (i32) SectorIndex + (i32) RenderData->VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + NextSectorIndex + (i32) RenderData->VerticesUsed;

                    Indices[IndexIndex++] = CurrentRingSector0 + (i32) SectorIndex + (i32) RenderData->VerticesUsed;
                    Indices[IndexIndex++] = NextRingSector0 + NextSectorIndex + (i32) RenderData->VerticesUsed;
                    Indices[IndexIndex++] = CurrentRingSector0 + NextSectorIndex + (i32) RenderData->VerticesUsed;
                }
            }
        }
    }
    Assert(IndexIndex == ExpectedIndexCount);

    RenderData->VerticesUsed += VertexIndex;
    RenderData->IndicesUsed += IndexIndex;
}

void
DD_Render(dd_render_data *RenderData, mat4 Projection, mat4 View)
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

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    UseShader(RenderData->Shader);
    SetUniformMat4F(RenderData->Shader, "Projection", (f32 *) &Projection, false);
    SetUniformMat4F(RenderData->Shader, "View", (f32 *) &View, false);
    glBindVertexArray(RenderData->VAO);
    glDrawElements(GL_TRIANGLES, RenderData->IndicesUsed, GL_UNSIGNED_INT, 0);

    RenderData->VerticesUsed = 0;
    RenderData->IndicesUsed = 0;
}
