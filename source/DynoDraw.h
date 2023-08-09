#ifndef DYNO_DRAW_H
#define DYNO_DRAW_H

#include <cstdlib>

#include "NewoCommon.h"

// struct dd_circle
// {
//     u32 VAO;
//     u32 VBO;
//     u32 IndexCount;
//     bool IsInitialized;
// };

#define MAX_VERTEX_COUNT 1024
struct dd_render_data
{
    f32 Vertices[MAX_VERTEX_COUNT * 3];
    u32 VerticesUsed;
};

//void
//DD_DrawCircle(dd_render_data *RenderData, f32 Radius, vec3 Position)
//{
//    RenderData->Vertices[VerticesUsed++] = ; // for each vertex that a circle will need
//}

#endif
