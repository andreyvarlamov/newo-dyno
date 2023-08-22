#ifndef DYNO_UI_H
#define DYNO_UI_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

struct glyph_info
{
    vec2 GlyphUVs[4];

    i32 MinX;
    i32 MaxX;
    i32 MinY;
    i32 MaxY;
    i32 Advance;
};

struct font_info
{
    glyph_info Glyphs[128];

    u32 AtlasTexture;
    u32 Points;
    u32 Height;
};

font_info
DUI_LoadFontFromFile(const char *FilePath, u32 PointsSize);

void
DUI_DrawString(font_info *Font, f32 NDC_X, f32 NDC_Y);

#endif
