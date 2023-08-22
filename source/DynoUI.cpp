#include "DynoUI.h"

#include <glad/glad.h>
#include <sdl2/SDL.h>
#include <sdl2/SDL_ttf.h>

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"

font_info
DUI_LoadFontFromFile(const char *FilePath, u32 PointsSize)
{
    font_info Font = {};
    Font.Points = PointsSize;

    TTF_Font *TTFFont = TTF_OpenFont(FilePath, PointsSize);

    Font.Height = (u32) TTF_FontHeight(TTFFont);

    u32 MaxGlyphWidth = 0;

    SDL_Surface *RenderedGlyphs[128];

    SDL_Color FontColor = { 255, 255, 255, 255 };

    u32 BytesPerPixel = 4; 

    //
    // NOTE: Use SDL TTF to render each glyph into an SDL surface, get glyph metrics and get max glyph width
    // to allocate an atlas surface
    //

    // ASCII Table: https://upload.wikimedia.org/wikipedia/commons/1/1b/ASCII-Table-wide.svg
    for (u8 GlyphChar = 32; GlyphChar < 128; ++GlyphChar)
    {
        i32 *MinX = &Font.Glyphs[GlyphChar].MinX;
        i32 *MaxX = &Font.Glyphs[GlyphChar].MaxX;
        i32 *MinY = &Font.Glyphs[GlyphChar].MinY;
        i32 *MaxY = &Font.Glyphs[GlyphChar].MaxY;
        i32 *Advance = &Font.Glyphs[GlyphChar].Advance;
        TTF_GlyphMetrics(TTFFont, GlyphChar, MinX, MaxX, MinY, MaxY, Advance);

        SDL_Surface *RenderedGlyph = TTF_RenderGlyph_Blended(TTFFont, GlyphChar, FontColor);
        if ((u32) RenderedGlyph->w > MaxGlyphWidth)
        {
            MaxGlyphWidth = RenderedGlyph->w;
        }
        Assert((u32) RenderedGlyph->h <= Font.Height);
        Assert(RenderedGlyph->format->BytesPerPixel == BytesPerPixel);

        RenderedGlyphs[GlyphChar] = RenderedGlyph;
    }

    // 12x8 = 96 -> for the 95 visible glyphs
    u32 AtlasColumns = 12;
    u32 AtlasRows = 8;
    u32 AtlasPxWidth = AtlasColumns * MaxGlyphWidth;
    u32 AtlasPxHeight = AtlasRows * Font.Height;
    u32 AtlasPitch = BytesPerPixel * AtlasPxWidth;
    SDL_Surface *AtlasSurface = SDL_CreateRGBSurface(0, AtlasPxWidth, AtlasPxHeight, 32,
                                                     0x00FF0000, 0x0000FF00,
                                                     0x000000FF, 0xFF000000);
    Assert((u32) AtlasSurface->pitch == AtlasPitch);

    //
    // NOTE: Blit each glyph surface to the atlas surface
    //

    u32 CurrentAtlasIndex = 0;
    for (u8 GlyphChar = 32; GlyphChar < 128; ++GlyphChar)
    {
        SDL_Surface *RenderedGlyph = RenderedGlyphs[GlyphChar];
        Assert(RenderedGlyph);
        glyph_info *Glyph = &Font.Glyphs[GlyphChar];

        u32 GlyphWidth = (u32) RenderedGlyph->w;
        u32 GlyphHeight = (u32) RenderedGlyph->h;
        Assert(GlyphWidth <= MaxGlyphWidth);
        Assert(GlyphHeight <= Font.Height);

        u32 CurrentAtlasCol = CurrentAtlasIndex % AtlasColumns;
        u32 CurrentAtlasRow = CurrentAtlasIndex / AtlasColumns;
        u32 AtlasPxX = CurrentAtlasCol * MaxGlyphWidth;
        u32 AtlasPxY = CurrentAtlasRow * Font.Height;
        size_t AtlasByteOffset = (AtlasPxY * AtlasPxWidth + AtlasPxX) * BytesPerPixel;

        u8 *Dest = ((u8 *) AtlasSurface->pixels) + AtlasByteOffset;
        u8 *Source = ((u8 *) RenderedGlyph->pixels);
        for (u32 GlyphPxY = 0; GlyphPxY < GlyphHeight; ++GlyphPxY)
        {
            u32 *DestPixel = (u32 *) Dest;
            u32 *SourcePixel = (u32 *) Source;
            for (u32 GlyphPxX = 0; GlyphPxX < GlyphWidth; ++GlyphPxX)
            {
                *DestPixel++ = *SourcePixel++;
            }
            Dest += AtlasPitch;
            Source += RenderedGlyph->pitch;
        }

        SDL_FreeSurface(RenderedGlyph);
        RenderedGlyphs[GlyphChar] = 0;

        //
        // NOTE: Use the atlas position and width/height to calculate UV coordinates
        //
        u32 GlyphTexWidth = ((Glyph->MinX >= 0) ? (Glyph->MaxX) : (Glyph->MaxX - Glyph->MinX));
        u32 GlyphTexHeight = Font.Height;
        f32 UVLeft = (f32) AtlasPxX / (f32) AtlasPxWidth;
        f32 UVTop = (f32) AtlasPxY / (f32) AtlasPxHeight;
        f32 UVRight = (f32) (AtlasPxX + GlyphTexWidth) / (f32) AtlasPxWidth;
        f32 UVBottom = (f32) (AtlasPxY + GlyphTexHeight) / (f32) AtlasPxHeight;
        Glyph->GlyphUVs[0] = { UVLeft, UVTop };
        Glyph->GlyphUVs[1] = { UVLeft, UVBottom };
        Glyph->GlyphUVs[2] = { UVRight, UVBottom };
        Glyph->GlyphUVs[3] = { UVRight, UVTop };
     
        CurrentAtlasIndex++;
    }

    TTF_CloseFont(TTFFont);

    glGenTextures(1, &Font.AtlasTexture);
    glBindTexture(GL_TEXTURE_2D, Font.AtlasTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                 AtlasPxWidth, AtlasPxHeight,
                 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV,
                 AtlasSurface->pixels);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    SDL_FreeSurface(AtlasSurface);

    return Font;
}

void
DUI_DrawString(font_info *Font, f32 NDC_X, f32 NDC_Y);
