#ifndef NEWO_MEMORY_H
#define NEWO_MEMORY_H

#include "NewoCommon.h"

struct memory_arena
{
    size_t Size;
    u8 *Base;
    size_t Used;

    size_t PrevUsed;
};

void
InitializeMemoryArena(memory_arena *Arena, size_t Size, u8 *Base);

#define PushStruct(Arena, type) (type *) PushStruct_(Arena, sizeof(type))
void *
PushStruct_(memory_arena *Arena, size_t Size);

void
BeginTempMemoryManagement(memory_arena *Arena);

void
EndTempMemoryManagement(memory_arena *Arena);

#endif