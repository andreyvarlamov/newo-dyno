#include "NewoMemory.h"

#include "NewoCommon.h"

void
InitializeMemoryArena(memory_arena *Arena, size_t Size, u8 *Base)
{
    Arena->Size = Size;
    Arena->Base = Base;
    Arena->Used = 0;
    Arena->PrevUsed = 0;
}

void *
PushStruct_(memory_arena *Arena, size_t Size)
{
    Assert((Arena->Used + Size) <= Arena->Size);
    void *Result = Arena->Base + Arena->Used;
    Arena->Used += Size;
    return Result;
}

void
BeginTempMemoryManagement(memory_arena *Arena)
{
    if (Arena->PrevUsed == 0)
    {
        Arena->PrevUsed = Arena->Used;
    }
}

void
EndTempMemoryManagement(memory_arena *Arena)
{
    if (Arena->PrevUsed != 0)
    {
        Arena->Used = Arena->PrevUsed;
    }
}
