@echo off
IF NOT DEFINED vcvarsall_ran (
    call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x64
    SET vcvarsall_ran=yes
)

msbuild newo-dyno.vcxproj
