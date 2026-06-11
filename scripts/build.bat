@echo off
setlocal

set ROOT=%~dp0..
set BUILD_DIR=%ROOT%\build

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"

cmake .. -DCMAKE_BUILD_TYPE=Release
if %ERRORLEVEL% neq 0 (
    echo CMake configure FAILED.
    exit /b 1
)

cmake --build . --config Release
if %ERRORLEVEL% neq 0 (
    echo Build FAILED.
    exit /b 1
)

echo.
echo Build succeeded. Binary: %BUILD_DIR%\bin\haldex.exe
