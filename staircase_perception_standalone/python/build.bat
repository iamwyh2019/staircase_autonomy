@echo off
REM Build script for Python bindings on Windows
REM This builds the module for x64 Windows

setlocal EnableDelayedExpansion

echo === Building Python bindings for staircase detector ===

REM Get script directory
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
for %%i in ("%SCRIPT_DIR%") do set "STANDALONE_DIR=%%~dpi"
set "STANDALONE_DIR=%STANDALONE_DIR:~0,-1%"

echo Script dir: %SCRIPT_DIR%
echo Standalone dir: %STANDALONE_DIR%

REM Find Python executable
set "PYTHON_BIN="
for /f "tokens=*" %%i in ('where python') do (
    set "PYTHON_BIN=%%i"
    goto :found_python
)

:found_python
if "%PYTHON_BIN%"=="" (
    echo Error: Python not found in PATH
    exit /b 1
)

echo Using Python: %PYTHON_BIN%
%PYTHON_BIN% --version

REM Get Python configuration
echo Getting Python configuration...
for /f "tokens=*" %%i in ('%PYTHON_BIN% -c "import sysconfig; print(sysconfig.get_path('include'))"') do set "PYTHON_INCLUDE=%%i"
for /f "tokens=*" %%i in ('%PYTHON_BIN% -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))"') do set "PYTHON_LIBDIR=%%i"
for /f "tokens=*" %%i in ('%PYTHON_BIN% -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"') do set "PYTHON_VERSION=%%i"
for /f "tokens=*" %%i in ('%PYTHON_BIN% -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"') do set "PYTHON_SUFFIX=%%i"

REM For Windows, library is usually in a different location
for /f "tokens=*" %%i in ('%PYTHON_BIN% -c "import sysconfig; import os; libdir = sysconfig.get_config_var('installed_base'); print(os.path.join(libdir, 'libs', f'python{sysconfig.get_config_var(\"py_version_nodot\")}.lib'))"') do set "PYTHON_LIB=%%i"

echo Python include: %PYTHON_INCLUDE%
echo Python lib: %PYTHON_LIB%
echo Python version: %PYTHON_VERSION%
echo Python suffix: %PYTHON_SUFFIX%

REM Create build directory
set "BUILD_DIR=%SCRIPT_DIR%\build_temp"
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM Change to build directory
cd /d "%BUILD_DIR%"

echo Configuring with CMake...

REM Configure with CMake for Windows x64
cmake ^
    -G "Visual Studio 17 2022" ^
    -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DPython3_EXECUTABLE="%PYTHON_BIN%" ^
    -DPython3_INCLUDE_DIR="%PYTHON_INCLUDE%" ^
    -DPython3_LIBRARY="%PYTHON_LIB%" ^
    -DPython3_FIND_STRATEGY=LOCATION ^
    -DCMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake ^
    "%SCRIPT_DIR%"

if %ERRORLEVEL% neq 0 (
    echo CMake configuration failed
    exit /b 1
)

echo Building...
cmake --build . --config Release

if %ERRORLEVEL% neq 0 (
    echo Build failed
    exit /b 1
)

REM Copy the built module to the python directory
echo Copying module to python directory...
set "MODULE_NAME=stair_detector%PYTHON_SUFFIX%"

REM Look for the module in Release subdirectory (typical for MSVC builds)
if exist "Release\%MODULE_NAME%" (
    copy "Release\%MODULE_NAME%" "%SCRIPT_DIR%\"
    echo Successfully built: %SCRIPT_DIR%\%MODULE_NAME%
) else if exist "%MODULE_NAME%" (
    copy "%MODULE_NAME%" "%SCRIPT_DIR%\"
    echo Successfully built: %SCRIPT_DIR%\%MODULE_NAME%
) else (
    echo Warning: Expected module name %MODULE_NAME% not found
    REM Try to find any .pyd file (Windows Python extension)
    for /r %%f in (*.pyd) do (
        copy "%%f" "%SCRIPT_DIR%\"
        echo Copied: %%f to %SCRIPT_DIR%\
        goto :found_module
    )
    echo Error: No .pyd file found in build directory
    exit /b 1
)

:found_module
echo === Build complete ===
echo Module location: %SCRIPT_DIR%\%MODULE_NAME%
echo.
echo To test the module, run:
echo   cd %SCRIPT_DIR%
echo   %PYTHON_BIN% -c "import stair_detector; print(stair_detector.__doc__)"

endlocal