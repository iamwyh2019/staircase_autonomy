@echo off
REM Convenience wrapper for running Python scripts with the stair detector module
REM Usage: run.bat test_detector.py [args...]

setlocal

REM Get script directory
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

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

REM Check if module exists
if not exist "%SCRIPT_DIR%\stair_detector_py*.pyd" (
    echo Error: Python module not found. Please run build.bat first.
    echo Expected: stair_detector_py*.pyd in %SCRIPT_DIR%
    exit /b 1
)

REM Add vcpkg DLLs to PATH and set PYTHONPATH
set "PATH=C:\vcpkg\installed\x64-windows\bin;%PATH%"
set "PYTHONPATH=%SCRIPT_DIR%;%PYTHONPATH%"

REM Change to script directory
cd /d "%SCRIPT_DIR%"

REM Run Python with all arguments
echo Running: %PYTHON_BIN% %*
%PYTHON_BIN% %*

endlocal