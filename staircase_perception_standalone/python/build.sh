#!/bin/bash

# Build script for Python bindings
# This builds the module as x86_64 to match PCL installation

set -e

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
STANDALONE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Building Python bindings for staircase detector ==="
echo "Script dir: $SCRIPT_DIR"
echo "Standalone dir: $STANDALONE_DIR"

# Use Python 3.11 from system
PYTHON_BIN="/Library/Frameworks/Python.framework/Versions/3.11/bin/python3"

if [ ! -f "$PYTHON_BIN" ]; then
    echo "Error: Python 3.11 not found at $PYTHON_BIN"
    exit 1
fi

echo "Using Python: $PYTHON_BIN"
$PYTHON_BIN --version

# Get Python info for x86_64 architecture
echo "Getting Python configuration..."
PYTHON_INCLUDE=$(arch -x86_64 $PYTHON_BIN -c "import sysconfig; print(sysconfig.get_path('include'))")
PYTHON_LIBDIR=$(arch -x86_64 $PYTHON_BIN -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
PYTHON_VERSION=$(arch -x86_64 $PYTHON_BIN -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
PYTHON_SUFFIX=$(arch -x86_64 $PYTHON_BIN -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")
PYTHON_LIB="${PYTHON_LIBDIR}/libpython${PYTHON_VERSION}.dylib"

echo "Python include: $PYTHON_INCLUDE"
echo "Python lib: $PYTHON_LIB"
echo "Python version: $PYTHON_VERSION"
echo "Python suffix: $PYTHON_SUFFIX"

# Create build directory
BUILD_DIR="$SCRIPT_DIR/build_temp"
mkdir -p "$BUILD_DIR"

# Change to build directory
cd "$BUILD_DIR"

echo "Configuring with CMake..."

# Configure with CMake, forcing x86_64 architecture and Python 3.11
arch -x86_64 cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES=x86_64 \
    -DPython3_EXECUTABLE="$PYTHON_BIN" \
    -DPython3_INCLUDE_DIR="$PYTHON_INCLUDE" \
    -DPython3_LIBRARY="$PYTHON_LIB" \
    -DPython3_FIND_STRATEGY=LOCATION \
    "$SCRIPT_DIR"

echo "Building..."
arch -x86_64 cmake --build . --config Release

# Copy the built module to the python directory
echo "Copying module to python directory..."
MODULE_NAME="stair_detector_py${PYTHON_SUFFIX}"
if [ -f "$MODULE_NAME" ]; then
    cp "$MODULE_NAME" "$SCRIPT_DIR/"
    echo "Successfully built: $SCRIPT_DIR/$MODULE_NAME"
else
    echo "Warning: Expected module name $MODULE_NAME not found"
    # Try to find any .so file
    SO_FILE=$(find . -name "*.so" -type f | head -1)
    if [ -n "$SO_FILE" ]; then
        cp "$SO_FILE" "$SCRIPT_DIR/"
        echo "Copied: $SO_FILE to $SCRIPT_DIR/"
    else
        echo "Error: No .so file found in build directory"
        exit 1
    fi
fi

echo "=== Build complete ==="
echo "Module location: $SCRIPT_DIR/$MODULE_NAME"
echo ""
echo "To test the module, run:"
echo "  cd $SCRIPT_DIR"
echo "  arch -x86_64 $PYTHON_BIN -c 'import stair_detector_py; print(stair_detector_py.__doc__)'"
