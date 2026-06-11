#!/bin/sh
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$ROOT/build"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j"$(nproc 2>/dev/null || echo 4)"

echo ""
echo "Build succeeded. Binary: $BUILD_DIR/bin/haldex"
