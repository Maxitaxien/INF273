#!/bin/bash
# fix_concorde.sh - Fixes concorde build issues on modern Linux/gcc
# Run from the truck_and_drone project root
# remember to chmod +x first

set -e
 
CONCORDE_DIR="third_party/concorde"
 
echo "=== Fixing concorde config.guess ==="
CONFIG_GUESS=$(find /usr/share/automake* -name "config.guess" 2>/dev/null | head -1)
if [ -z "$CONFIG_GUESS" ]; then
    echo "automake not found, installing..."
    sudo apt install -y automake
    CONFIG_GUESS=$(find /usr/share/automake* -name "config.guess" | head -1)
fi
cp -f "$CONFIG_GUESS" "$CONCORDE_DIR/config.guess"
cp -f "$(dirname $CONFIG_GUESS)/config.sub" "$CONCORDE_DIR/config.sub"
chmod +x "$CONCORDE_DIR/config.guess" "$CONCORDE_DIR/config.sub"
echo "  config.guess: $($CONCORDE_DIR/config.guess)"
 
echo "=== Fixing gethostname declaration in machdefs.h ==="
sed -i 's|    gethostname (char \*, int);|    /* gethostname (char *, int); */|' \
    "$CONCORDE_DIR/INCLUDE/machdefs.h"
 
echo "=== Patching CMakeLists.txt to pass CFLAGS and CPPFLAGS to concorde configure ==="
# Only patch if not already patched
if ! grep -q 'CFLAGS=-w' CMakeLists.txt; then
    sed -i 's|COMMAND "${CONCORDE_SOURCE_DIR}/configure"|COMMAND ${CMAKE_COMMAND} -E env "CC=gcc" "CFLAGS=-w -std=gnu89" "CPPFLAGS=-w" "${CONCORDE_SOURCE_DIR}/configure"|' \
        CMakeLists.txt
    echo "  CMakeLists.txt patched"
else
    echo "  CMakeLists.txt already patched, skipping"
fi
 
echo "=== Building ==="
rm -rf build-native
cmake -S . -B build-native -DCMAKE_BUILD_TYPE=Release
cmake --build build-native -j"$(nproc)"
 
echo "=== Done! ==="
 
