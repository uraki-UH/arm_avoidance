#!/bin/bash
echo "x86_64"
nm -D ./ais_gng/libgng/lib_x86_64/libgng_cpu.so

echo "aarch64"
nm -D ./ais_gng/libgng/lib_aarch64/libgng_cpu.so

# echo "aarch64"
# nm -D ./ais_gng/libgng/lib_aarch64/libgng_cpu.so