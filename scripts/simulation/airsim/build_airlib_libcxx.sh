#!/usr/bin/env bash
# Build AirSim's static libraries (AirLib, MavLinkCom, rpc) with the Unreal
# Engine bundled clang + libc++ so they link into the Unreal plugin.
#
# AirSim's own build.sh uses whatever clang it finds (or GCC); libraries built
# against libstdc++ compile fine and fail only at the final plugin link.
#
# usage: build_airlib_libcxx.sh <ue-root> <airsim-source> [<environment-project-dir>...]
#   Libraries are copied into <airsim-source>/AirLib, the source plugin, and
#   every environment project's Plugins/AirSim given on the command line.
set -Eeuo pipefail

UE_ROOT="$(cd -- "$1" && pwd)"; shift
AIRSIM="$(cd -- "$1" && pwd)"; shift

TOOLCHAIN="$UE_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v19_clang-11.0.1-centos7/x86_64-unknown-linux-gnu"
LIBCXX="$UE_ROOT/Engine/Source/ThirdParty/Linux/LibCxx"
[ -x "$TOOLCHAIN/bin/clang++" ] || { echo "UE clang toolchain not found: $TOOLCHAIN" >&2; exit 1; }
[ -d "$LIBCXX/include/c++/v1" ] || { echo "UE libc++ headers not found: $LIBCXX" >&2; exit 1; }

# AirSim's CommonSetup.cmake calls find_package(LLVM) only to locate libc++;
# point it at the engine's copy instead of a system LLVM.
FAKE_LLVM="$(mktemp -d)"
printf 'set(LLVM_LIBRARY_DIRS "%s")\nset(LLVM_FOUND TRUE)\n' \
  "$LIBCXX/lib/Linux/x86_64-unknown-linux-gnu" > "$FAKE_LLVM/LLVMConfig.cmake"

BUILD="$AIRSIM/build_libcxx"
rm -rf "$BUILD"; mkdir -p "$BUILD"
cmake -S "$AIRSIM/cmake" -B "$BUILD" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER="$TOOLCHAIN/bin/clang" \
  -DCMAKE_CXX_COMPILER="$TOOLCHAIN/bin/clang++" \
  -DCMAKE_AR="$TOOLCHAIN/bin/llvm-ar" \
  -DCMAKE_SYSROOT="$TOOLCHAIN" \
  -DCMAKE_CXX_FLAGS="-nostdinc++ -I$LIBCXX/include/c++/v1 -fPIC" \
  -DLLVM_DIR="$FAKE_LLVM"
ninja -C "$BUILD" rpc MavLinkCom AirLib
rm -rf "$FAKE_LLVM"

OUT="$BUILD/output/lib"
for lib in libAirLib.a libMavLinkCom.a librpc.a; do
  if grep -q 'St7__cxx11' "$OUT/$lib"; then
    echo "$OUT/$lib still carries libstdc++ symbols" >&2; exit 1
  fi
done

install_into() {
  local root="$1"
  mkdir -p "$root/lib/x64/Release" "$root/deps/MavLinkCom/lib" "$root/deps/rpclib/lib"
  cp "$OUT/libAirLib.a" "$root/lib/"
  cp "$OUT/libMavLinkCom.a" "$root/deps/MavLinkCom/lib/"
  cp "$OUT/librpc.a" "$root/deps/rpclib/lib/"
  cp "$OUT"/*.a "$root/lib/x64/Release/"
  echo "installed libc++ AirLib into $root"
}
install_into "$AIRSIM/AirLib"
install_into "$AIRSIM/Unreal/Plugins/AirSim/Source/AirLib"
for project in "$@"; do
  install_into "$(cd -- "$project" && pwd)/Plugins/AirSim/Source/AirLib"
done
