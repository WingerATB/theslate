#!/usr/bin/env bash
# Build the two artifacts a release needs. They are NOT interchangeable, and
# the names say so, because picking the wrong one is the obvious mistake:
#
#   -update.bin   the app alone. This is what the settings page installs.
#   -full.bin     bootloader + partition table + app, flashed at 0x0 over USB.
#                 First flash on a blank board, or recovery.
#
# Feeding -full.bin to the updater would write a bootloader into an app slot.
# The firmware rejects it (the image header check fails), but the names should
# make it unlikely in the first place.
set -euo pipefail
cd "$(dirname "$0")/.."
# Version comes from the VERSION file so the artifact names and the version the
# firmware reports about itself cannot disagree.
VER="v$(cat VERSION)"

# Build from the TRACKED defaults, in a build directory of its own -- never from
# the working sdkconfig.
#
# sdkconfig is gitignored and accumulates whatever menuconfig was last used for.
# Building releases from it shipped v0.8.0 with a first-boot transmit power of
# 0 dBm instead of the flight default of -24 dBm, and a switch channel that did
# not match the documentation, because those were bench settings that had been
# sitting in the working config for weeks. Nobody would have noticed until a
# factory-fresh unit was louder than it should be next to someone's receiver.
#
# Regenerating from sdkconfig.defaults makes the artifact identical to what a
# clean clone builds, which is the only version of "reproducible" that means
# anything to someone who did not write it.
BUILD_DIR="build-release"
SDKCFG="${BUILD_DIR}/sdkconfig.release"
rm -f "$SDKCFG"

idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" build
mkdir -p releases

cp "${BUILD_DIR}/slate.bin" "releases/slate-${VER}-update.bin"
( cd "$BUILD_DIR" && esptool.py --chip esp32c3 merge_bin \
    -o "../releases/slate-${VER}-full.bin" @flash_args >/dev/null )
cp "$SDKCFG" "releases/sdkconfig-${VER}"

ls -l "releases/slate-${VER}"*
