#!/usr/bin/env bash
# Build the release artifacts, one pair per supported chip.
#
# ONE IMAGE PER CHIP IS NOT A CHOICE. An ESP-IDF application carries its chip id
# in the image header; the second-stage bootloader refuses a mismatch, and so
# does our own updater (webcfg.c, image_is_ours()). Underneath that, the
# C3/C5/C6/C61 are RISC-V while the S3 and the original ESP32 are Xtensa, and
# each links a different prebuilt BLE controller. There is no fat binary for
# this platform, so a release is a set.
#
# The user is not asked to know which one is theirs: the browser flasher reads
# the chip id off the board on connect and picks the matching file. The names
# below are what it matches on, so they are structured rather than pretty:
#
#   slate-<version>-<target>-update.bin   the app alone. What the settings page
#                                         installs over Wi-Fi.
#   slate-<version>-<target>-full.bin     bootloader + partition table + app,
#                                         flashed at 0x0 over USB. First flash
#                                         on a blank board, or recovery.
#
# Feeding -full.bin to the updater would write a bootloader into an app slot.
# The firmware rejects it, but the names should make it unlikely first.
set -euo pipefail
cd "$(dirname "$0")/.."

# What a release is built for. Order matters only in that the reference part
# goes first, so a failure there stops the run before five more builds.
#
#   esp32c3   ships and flies on the C3 Supermini -- the reference
#   esp32s3   shares the C3's Bluetooth controller, so the same TX ladder
#   esp32c6   radio floor is -15 dBm, not -24 dBm; see board.c
#   esp32c61  as the C6
#   esp32c5   full ladder; preview silicon
#   esp32     three TX rungs, no USB, needs the IRAM options
#
# Not here, and why: the C2 compiles and runs out of RAM when both radios come
# up, which is worse than a link error because it gets as far as a flashable
# image. The S2 has no Bluetooth, the H2/H21/H4 no Wi-Fi and the P4 no radios.
# See docs/COMPATIBILITY.md.
TARGETS=(esp32c3 esp32s3 esp32c6 esp32c61 esp32c5 esp32)
[ $# -gt 0 ] && TARGETS=("$@")

VER="v$(cat VERSION)"
mkdir -p releases

# Build from the TRACKED defaults, in a build directory of its own -- never from
# the working sdkconfig.
#
# sdkconfig is gitignored and accumulates whatever menuconfig was last used for.
# Building releases from it shipped v0.8.0 with a first-boot transmit power of
# 0 dBm instead of the flight default, and a switch channel that did not match
# the documentation, because those were bench settings that had been sitting in
# the working config for weeks. Regenerating from sdkconfig.defaults makes the
# artifact identical to what a clean clone builds, which is the only version of
# "reproducible" that means anything to someone who did not write it.
#
# Each target gets its own -B directory and its own SDKCONFIG. Sharing either
# would make the second build inherit the first one's target.
for T in "${TARGETS[@]}"; do
    echo "=== $T"
    BUILD_DIR="build-release/$T"
    SDKCFG="${BUILD_DIR}/sdkconfig.release"
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"

    idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" -D IDF_TARGET="$T" build

    # The bench console can rebind the module to any MAC, drop its camera link
    # and clear its BLE bonds. Asked of the artifact rather than of the working
    # config, because the artifact is the thing that ships.
    if grep -q '^CONFIG_CAMLINK_BENCH=y' "$SDKCFG"; then
        echo "$T was built WITH the bench console -- refusing to release it" >&2
        exit 1
    fi

    cp "${BUILD_DIR}/slate.bin" "releases/slate-${VER}-${T}-update.bin"
    ( cd "$BUILD_DIR" && esptool.py --chip "$T" merge_bin \
        -o "../../releases/slate-${VER}-${T}-full.bin" @flash_args >/dev/null )
    cp "$SDKCFG" "releases/sdkconfig-${VER}-${T}"

    if grep -aq "BENCH CONSOLE ACTIVE" "releases/slate-${VER}-${T}-update.bin"; then
        echo "releases/slate-${VER}-${T}-update.bin contains the bench console" >&2
        exit 1
    fi
done

echo
ls -l "releases/slate-${VER}"*
