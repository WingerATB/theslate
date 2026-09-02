#!/usr/bin/env bash
# Host-side tests. No hardware, no ESP-IDF required.
set -e
cd "$(dirname "$0")"
CFLAGS="-Wall -Wextra -O1"
echo "=== building ==="
cc $CFLAGS -o test_msp_frames test_msp_frames.c
cc $CFLAGS -o test_duml test_duml.c
cc $CFLAGS -I../../components/camlink/include -o test_osd_format \
   test_osd_format.c ../../components/camlink/osd_format.c
cc $CFLAGS -I../../components/camlink/include -o test_osd_layout \
   test_osd_layout.c ../../components/camlink/osd_format.c
cc $CFLAGS -I../../components/camlink/include -o test_switch_window \
   test_switch_window.c
cc $CFLAGS -I../../components/webcfg -o test_url_decode \
   test_url_decode.c ../../components/webcfg/url_decode.c
cc $CFLAGS -I../../components/camlink/include -o test_press test_press.c
cc $CFLAGS -I../../components/camlink/include -o test_override test_override.c
cc $CFLAGS -I../../components/camlink/include -o test_setup_guard test_setup_guard.c
echo
./test_msp_frames
echo
./test_duml
echo
./test_osd_format
echo
./test_osd_layout
echo
./test_switch_window
echo
./test_url_decode
echo
./test_press
echo
./test_override
echo
./test_setup_guard
