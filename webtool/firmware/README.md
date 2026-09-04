# Bundled firmware for the browser flasher

The browser tool (`../index.html`) can flash a bundled image with one click.
Put the merged full-flash binary here as `slate-full.bin` (written to
flash offset `0x0`). It is a build artifact and is gitignored.

Generate it from a build:

```
idf.py build
(cd build && esptool.py --chip esp32c3 merge_bin \
    -o ../webtool/firmware/slate-full.bin @flash_args)
```

Or copy a release `SlateFPV-<ver>-full.bin` here and rename it. Without this
file, the tool's "choose a .bin file" option still works.
