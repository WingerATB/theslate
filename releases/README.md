# Releases

Every release is published under
[Releases](https://github.com/WingerATB/theslate/releases), as two files:

| File | What it is | How it goes on |
|---|---|---|
| `slate-<version>-full.bin` | bootloader, partition table, OTA data and app, merged | flashed at offset `0x0` over USB |
| `slate-<version>-update.bin` | the app alone | installed by the settings page, over Wi-Fi |

They are not interchangeable. Feeding `-full.bin` to the updater writes a
bootloader into an app slot.

Recovering a module that will not boot needs no toolchain and no rebuild:

```bash
esptool.py --chip esp32c3 -p /dev/cu.usbmodemXXXX write_flash 0x0 slate-v1.0.0-beta1-full.bin
```

NVS sits at `0x9000` and is not covered by either image, so a rollback keeps the
bound camera and the saved settings. That is deliberate: a rollback is for
recovering from bad firmware, and it should not also cost the user their setup.

**Why the binaries are not in git.** Sixteen versions of two artifacts came to
56 MB, and git keeps every byte for ever — a cost every clone pays, against no
benefit to anyone who came for the source. `tools/release.sh` still writes them
into this directory locally; they are attached to the tag instead.

**The `sdkconfig` snapshot.** `sdkconfig` is gitignored, and a build reproduced
from `sdkconfig.defaults` alone will not carry the same local options (bench
CLI, transmit power) — so the snapshot is what actually makes a release
reproducible. It is 74 KB per version, which is why only the current release's
is kept here; older ones are attached to their own tags.

The history below is kept because it is the honest record of how the module got
here, and several entries are the only written explanation of why a thing works
the way it does.

| Version | What it is |
|---|---|
| **v1.0.0-beta1** | **First beta — flown.** All shipped features tested in the air and working as intended.  Two cameras verified end to end with the protocol chosen automatically; MSP, ARM, the AUX trigger and the button all run against a real flight controller; firmware updates with rollback proven on hardware. The one remaining gap is hardware rather than behaviour: no Osmo Action camera has been available to test against. |
| v0.15.0 | **The setup gesture is narrated on the OSD.** Flick the switch up and it says `ENTER`, after the hold `READY`, and dropping the switch says `CONFIG` and opens setup. Let go early and nothing happens. Committing on the release means the switch is already off when setup starts. Button mode goes straight to `CONFIG`. Setup mode now says `CONFIG` too, so it reads as one continuous act. |
| v0.14.0 | **The setup switch only opens setup.** Leaving is by arming or by Done & restart — an exit on the same switch would drop you out of the page the moment you put the phone down and started moving switches. Entry is now an edge, because otherwise exiting with the switch still up walked straight back in, forever. |
| v0.13.1 | **The armed guard now covers every way into setup.** It was on the setup switch only — the ten-second button hold and the bench console went straight in with no check. One predicate now answers for all three, and it also refuses while an FC link is up but BOXARM is unresolved, because there `armed` is a default rather than a reading. |
| v0.13.0 | **The setup switch, finished.** Switch or button (same detector as the record control), a hold from instant to 10 s, and button mode flagged as instant in the description. **Arming now leaves setup** — a module in setup has no camera link, so an aircraft that took off in it would record nothing and say nothing about why. |
| v0.12.0 | **Setup from the transmitter.** Bind an AUX channel to the settings page, for a module buried in the frame: hold it two seconds while disarmed, drop it to leave. Ignored while armed. Also fixes setup mode never writing the OSD at all — it starts the MSP task but not the logic task, so the screen froze on whatever was there when you entered. It now says `SETUP`. |
| v0.11.0 | **The button toggles recording, always — and a fourth mode.** The C3 button is a test button now: it toggles whatever the FC is doing. COMBI is renamed **Cut**, and a new **Both** mode adds an independent switch toggle on top of arm-follows-recording. One override arbiter serves both, reviewed against twelve blocking scenarios and pinned by eleven host tests. It may stop a clip in any state but may only start one when the link and camera are healthy. |
| v0.10.0 | **Settings page no longer freezes, and the radio turns itself down.** A touch that became a scroll fired `pointerdown` then `pointercancel`, leaking the counter that gated polling — one scroll and the page stopped updating for the session. Now a deadline, which cannot leak. Adds **Turn it down while flying**: the chosen power while disarmed, −24 dBm for as long as the quad is armed, applied to the live link. |
| v0.9.4 | **The scanner reports whether a camera is connectable.** A busy camera still advertises, so "talking but not listening" and "listening and refusing" looked identical. The settings page now splits its advice on signal strength — at good signal it stops suggesting you move the camera and points at the real cause, a phone holding the connection. |
| v0.9.3 | **Choose file works on a laptop.** The firmware updater's picker used a hidden input and a scripted click, which Safari on macOS refuses on both counts — it opened on a phone and did nothing on a desktop. Now a real `<label for>`, so the browser opens the picker itself. |
| v0.9.2 | **The module says why the camera will not connect.** The settings page now shows the actual failure — out of range, flat camera battery, pairing keys rejected — with the attempt count and whether the camera is even on the air, plus a **Forget pairing keys** button for the one cause that cannot be detected. Previously every cause looked identical and the only remedy anyone could find was to re-pair and hope. |
| v0.9.1 | **The OSD refreshes itself.** Slots were written only on change, which assumed the FC still showed what we last sent — untrue after either side reboots, or after a single lost write, and the module had no way to notice. All four rows are now rewritten once a second and immediately when the MSP link returns. |
| v0.9.0 | **Momentary buttons work.** New `Control type` setting: **Switch** keeps the µs window, **Button** treats any movement of the channel as a press — a momentary button leaves the channel where the press put it, so level logic needed two presses to do anything. The config-mode scanner now reports whether it actually started and restarts itself if it goes deaf, so a camera switched on later turns up. The camera name leaves the OSD with the camera. |
| v0.8.1 | **Releases build from the tracked defaults.** They were built from the local, gitignored `sdkconfig`, so v0.8.0 shipped with a first-boot transmit power of 0 dBm instead of the flight default −24 dBm. A clean clone now builds a byte-identical image apart from the embedded build hashes. Kconfig first-boot defaults corrected to match the README (AUX3, 1750 µs). v0.8.0 artifacts withdrawn. |
| v0.8.0 | **Development tree split out.** The bench console, desk tooling and serial captures move to `dev/`, which is gitignored — a checkout without it builds a release image containing no reference to it, and the seam in `app_main()` is three lines. The two startup paths are unified, so a bench build now runs the same firmware as a flight build with a console added rather than a different program. Removes the raw-notification dump and a `DUML_PROBE` option that had been a link error since the scaffolding strip. |
| v0.7.4 | **First release build.** `CONFIG_CAMLINK_BENCH` off: no serial CLI, so nothing can rebind the module, drop its camera link or clear its bonds over USB. Everything before this was a development build. Verified on hardware — every interactive command ignored, binding and OTA probation intact. |
| v0.7.3 | The CLI's `#CF` line now carries the OSD layout as field ids, so the stored layout can be read back without the settings page — the same reason `tx_power` was added to it. |
| v0.7.2 | Settings page reordered: OSD layout sits above the firmware updater, which is now last. Setup reads top to bottom and the one destructive action is at the end. |
| v0.7.1 | **Save layout works, and a Camera field.** `Save layout` answered "bad id" for every layout: URLSearchParams encodes the separators and esp_http_server does not decode them — the same trap that once broke camera selection, now fixed with a tested decoder rather than dodged. Adds a `Camera` field (`NANO`, `O360`, `A5`…) that stays on screen when the camera goes quiet. |
| v0.7.0 | **Configurable OSD layout.** Drag fields into the four rows on the settings page, with a live budget: Betaflight never clears a row, so each is padded to its worst case and 16 characters is a hard limit. Adds unlabelled `87%` / `1H49` variants so two fields fit on one row. The field catalogue comes from the firmware, so the page cannot drift from the renderer. Existing units keep the layout they had. |
| v0.6.4 | **A switched-off camera reads `NO CAM`, not `CAM ?`.** The OSD was waiting on the BLE supervision timeout (6 s) while the status went stale at 3 s, leaving a three-second window where the module held a link nothing came through — a regression from raising that timeout in v0.6.2. Slot 0 now depends only on whether the camera is talking. Adds `D` (drop the link as if the camera went out of range) and `#OS` (read the OSD slots) to the CLI, so this path is testable without a drone. |
| v0.6.3 | **The disconnect handler no longer blocks the BT event loop.** It reconnected from inside the Bluedroid callback and waited up to 60 s there, for events that same task had to deliver — so every disconnect bought 30 s of deafness, the link state stayed "connected" (OSD showed `CAM ?` instead of `NO CAM`), and a silent reconnect could skip re-establishing the camera session entirely. Also: scanning now retries immediately instead of idling out a 30 s timeout — the 360 connects in 8 s rather than 36 — and the settings page drops a switched-off camera after 5 s instead of 12. |
| v0.6.2 | **Stopping a clip no longer drops the camera.** The Nano stops servicing BLE for over 2 s while finalising a clip; the supervision timeout was 2 s, so a stop could kill the link mid-close (`reason=0x08`) and leave the camera still recording. Now 6 s — OSD blanking is unaffected, it was never governed by this. The settings page camera list also stops reordering itself: first-detected order, bars repainted in place. |
| v0.6.1 | **Record path fixes.** One command per press: the retry period now clears the ~1.65 s a camera takes to finalise a clip, so a stop no longer sends a second command into a camera mid-file-close. The CLI’s record keys drive the real state machine instead of bypassing it. BLE connects using the advertised address type rather than a hard-coded public, and disconnect reasons are named in the log. |
| v0.6.0 | **The AUX switch fix.** The µs window now spans 900–2100, matching Betaflight’s Modes tab and the receiver’s real output — CRSF and SBUS both put a switch at +100% on ~2012 µs, outside the old 2000 ceiling, so SWITCH mode could never fire. Stored windows are migrated once on first boot. The settings page also marks unsaved edits instead of letting a restart discard them. |
| v0.5.0 | **Two cameras.** Osmo Nano (DUML) and Osmo 360 (R SDK), protocol chosen automatically from the camera. Action series recognised by model code, and Action 6 by name. Bring-up scaffolding removed. |
| v0.4.0 | **Renamed to SlateFPV.** Access point is now `SLATE-XXXX`; artifacts are `slate-*`. Settings and bound cameras carry over — the NVS namespace deliberately did not change. First flash after the rename must go over USB: firmware still named `camlink` rejects a `slate` image, correctly. |
| v0.3.2 | Light UI: white ground, near-black controls, segmented switches with the description of the selected option shown beneath it. |
| v0.3.1 | Verified working: firmware updates from the settings page, with rollback proven on hardware. Two artifacts per release -- `-update.bin` for the page, `-full.bin` for USB at `0x0`. |
| v0.2.0 | Verified working: adds the live camera picker. The module no longer chooses a camera on its own — with nothing selected it connects to nothing. Button is down to two gestures (tap = bench record, 10 s = setup). |
| v0.1.0 | Verified working: DUML camera link, MSP/OSD, ARM record mode, Wi-Fi config mode with the settings page. Camera binding is still "adopt the nearest" — the last thing before live camera selection. |
