<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="assets/mark-on-ink.svg">
  <img src="assets/mark-on-paper.svg" alt="" width="60">
</picture>

# The Slate

**Arm the quad. The camera rolls.**

[![Release](https://img.shields.io/badge/release-1.0.0--beta1-0F8A4A?style=flat-square)](../../releases)
[![License](https://img.shields.io/badge/license-PolyForm%20Strict-14171C?style=flat-square)](LICENSE)
[![Hardware](https://img.shields.io/badge/hardware-ESP32--C3-5B636E?style=flat-square)](#what-you-need)
[![Buy me a coffee](https://img.shields.io/badge/buy%20me%20a%20coffee-FFDD00?style=flat-square&logo=buymeacoffee&logoColor=14171C)](https://www.buymeacoffee.com/wingeratb)

<img src="assets/hero.jpg" alt="The Slate mounted inside an FPV drone frame, wired to the flight controller" width="100%">

</div>

Your DJI Osmo starts recording the moment you arm, and stops when you disarm.
The camera's own state — recording, clip time, battery, card remaining — is
drawn onto your Betaflight OSD, so you know it is rolling before you leave the
ground. No more landing after a good pack to find the camera was never on.

Everything is set up from your phone over Wi-Fi. After the first flash you never
need a cable again, including for firmware updates.

---

## What you need

| | |
|---|---|
| **Module** | An ESP32-C3 Supermini. This is the only board officially supported. |
| **Camera** | DJI **Osmo Nano**, **Osmo 360**, or an **Osmo Action** camera. |
| **Flight controller** | Betaflight **2025.12 or newer**, with one spare UART. This is a hard requirement — older firmware cannot draw the OSD text. |
| **A phone** | For setup. Any browser. |

The module draws about 80 mA from the flight controller's 5 V rail.

> **Antenna.** The board's printed antenna has to stick out past the frame.
> Carbon fibre blocks 2.4 GHz completely, and no setting compensates for an
> antenna sealed inside a carbon box.

---

## Wiring

Four wires to a spare UART:

| Module | Flight controller |
|---|---|
| `5V` | `5V` |
| `GND` | `GND` |
| `GPIO4` | UART **RX** |
| `GPIO5` | UART **TX** |

Transmit goes to receive, and the grounds must be common. In Betaflight's Ports
tab, no peripheral is needed on that UART — just leave it free.

> **Do not use GPIO2, GPIO8 or GPIO9.** They decide how the chip boots, and
> anything pulling on them at power-up stops it starting at all.

> **Never plug the module into USB with the flight battery in.** The shared 5 V
> wire ties them together, so USB back-feeds the flight controller and
> everything else on that rail. Two supplies in parallel through one wire is how
> regulators die. Battery out first, every time.

---

## Installing

You only do this once. After that, updates happen over Wi-Fi from your phone.

Grab **`slate-<version>-full.bin`** from [Releases](../../releases) — the
`-full` one, which contains everything the module needs to boot.

### From your browser

Espressif's own flashing tool runs in the browser, so there is nothing to
install:

**<https://espressif.github.io/esptool-js/>**

1. Plug the module into your computer with a USB-C cable, **with the flight
   battery out**.
2. Open the page in **Chrome or Edge** — it uses Web Serial, which Safari and
   Firefox do not support.
3. Click **Connect** and pick the port that appears.
4. Choose the `-full.bin` you downloaded.
5. **Change the Flash Address to `0x0`.** The box is pre-filled with `0x1000`,
   which is the one thing here that will quietly go wrong — at that address the
   module has no bootloader in front of it and will not start.
6. Click **Program**.

That is the whole job. When it finishes, unplug it and wire it to your flight
controller.

**If no port appears:** hold the module's BOOT button, tap RESET, then release
BOOT, and click Connect again. Some boards need this the first time.

### Other ways, for the adventurous

If you would rather work in a terminal, the same file can be written with
Espressif's command-line `esptool` — installed with `pip install esptool`, or
downloaded as a standalone binary from
[its releases page](https://github.com/espressif/esptool/releases) if you would
rather not have Python involved.

Either way you are writing the same `-full.bin` to offset `0x0` on an
`esp32c3`. The browser tool above does exactly this and asks fewer questions, so
reach for the terminal only if you already know you want to.

### The other file in each release

Every release also ships a **`-update.bin`**. That one is the app on its own,
and it is what the settings page installs over Wi-Fi. It is not
interchangeable with `-full.bin`: written at `0x0` it will not boot, because it
has no bootloader in front of it.

Use `-full.bin` for a cable. Use `-update.bin` for updates from your phone.

---

## Setting it up

Everything below is set from a page your phone opens. There is no app, no
account and no cable.

### Getting in

**Hold the module's button for ten seconds.** It reboots, the LED goes dark, and
an open Wi-Fi network called `SLATE-XXXX` appears — the last four characters
come from the board itself, so two modules in the same field are never confused.
Join it and the page opens on its own. If your phone does not offer it, go to
`http://192.168.4.1/`.

Once the module is buried in a frame and you cannot reach the button, use the
**setup switch** instead — see below. Setup is refused while armed, and arming
leaves it.

You do not need the flight controller powered to change settings. You do need it
if you want to watch a channel move while you pick one.

---

## Every setting

### Camera

A live list of what is in range, with signal bars. Pick yours and confirm the
four-digit code the camera shows. That pairing is remembered across reboots, so
this is a once-per-camera job.

The list stays in the order cameras were first seen. It does not re-sort itself
as signal changes — a list that reorders under your finger is worse than one
that is merely unsorted.

If the camera you want is not listed, it is off, asleep, or already connected to
a phone.

### Recording

**What starts the clip.** Four modes:

| Mode | What happens |
|---|---|
| **Arm** | Arm starts recording, disarm stops it. |
| **Switch** | An RC channel controls recording; the arm switch is ignored. |
| **Cut** *(default)* | Arm starts the clip. A press cuts the current take and immediately starts a fresh one — it never leaves you not recording, it just breaks the footage into separate files. |
| **Both** | Arm starts and disarm stops, *and* the switch works independently on top: press mid-flight to stop, press again to start. Arming or disarming takes control back, so you always land on a stopped camera. |

On a module that has never had a switch configured, **Cut** behaves exactly like
**Arm**.

**Control type.** How your channel is read — this matters, and the right answer
depends on your transmitter:

- **Switch** — recording runs while the channel *sits inside* the window below.
  Position-driven, so after a flight-controller reboot or a brownout the module
  can look at the switch and know what you meant. Use this for a normal two- or
  three-position toggle.
- **Button** — every *movement* of the channel counts as one press, and each
  press flips recording. The window is ignored. Use this for a momentary button,
  which springs back and so has no position to test. The trade-off is that after
  a brownout the module cannot recover what the state was.

**Channel.** Any of AUX1–AUX14. It does not need to be mapped to anything in
Betaflight's Modes tab — the module reads the channel directly.

**Records above / …and below.** The microsecond window that counts as "on",
anywhere from 900 to 2100 µs. You do not have to know your numbers: move the
switch and watch the live marker, then drag the two handles around where it
lands. Recording runs while the marker is inside the shaded band.

Leave a little margin either side. Setting the edges exactly on the resting
value works until a trim or a rate change moves it a few microseconds.

### The module's own button

Not on the settings page, but worth knowing. The button has exactly two
gestures:

- **A tap** toggles recording, whatever the flight controller is doing. It is a
  test button — the quickest way to confirm on the bench that the camera is
  paired and listening, without arming anything.
- **A ten second hold** opens setup, as above.

---

### Radio power

**Low, Medium, High, Max.** How loudly the module talks to the camera. Low is
the flight default.

**Turn it down while flying.** With this on, the module uses your chosen setting
while disarmed and drops to Low for as long as the quad is armed. Setting up
wants reach — the camera may be across the room. Flying does not: the module is
centimetres from the camera and millimetres from your receiver, where being loud
buys nothing and costs the link that actually matters.

Turn this on and you can leave the setting on High without carrying it into the
air.

### Setup switch

Binds this settings page to an AUX channel, for a module you cannot physically
reach.

**Channel.** Any of AUX1–AUX14. Pick one you are not using for anything else.

**Control type.** *Switch* holds for a set time; *Button* enters instantly on
any movement of the channel.

**Hold for.** Zero to ten seconds, for switch mode. How long the switch must be
held up before the module is willing to open setup.

The gesture is narrated on your OSD so you are never guessing:

| The OSD says | Meaning |
|---|---|
| `ENTER` | The switch is up and the hold has started. |
| `READY` | The hold is complete. Drop the switch now to enter setup. |
| `CONFIG` | You are in setup. The camera link is down and the Wi-Fi page is up. |

Let go before `READY` and nothing happens. The switch only ever *opens* setup —
you leave with **Done & restart** at the bottom of the page, or by arming.
Nothing else on the aircraft will drop you out of the page while you are using
it.

**Setup is refused while armed, and arming leaves it.** A module in setup has no
camera link at all, so an aircraft that took off in that state would record
nothing and say nothing about why.

### Flight controller

Not a setting — a readout. It shows whether the MSP link is up and whether the
module currently sees you as armed. Use it to confirm your wiring and your UART
before you go looking for other problems.

### OSD layout

Drag the fields you want into four rows. Each row is one of Betaflight's four
custom messages and holds **16 characters**; the bar shows how much of that a
row has spent.

| Field | Looks like | Width |
|---|---|---:|
| **State** | `NO CAM` | 7 |
| **Clip time** | `12:34` | 7 |
| **Battery** | `BAT 87%` | 8 |
| **Battery %** | `87%` | 4 |
| **Card left** | `SD 1H49` | 8 |
| **Card** | `1H49` | 5 |
| **Alive dot** | `.` | 1 |
| **Camera** | `O360` | 4 |

**State** is the field to give a row to if you only pick one:

| It says | Meaning |
|---|---|
| `REC` | The camera is recording. |
| `IDLE` | The camera is connected and not recording. |
| `NO CAM` | No camera is talking to the module. |
| `CAM HOT` | The camera is reporting a temperature warning. |
| `ENTER` / `READY` / `CONFIG` | The setup switch gesture, as above. |

Widths are worst cases, not what is on screen right now: a battery reading books
room for `100%` even while it says `87%`, and the clip timer books room for a
long flight. That is why the bar can look fuller than the text appears.

The **alive dot** blinks once a second while the module is talking to the
camera. If it stops, the module has stopped, and that is worth knowing at a
glance.

The **camera** field shows which camera is bound — `NANO`, `O360`, `A5` — and
disappears with the camera if it goes away.

### Firmware

Choose a `-update.bin` from the Releases page and install it. See
[Updating](#updating).

### Saving

**Done & restart** at the bottom writes everything and reboots the module back
into flying mode. Nothing is applied until you do.

---

## Updating

Open the settings page, choose the `-update.bin` from
[Releases](../../releases), and install. The module keeps the previous firmware
and rolls back to it by itself if the new one fails to run, so a bad update does
not leave you with a dead module inside a frame.

Your camera pairing and settings survive updates.

---

## Built on DJI's own protocols

The Osmo Action series and the Osmo 360 speak DJI's **R SDK** protocol, and this
project's implementation of it derives from DJI's own published
[Osmo-GPS-Controller-Demo](https://github.com/dji-sdk/Osmo-GPS-Controller-Demo).
The Osmo Nano speaks a different protocol, and the module works out which one
your camera uses when you pick it — you never have to know.

Attribution and the terms that came with DJI's code are in
[NOTICE](NOTICE). That component stays under its original MIT licence.

---

## Licence

**[PolyForm Strict 1.0.0](LICENSE).** The source is here to be read and run:
inspect it, build it, flash it on your own module, for any noncommercial
purpose. Hobby projects, personal builds, study and experiment are all expressly
permitted.

What the licence does **not** grant is the right to modify it, to redistribute
it, or to use it commercially — including selling hardware built on it.

That is deliberate rather than unfriendly. Publishing the source means you can
check what runs on your aircraft instead of taking my word for it, which is the
point. It does not mean handing someone a head start on selling the same thing.

If you want to do something the licence does not cover — a modification you
need, a commercial use, anything — ask. The answer is often yes.

One part is different: `components/dji_link/` derives from DJI's own published
demo and stays under its original MIT licence. See [NOTICE](NOTICE).

---

## Status

**1.0.0-beta1 — flown, and working as intended.**

Verified in the air on an Osmo Nano and an Osmo 360: pairing, live camera
status, every record mode, the OSD, the AUX trigger as both a switch and a
momentary button, and firmware updates with rollback proven on hardware.

The one gap is hardware rather than behaviour — there is no Osmo Action camera
here to test against. It speaks the same protocol as the Osmo 360, which has
flown.

Found a problem? [Open an issue](../../issues).

---

## Contributing

**I'm not taking pull requests.** The licence does not grant the right to make
derivative works, so a PR sits outside what it permits — and accepting one would
leave the author holding copyright on code I could not then license under the
same terms. Sorting that out means a contributor agreement, which is more
process than this project wants.

That is a licensing decision and nothing to do with the quality of anyone's
work. Please don't spend a weekend on a patch here; I would rather say so now
than after.

**Bug reports and questions are very welcome.** If something is broken, or the
OSD says one thing while your camera does another, [open an
issue](../../issues) — a good bug report is worth considerably more to this
project than a patch I cannot merge.
