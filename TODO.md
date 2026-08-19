# TODO

## Firmware

- **`Mavlink::_rcChannelPulses` is dead code.** It is allocated in
  `Mavlink::init()` and filled with `800`, but never read anywhere;
  `Mavlink::sendRcOverrides()` packs the caller's array (the global
  `rcChannels[]` in `main.cpp`, initialized by `initRcChannels()`).
  Decide between:
  - removing the field and its allocation, or
  - moving ownership of the RC failsafe defaults into `Mavlink` and having
    `sendRcOverrides()` use them, so the defaults live next to the code that
    sends them.

  See `src/Mavlink.h` and `src/Mavlink.cpp`.

## Notes

### Building and flashing from VS Code

Open this repository as the VS Code workspace folder and accept the
recommended **PlatformIO IDE** extension (`.vscode/extensions.json`). Wait
for "PlatformIO: Loading tasks..." in the status bar to finish before using
any of the commands below.

Status bar (bottom left):

| Icon    | Action                       | Shortcut     |
| ------- | ---------------------------- | ------------ |
| ✓       | Build                        | `Ctrl+Alt+B` |
| →       | Upload (builds, then flashes)| `Ctrl+Alt+U` |
| 🔌      | Serial Monitor               | `Ctrl+Alt+S` |
| 🗑       | Clean                        |              |

For a debug cycle use **Upload and Monitor** from the PlatformIO sidebar
(alien icon) → *Project Tasks* → `esp32dev` → *General*. It flashes and
reattaches the console in one step; there is no status bar button for it.
The plain monitor holds the serial port open, so an upload started while it
is running fails with a busy-port error — close it with `Ctrl+C` first.

The monitor is preconfigured in `platformio.ini` at 115200 baud with the
`time` and `esp32_exception_decoder` filters, so panic backtraces come out
as file names and line numbers. Use it rather than an external terminal
when chasing a crash.

### Flashing gotchas

- The port is autodetected. Override with `upload_port` / `monitor_port` in
  `platformio.ini` only if the wrong device is picked; `pio device list`
  with the board attached shows the real name (usually `/dev/ttyUSB0` for
  CP2102/CH340, `/dev/ttyACM0` for native USB). Membership in the `dialout`
  group is required on Fedora.
- If the upload does not start, hold **BOOT**, tap **EN/RST**, release BOOT,
  then upload. Persistent flakiness: drop `upload_speed` to `460800`.
- GPIO0, 2, 12 and 15 are strapping pins and can block entry into the
  bootloader depending on what is wired to them. This already bit us once —
  see 0bdd60b, which moved the MAVLink TX off GPIO12 to GPIO14. If uploads
  start failing right after a wiring change, look there first.
