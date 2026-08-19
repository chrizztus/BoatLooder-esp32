# BoatLooder-esp32
## License

This project is licensed under a Custom Non-Commercial License. You may use, modify, and distribute this software for non-commercial purposes only. Commercial use is prohibited without explicit permission from the copyright holder.

See the [LICENSE](LICENSE) file for more details.

## Building (PlatformIO / VS Code)

The firmware is a plain PlatformIO project — no Arduino IDE required.

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Open the `boatlooder-esp32/` folder (not the repo root) in VS Code.
3. PlatformIO downloads the toolchain and libraries on first build.

From the command line:

```sh
cd boatlooder-esp32
pio run                  # build
pio run -t upload        # flash
pio device monitor       # serial console @ 115200
```

### Layout

| Path                    | Contents                                        |
| ----------------------- | ----------------------------------------------- |
| `src/`                  | firmware sources (`main.cpp` — was the `.ino`)  |
| `include/mavlink/`      | generated MAVLink v2.0 headers                  |
| `platformio.ini`        | board, framework and library pins               |

The board target is `esp32dev` on the [pioarduino](https://github.com/pioarduino/platform-espressif32)
platform, which provides Arduino-ESP32 core 3.x (needed for `ledcAttachChannel()`).
