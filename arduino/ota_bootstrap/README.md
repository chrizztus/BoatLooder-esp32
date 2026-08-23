# OTA bootstrap sketch

One-time setup for a fresh ESP32 that can't build the real
`boatlooder-esp32` PlatformIO project (missing toolchain / library
dependencies). Flash this sketch once over USB, and the app installs the
real firmware entirely over Bluetooth from then on — no PlatformIO, no
USB, ever again after this.

## What you need

- Arduino IDE (2.x) with the **ESP32 board support package** installed
  (Tools → Board → Boards Manager → search "esp32", install the one by
  Espressif Systems).
- A USB cable to the board.

No other libraries — this sketch only uses what the ESP32 board package
already bundles (`BLEDevice`, `Update.h`).

## Steps

1. Open `ota_bootstrap.ino` in Arduino IDE (open the `.ino` file directly;
   Arduino IDE will create/use the surrounding folder as the sketch
   folder — that's expected, the folder name already matches the file).
2. **Tools → Board**: pick **"ESP32 Dev Module"** — the generic board
   entry, right for most dev boards. Do **not** pick a specific fixed
   entry like "DOIT ESP32 DEVKIT V1": those don't expose a Partition
   Scheme submenu at all (Tools → Partition Scheme just won't be there),
   even though the board is the same chip physically. If Tools →
   Partition Scheme is missing after selecting your board, that's the
   fix — switch to "ESP32 Dev Module".
3. **Tools → Partition Scheme**: select **"Minimal SPIFFS (1.9MB APP with
   OTA/190KB SPIFFS)"** (newer core versions label it "...128KB SPIFFS"
   instead — same scheme, just a version-dependent label; pick whichever
   one is named "Minimal SPIFFS ... with OTA").

   This is the one step that actually matters. It has to match
   `boatlooder-esp32/platformio.ini`'s `board_build.partitions =
   min_spiffs.csv` exactly, byte-for-byte — that's the same named scheme,
   just picked from a menu instead of a config file. Get this wrong (pick
   a different scheme, or skip it and leave whatever default was
   selected) and OTA updates will fail to find room to write into, or
   worse, may not boot correctly afterward.

   Everything else in Tools (CPU Frequency, Flash Mode, Flash Frequency,
   Flash Size, PSRAM, Upload Speed, JTAG Adapter, Zigbee Mode, Core
   Debug Level, etc.) is fine left at whatever it defaults to once you've
   picked "ESP32 Dev Module" — none of it matters for OTA.
4. Connect the board via USB, select the right **Tools → Port**.
5. Click **Upload**.
6. Open the Serial Monitor at 115200 baud — you should see:
   ```
   OTA bootstrap ready -- advertising as BoatLuder@chr!zz+us
   ```

## After that

Open the boatlooder app, scan, connect to the vessel — it'll show up the
same way it always does. Since this board is only running the bootstrap
sketch (no real firmware yet), the app will recognize that and offer to
install it. That first install, and every one after it, happens entirely
over Bluetooth — this USB step never needs repeating unless something
goes wrong and the board needs USB recovery.
