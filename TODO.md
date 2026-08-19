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
