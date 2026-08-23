# Changelog

High-level, from the perspective of someone piloting the boat and updating
its firmware — not a commit log.

## 1.1.0

- **Rename your vessel from the app.** Boats now advertise as
  `<fixed prefix>@<your name>`, so with more than one boat around you can
  tell them apart in the scan list at a glance. A rename takes effect the
  next time the vessel powers on.
- **Switch motor drivers without re-flashing.** Which motor driver board
  is wired up is now a setting you pick in the app instead of something
  baked into the firmware — switch it live, with a warning first since it
  can spin the propellers. Only allowed while disarmed.

## 1.0.0

Everything below shipped in the run-up to this release (last 7 days):

- **Live telemetry and full parameter access over Bluetooth** — no more
  USB needed to see GPS, battery, EKF/navigation health, or to read and
  change vehicle parameters.
- **Wireless firmware updates.** The app can now push a new firmware
  build straight to the boat over Bluetooth, including a way to bootstrap
  a brand-new board over USB just once so all updates after that are
  wireless too.
- **Mission waypoints can now be downloaded from the vessel**, not just
  uploaded, and a bug that could scramble the home waypoint was fixed.
- **Status messages from the flight controller show up in the app** —
  in particular, the reason arming was refused is no longer a mystery.
- Fixed the app sometimes still showing "connected" after the vessel had
  actually dropped the link, and other Bluetooth link reliability fixes
  under heavy data (e.g. large parameter lists).
