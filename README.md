# NinaDisplay
Real-time hardware monitor for N.I.N.A. astrophotography sessions on an ESP32-S3 AMOLED touchscreen — live guiding, HFR and session stats over WiFi.

A standalone, battery-powered hardware monitor for N.I.N.A. (Nighttime Imaging 'N' Astronomy) that displays your imaging session in real time on a 2.41" AMOLED touchscreen. Built on the Waveshare ESP32-S3-Touch-AMOLED-2.41, it polls NINA's Advanced API over WiFi and shows guiding performance, focus quality, and session progress at a glance — no laptop screen needed at the mount.
Features

Live guiding stats — RA/Dec/Total RMS error with a real-time scrolling graph, plus a lost-lock alert
Imaging metrics — per-sub HFR and star count, charted across the session
Focus & temperature — camera sensor and ambient temperature on color-coded arcs, with a °C/°F toggle
Session summary — current target, filter, sub count, time-to-meridian-flip countdown, and accumulated HFR and total-error trend charts for the whole night
Multi-screen touch UI — swipe between guiding, imaging, focus, and summary screens (built with SquareLine Studio / LVGL)
Auto-discovery — scans the local network to find the NINA host automatically and remembers it across reboots
WiFi provisioning — captive-portal setup on first boot; no hardcoded credentials
Battery monitoring — onboard LiPo voltage and percentage, shown on every screen
Software brightness control and a settings screen with WiFi reset

Hardware

Waveshare ESP32-S3-Touch-AMOLED-2.41 (SH8601 QSPI display, 600×450)
Single-cell LiPo battery (2000–3000 mAh recommended for a full night)
A 3D-printed stand/enclosure (STLs included)

Software requirements

N.I.N.A. with the Advanced API plugin (v2.2.x), API on port 1888
Arduino IDE / arduino-cli with ESP32 board support
Libraries: LVGL 8.3.x, ArduinoJson v5

Or you can download precompiled bin file and flash it with https://espressif.github.io/esptool-js/ at 0x0000 address, baud rate 115200 

