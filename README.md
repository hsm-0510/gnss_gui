# gnss_gui
This Python Tkinter app controls and monitors a PocketSDR GNSS receiver, showing real-time satellite, position, and kinematic data with C/N0, CEP, and RMS plots. Supports multi-constellation tracking, COM port NMEA transmission, logging, and adjustable output rates.

Here's your README rewritten in a GitHub-friendly format:

---

# SDR\_GNSS\_UI\_v9

## Overview

**SDR\_GNSS\_UI\_v9** is a Python Tkinter application for controlling and monitoring a PocketSDR-based GNSS receiver.
It starts/stops the `pocket_trk` process, parses NMEA data, and visualizes real-time GNSS, positioning, and kinematic metrics.
Supports multi-constellation tracking (GPS, Galileo, BeiDou, GLONASS), COM port NMEA transmission, and logging.

---

## Features

* **Process Control**

  * Start/stop the `pocket_trk` GNSS receiver.
  * Auto TCP port allocation and socket handling.
  * Minimize or close background CMD windows.

* **Real-Time Data**

  * Satellite table with CH, PRN, signal type, C/N0, lock time, DOP, NAV, and FEC.
  * Live position updates (lat, lon, alt, fix status).
  * Velocity, acceleration, and jerk calculations.
  * CEP (Circular Error Probability) and RMS velocity metrics.

* **Visualization**

  * C/N0 per satellite (color-coded by constellation).
  * Velocity/acceleration/jerk over time.
  * CEP and RMS velocity over time.

* **COM Port Transmission**

  * Send adjusted NMEA sentences over selected COM port.
  * Configurable output rate (0.5 Hz, 1 Hz, 10 Hz) and baud rate.
  * Optional SNR adjustment in GSV sentences.

* **Logging**

  * Save NMEA output to file with timestamps and checksum recalculation.

---

## Requirements

* Python 3.x
* Dependencies:

  ```bash
  pip install matplotlib numpy pillow geopy pyserial pygetwindow
  ```
* PocketSDR binaries (`pocket_trk`) and `libsdr.so` in correct paths.
* Windows environment (uses `devcon`, `taskkill`, `elevat`).

---

## Installation

1. Place `SDR_GNSS_UI_v9.py` in the same directory as `pocket_trk` or update `self.base_path`.
2. Update the `libsdr.so` path in the code.
3. Ensure `pocket_trk_balanced.conf` is available.

---

## Usage

```bash
python SDR_GNSS_UI_v9.py
```

1. Click **Start** to launch PocketSDR.
2. Monitor satellite table, position data, and plots.
3. Select COM port and click **Start Transmission** to send NMEA.
4. Click **Save Log** to store data.
5. Click **Stop** to end the session.

---

## Notes

* Uses random TCP ports in the range 49152–50000 for NMEA streaming.
* CEP/RMS plots reset via the **Reset CEP** button.
* Can be adapted for different GNSS configurations.

## Reqd Fix
* GUI process needs to be terminated via task manager
* TCP port handling issues, can't access data on tcp port when repeating through UI (requires hardware disconnect of PocketSDR)
