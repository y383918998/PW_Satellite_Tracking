# PW_Satellite_Tracking

Vision-Based LEO Satellite Tracking and Calibration System.

This project is a Master's thesis prototype for an AI-assisted satellite
tracking ground station. The system combines rotator control, RF signal
monitoring, and camera-based visual calibration to compensate for mechanical
backlash and logical encoder error.

## Current Scope

The current repository contains the Hardware_Agent prototype:

- `hardware_controller.py`
  - Windows-oriented all-in-one lifecycle script.
  - Starts Hamlib `rotctld.exe`.
  - Connects to Hamlib on TCP `4533`.
  - Optionally reads SDR++/rigctl signal strength on TCP `4532`.
  - Captures Yi camera HTTP snapshots and saves angle-image metadata.

- `hardware_controller_linux.py`
  - Linux/SBC-oriented version for Orange Pi Zero 3 or Raspberry Pi.
  - Starts Linux `rotctld`.
  - Uses `/dev/ttyUSB0` by default for the MD-02 serial interface.
  - Supports command-line overrides for serial device, ports, capture interval,
    camera URLs, and capture directory.

- `AGENTS.md`
  - System architecture and agent role definitions.

## Hardware Architecture

- Rotator: SPID SPX AZ/EL-01 with MD-02 controller.
- RF receiver path: patch antenna, LNA, HyderSDR or SDRplay-based receiver.
- Vision sensor: Xiaoyi Yi Outdoor FullHD 1080P LED camera with
  `roleoroleo/yi-hack-MStar` firmware.
- Camera stream:
  - RTSP pattern: `rtsp://<camera-host>/ch0_0.h264`
  - Snapshot pattern:
    `http://<camera-host>/cgi-bin/snapshot.sh?res=high&watermark=yes`

## Basic Usage

Create a local configuration file first:

```bash
cp .env.example .env
```

Then edit `.env` with the real camera host, WiFi credentials, serial device,
and MQTT settings for your deployment. The scripts load `.env` automatically
and existing shell environment variables still take priority.

Windows:

```powershell
$env:CAMERA_SNAPSHOT_URL="http://<camera-host>/cgi-bin/snapshot.sh?res=high&watermark=yes"
$env:CAMERA_RTSP_URL="rtsp://<camera-host>/ch0_0.h264"
python hardware_controller.py
```

Linux SBC:

```bash
export CAMERA_SNAPSHOT_URL="http://<camera-host>/cgi-bin/snapshot.sh?res=high\&watermark=yes"
export CAMERA_RTSP_URL="rtsp://<camera-host>/ch0_0.h264"
python3 hardware_controller_linux.py --serial-device /dev/ttyUSB0
```

If `rotctld` is already running:

```bash
python3 hardware_controller_linux.py --no-start-rotctld
```

If testing only the rotator without camera capture:

```bash
python3 hardware_controller_linux.py --no-camera
```

## Data Output

Captured calibration samples are stored as paired files:

- `.jpg` image from the Yi camera snapshot endpoint.
- `.json` metadata with timestamp, logical AZ/EL, optional command AZ/EL,
  optional RSSI/SNR, safety status, and camera URLs.

Capture directories are intentionally ignored by git because they may become
large quickly.

## Security Notes

Do not commit WiFi passwords, MQTT passwords, SSH keys, API tokens, or local
site-specific configuration. Use local untracked files such as `.env` or
`config.local.json` for secrets.
