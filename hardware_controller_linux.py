"""
Linux Hardware_Agent lifecycle script for Orange Pi Zero 3 / Raspberry Pi.

Target deployment:
  - Patch antenna + LNA + HyderSDR + Orange Pi/Raspberry Pi mounted near the
    antenna head.
  - Hamlib rotctld runs locally on the Linux board and talks to the MD-02 over
    a USB serial adapter such as /dev/ttyUSB0.
  - Yi camera snapshots are captured over the LAN and paired with AZ/EL data.

This Linux version intentionally leaves the Windows script untouched.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import subprocess
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple


DEFAULT_ROTCTLD_EXECUTABLE = "rotctld"
DEFAULT_ROTATOR_MODEL = "1"
DEFAULT_SERIAL_DEVICE = "/dev/ttyUSB0"
DEFAULT_SERIAL_SPEED = "115200"
DEFAULT_HAMLIB_PORT = 4533
DEFAULT_RIGCTL_PORT = 4532
DEFAULT_HOST = "127.0.0.1"
DEFAULT_TIMEOUT = 2.0

DEFAULT_CENTER_AZ = 180.0
DEFAULT_CENTER_EL = 167.5
DEFAULT_SAFE_WINDOW_DEGREES = 20.0
DEFAULT_CAPTURE_INTERVAL_SECONDS = 5.0

DEFAULT_CAMERA_SNAPSHOT_URL = os.getenv(
    "CAMERA_SNAPSHOT_URL",
    "http://camera.local/cgi-bin/snapshot.sh?res=high&watermark=yes",
)
DEFAULT_CAMERA_RTSP_URL = os.getenv("CAMERA_RTSP_URL", "rtsp://camera.local/ch0_0.h264")
DEFAULT_CAPTURE_DIR = Path("captures_linux")


class NetworkController:
    """Timed TCP interface for local Hamlib rotctld and optional rigctl server."""

    def __init__(
        self,
        host: str,
        hamlib_port: int,
        rigctl_port: int,
        timeout: float,
    ) -> None:
        self.host = host
        self.hamlib_port = hamlib_port
        self.rigctl_port = rigctl_port
        self.timeout = timeout
        self.hamlib_socket: Optional[socket.socket] = None
        self.rigctl_socket: Optional[socket.socket] = None

    def connect_all(self) -> None:
        self.hamlib_socket = self._connect_socket("Hamlib rotctld", self.hamlib_port)
        self.rigctl_socket = self._connect_socket("Rigctl/SDR", self.rigctl_port)

    def move_antenna(self, azimuth: float, elevation: float) -> bool:
        response = self._send_and_receive(
            "Hamlib rotctld",
            self.hamlib_socket,
            f"P {azimuth:.3f} {elevation:.3f}\n",
        )
        if response is None:
            return False
        if "RPRT" in response and "RPRT 0" not in response:
            print(f"Warning: Hamlib rejected move command: {response!r}")
            return False
        return True

    def read_current_angles(self) -> Optional[Tuple[float, float]]:
        response = self._send_and_receive("Hamlib rotctld", self.hamlib_socket, "p\n")
        if response is None:
            return None

        values = self._extract_floats(response)
        if len(values) < 2:
            print(f"Warning: Could not parse Hamlib angles from: {response!r}")
            return None

        return values[0], values[1]

    def read_signal_strength(self) -> Optional[float]:
        response = self._send_and_receive(
            "Rigctl/SDR",
            self.rigctl_socket,
            "\\get_level STRENGTH\n",
        )
        if response is None:
            return None

        values = self._extract_floats(response)
        if not values:
            print(f"Warning: Could not parse signal strength from: {response!r}")
            return None

        return values[0]

    def close(self) -> None:
        self._close_socket("Hamlib rotctld", self.hamlib_socket)
        self._close_socket("Rigctl/SDR", self.rigctl_socket)
        self.hamlib_socket = None
        self.rigctl_socket = None

    def _connect_socket(self, name: str, port: int) -> Optional[socket.socket]:
        try:
            sock = socket.create_connection((self.host, port), timeout=self.timeout)
            sock.settimeout(self.timeout)
            print(f"{name}: connected to {self.host}:{port}")
            return sock
        except (OSError, socket.timeout) as exc:
            print(f"Warning: Could not connect to {name} at {self.host}:{port}: {exc}")
            return None

    def _send_and_receive(
        self,
        name: str,
        sock: Optional[socket.socket],
        command: str,
    ) -> Optional[str]:
        if sock is None:
            print(f"Warning: {name} socket is not connected.")
            return None

        try:
            sock.sendall(command.encode("ascii"))
            return self._receive_text(sock)
        except (OSError, socket.timeout) as exc:
            print(f"Warning: {name} command failed for {command.strip()!r}: {exc}")
            return None

    def _receive_text(self, sock: socket.socket, max_bytes: int = 4096) -> str:
        chunks: list[bytes] = []
        while True:
            try:
                chunk = sock.recv(max_bytes)
            except socket.timeout:
                break

            if not chunk:
                break

            chunks.append(chunk)
            if len(chunk) < max_bytes:
                break

        return b"".join(chunks).decode("ascii", errors="replace").strip()

    @staticmethod
    def _extract_floats(text: str) -> list[float]:
        matches = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", text)
        return [float(match) for match in matches]

    @staticmethod
    def _close_socket(name: str, sock: Optional[socket.socket]) -> None:
        if sock is None:
            return
        try:
            sock.close()
            print(f"{name}: socket closed")
        except OSError as exc:
            print(f"Warning: Error while closing {name} socket: {exc}")


class CameraController:
    """HTTP snapshot client for the Yi camera running yi-hack-MStar."""

    def __init__(self, snapshot_url: str, timeout: float) -> None:
        self.snapshot_url = snapshot_url
        self.timeout = timeout

    def capture_snapshot(self, output_path: Path) -> bool:
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with urllib.request.urlopen(self.snapshot_url, timeout=self.timeout) as response:
                image_bytes = response.read()

            if not image_bytes:
                print("Warning: Camera snapshot response was empty.")
                return False

            output_path.write_bytes(image_bytes)
            return True
        except (OSError, urllib.error.URLError, TimeoutError) as exc:
            print(f"Warning: Failed to capture camera snapshot: {exc}")
            return False


class DatasetCollector:
    """Save Yi snapshots with matching antenna/RF metadata."""

    def __init__(
        self,
        camera: CameraController,
        output_dir: Path,
        rtsp_url: str,
    ) -> None:
        self.camera = camera
        self.output_dir = output_dir
        self.rtsp_url = rtsp_url

    def capture_angle_sample(
        self,
        logical_az: float,
        logical_el: float,
        command_az: Optional[float],
        command_el: Optional[float],
        rssi_snr: Optional[float],
        safety_status: str,
        platform_note: str,
    ) -> Optional[Path]:
        timestamp = datetime.now(timezone.utc)
        stamp = timestamp.strftime("%Y%m%dT%H%M%S_%fZ")
        base_name = f"{stamp}_az{logical_az:.3f}_el{logical_el:.3f}"
        image_path = self.output_dir / f"{base_name}.jpg"
        metadata_path = self.output_dir / f"{base_name}.json"

        if not self.camera.capture_snapshot(image_path):
            return None

        metadata = {
            "timestamp_utc": timestamp.isoformat(),
            "logical_az": logical_az,
            "logical_el": logical_el,
            "command_az": command_az,
            "command_el": command_el,
            "rssi_snr": rssi_snr,
            "safety_status": safety_status,
            "camera_snapshot_url": self.camera.snapshot_url,
            "camera_rtsp_url": self.rtsp_url,
            "image_file": image_path.name,
            "platform_note": platform_note,
        }
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        print(f"Captured sample: {image_path}")
        return image_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Linux Hardware_Agent for Orange Pi/Raspberry Pi deployment."
    )
    parser.add_argument("--rotctld", default=DEFAULT_ROTCTLD_EXECUTABLE)
    parser.add_argument("--model", default=DEFAULT_ROTATOR_MODEL)
    parser.add_argument("--serial-device", default=DEFAULT_SERIAL_DEVICE)
    parser.add_argument("--serial-speed", default=DEFAULT_SERIAL_SPEED)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--hamlib-port", type=int, default=DEFAULT_HAMLIB_PORT)
    parser.add_argument("--rigctl-port", type=int, default=DEFAULT_RIGCTL_PORT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--safe-window", type=float, default=DEFAULT_SAFE_WINDOW_DEGREES)
    parser.add_argument("--capture-interval", type=float, default=DEFAULT_CAPTURE_INTERVAL_SECONDS)
    parser.add_argument("--snapshot-url", default=DEFAULT_CAMERA_SNAPSHOT_URL)
    parser.add_argument("--rtsp-url", default=DEFAULT_CAMERA_RTSP_URL)
    parser.add_argument("--capture-dir", type=Path, default=DEFAULT_CAPTURE_DIR)
    parser.add_argument(
        "--no-start-rotctld",
        action="store_true",
        help="Connect to an already running rotctld instead of launching one.",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Disable camera snapshots and only print/monitor AZ/EL.",
    )
    return parser.parse_args()


def start_rotctld(args: argparse.Namespace) -> Optional[subprocess.Popen]:
    command = [
        args.rotctld,
        "-m",
        args.model,
        "-r",
        args.serial_device,
        "-s",
        args.serial_speed,
        "-t",
        str(args.hamlib_port),
        "-T",
        args.host,
        "-C",
        "timeout=200",
    ]
    print("Starting Linux Hamlib rotctld...")
    print("Command:", " ".join(command))

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        print(f"Warning: Failed to start rotctld: {exc}")
        return None

    time.sleep(2)
    if process.poll() is not None:
        print(f"Warning: rotctld exited early with code {process.returncode}")
        return process

    print("Linux Hamlib rotctld is running.")
    return process


def ask_for_center_angles() -> Tuple[float, float]:
    prompt = (
        "Please enter the target AZ and EL centers "
        "(Format: 'AZ EL', e.g., '120 163.2') or press Enter to use defaults:"
    )
    raw_value = input(prompt + " ").strip()
    if not raw_value:
        return DEFAULT_CENTER_AZ, DEFAULT_CENTER_EL

    try:
        az_text, el_text = raw_value.split()
        return float(az_text), float(el_text)
    except ValueError:
        print(
            "Warning: Invalid input. Using defaults "
            f"AZ={DEFAULT_CENTER_AZ}, EL={DEFAULT_CENTER_EL}."
        )
        return DEFAULT_CENTER_AZ, DEFAULT_CENTER_EL


def build_safety_limits(
    center_az: float,
    center_el: float,
    safe_window: float,
) -> dict[str, float]:
    return {
        "AZ_MIN": center_az - safe_window,
        "AZ_MAX": center_az + safe_window,
        "EL_MIN": center_el - safe_window,
        "EL_MAX": center_el + safe_window,
    }


def is_inside_limits(azimuth: float, elevation: float, limits: dict[str, float]) -> bool:
    return (
        limits["AZ_MIN"] <= azimuth <= limits["AZ_MAX"]
        and limits["EL_MIN"] <= elevation <= limits["EL_MAX"]
    )


def clamp_to_limits(azimuth: float, elevation: float, limits: dict[str, float]) -> Tuple[float, float]:
    safe_az = min(max(azimuth, limits["AZ_MIN"]), limits["AZ_MAX"])
    safe_el = min(max(elevation, limits["EL_MIN"]), limits["EL_MAX"])
    return safe_az, safe_el


def main() -> None:
    args = parse_args()
    process: Optional[subprocess.Popen] = None
    controller: Optional[NetworkController] = None

    try:
        if not args.no_start_rotctld:
            process = start_rotctld(args)

        center_az, center_el = ask_for_center_angles()
        limits = build_safety_limits(center_az, center_el, args.safe_window)

        print(f"Using center position: AZ={center_az:.3f}, EL={center_el:.3f}")
        print(
            "Safe limits: "
            f"AZ [{limits['AZ_MIN']:.3f}, {limits['AZ_MAX']:.3f}], "
            f"EL [{limits['EL_MIN']:.3f}, {limits['EL_MAX']:.3f}]"
        )

        controller = NetworkController(
            host=args.host,
            hamlib_port=args.hamlib_port,
            rigctl_port=args.rigctl_port,
            timeout=args.timeout,
        )
        controller.connect_all()

        collector: Optional[DatasetCollector] = None
        if not args.no_camera:
            collector = DatasetCollector(
                CameraController(args.snapshot_url, args.timeout),
                args.capture_dir,
                args.rtsp_url,
            )

        rigctl_unavailable_reported = False
        last_capture_time = 0.0
        platform_note = "linux_sbc_antenna_head_payload"

        print("Starting Linux safety monitor loop. Press Ctrl+C to stop.")
        while True:
            command_az: Optional[float] = None
            command_el: Optional[float] = None
            safety_status = "unknown"
            strength: Optional[float] = None

            current_angles = controller.read_current_angles()
            if current_angles is None:
                print("Warning: Current Hamlib angles unavailable. Motor command skipped.")
            else:
                current_az, current_el = current_angles
                if is_inside_limits(current_az, current_el, limits):
                    safety_status = "inside_limits"
                    print(f"Position OK: AZ={current_az:.3f}, EL={current_el:.3f}")
                else:
                    safe_az, safe_el = clamp_to_limits(current_az, current_el, limits)
                    command_az = safe_az
                    command_el = safe_el
                    safety_status = "corrected_to_limit"
                    print(
                        "Warning: Position outside safe limits. "
                        f"Current AZ={current_az:.3f}, EL={current_el:.3f}; "
                        f"commanding nearest safe point AZ={safe_az:.3f}, EL={safe_el:.3f}."
                    )
                    moved = controller.move_antenna(safe_az, safe_el)
                    if not moved:
                        print("Warning: Safety correction command was not confirmed.")

            if controller.rigctl_socket is None:
                if not rigctl_unavailable_reported:
                    print("RSSI/SNR: unavailable because rigctl/SDR is not connected.")
                    rigctl_unavailable_reported = True
            else:
                strength = controller.read_signal_strength()
                if strength is None:
                    print("RSSI/SNR: unavailable")
                else:
                    print(f"RSSI/SNR: {strength:.3f}")

            now = time.monotonic()
            if (
                collector is not None
                and current_angles is not None
                and now - last_capture_time >= args.capture_interval
            ):
                collector.capture_angle_sample(
                    logical_az=current_az,
                    logical_el=current_el,
                    command_az=command_az,
                    command_el=command_el,
                    rssi_snr=strength,
                    safety_status=safety_status,
                    platform_note=platform_note,
                )
                last_capture_time = now

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down Linux hardware layer...")
    finally:
        if controller is not None:
            controller.close()

        if process is not None:
            print("Stopping Linux Hamlib rotctld...")
            process.terminate()
            try:
                process.wait(timeout=5)
                print("Linux Hamlib rotctld stopped.")
            except subprocess.TimeoutExpired:
                print("Warning: rotctld did not stop within 5 seconds.")


if __name__ == "__main__":
    main()
