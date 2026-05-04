"""
All-in-one Hardware_Agent lifecycle script.

Responsibilities:
  1. Start Hamlib rotctld.exe internally with subprocess.Popen.
  2. Ask the operator for safe AZ/EL center angles.
  3. Connect to Hamlib and SDR++ over localhost TCP sockets.
  4. Monitor current antenna angles and only intervene when they leave the
     configured safe operating window.
  5. Capture Yi camera snapshots and bind each image to timestamped AZ/EL
     metadata for later CV_Agent calibration tests.

This file intentionally does not depend on any legacy code or batch scripts.
"""

from __future__ import annotations

import re
import socket
import subprocess
import time
import json
import os
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple


def load_dotenv(path: Path = Path(".env")) -> None:
    """
    Load KEY=VALUE pairs from a local .env file without third-party packages.

    Existing environment variables win over .env values, which allows temporary
    command-line overrides during field tests.
    """
    if not path.exists():
        return

    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if "=" not in line:
            print(f"Warning: Ignoring invalid .env line {line_number}: {raw_line!r}")
            continue

        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")

        if not key:
            print(f"Warning: Ignoring .env line {line_number} with empty key.")
            continue

        os.environ.setdefault(key, value)


def getenv_float(name: str, default: float) -> float:
    """Read a float from the environment and fall back safely if invalid."""
    raw_value = os.getenv(name)
    if raw_value is None:
        return default

    try:
        return float(raw_value)
    except ValueError:
        print(f"Warning: Invalid float for {name}={raw_value!r}. Using {default}.")
        return default


def getenv_int(name: str, default: int) -> int:
    """Read an integer from the environment and fall back safely if invalid."""
    raw_value = os.getenv(name)
    if raw_value is None:
        return default

    try:
        return int(raw_value)
    except ValueError:
        print(f"Warning: Invalid integer for {name}={raw_value!r}. Using {default}.")
        return default


load_dotenv()

HAMLIB_EXECUTABLE = os.getenv("HAMLIB_EXECUTABLE_WINDOWS", r"D:\hamlib-w64-4.7.1\bin\rotctld.exe")
HAMLIB_MODEL = os.getenv("HAMLIB_MODEL", "1")
HAMLIB_SERIAL_DEVICE = os.getenv("HAMLIB_SERIAL_DEVICE_WINDOWS", "")
HAMLIB_SERIAL_SPEED = os.getenv("HAMLIB_SERIAL_SPEED", "115200")
HAMLIB_HOST = os.getenv("HAMLIB_HOST", "127.0.0.1")
HAMLIB_PORT = getenv_int("HAMLIB_PORT", 4533)
RIGCTL_PORT = getenv_int("RIGCTL_PORT", 4532)
SOCKET_TIMEOUT = getenv_float("SOCKET_TIMEOUT", 2.0)

HAMLIB_ARGUMENTS = [
    "-m",
    HAMLIB_MODEL,
    "-s",
    HAMLIB_SERIAL_SPEED,
    "-t",
    str(HAMLIB_PORT),
    "-T",
    HAMLIB_HOST,
    "-C",
    "timeout=200",
    "-vvv"
]

if HAMLIB_SERIAL_DEVICE:
    HAMLIB_ARGUMENTS[2:2] = ["-r", HAMLIB_SERIAL_DEVICE]

DEFAULT_CENTER_AZ = getenv_float("DEFAULT_CENTER_AZ", 180.0)
DEFAULT_CENTER_EL = getenv_float("DEFAULT_CENTER_EL", 167.5)
SAFE_WINDOW_DEGREES = getenv_float("SAFE_WINDOW_DEGREES", 20.0)

CAMERA_SNAPSHOT_URL = os.getenv(
    "CAMERA_SNAPSHOT_URL",
    "http://camera.local/cgi-bin/snapshot.sh?res=high&watermark=yes",
)
CAMERA_RTSP_URL = os.getenv("CAMERA_RTSP_URL", "rtsp://camera.local/ch0_0.h264")
CAPTURE_DIR = Path("captures")


class NetworkController:
    """Timed TCP socket interface for Hamlib rotctld and SDR++ rigctl."""

    def __init__(
        self,
        host: str = HAMLIB_HOST,
        hamlib_port: int = HAMLIB_PORT,
        sdr_port: int = RIGCTL_PORT,
        timeout: float = SOCKET_TIMEOUT,
    ) -> None:
        self.host = host
        self.hamlib_port = hamlib_port
        self.sdr_port = sdr_port
        self.timeout = timeout
        self.hamlib_socket: Optional[socket.socket] = None
        self.sdr_socket: Optional[socket.socket] = None

    def connect_all(self) -> None:
        """Connect both TCP services. Failures are printed, not raised."""
        self.hamlib_socket = self._connect_socket("Hamlib rotctld", self.hamlib_port)
        self.sdr_socket = self._connect_socket("SDR++ rigctl", self.sdr_port)

    def move_antenna(self, azimuth: float, elevation: float) -> bool:
        """Send a Hamlib position command: P {az} {el}\\n."""
        command = f"P {azimuth:.3f} {elevation:.3f}\n"
        response = self._send_and_receive("Hamlib rotctld", self.hamlib_socket, command)
        if response is None:
            return False

        if "RPRT" in response and "RPRT 0" not in response:
            print(f"Warning: Hamlib rejected move command: {response!r}")
            return False

        return True

    def read_current_angles(self) -> Optional[Tuple[float, float]]:
        """Read current logical AZ/EL from Hamlib using p\\n."""
        response = self._send_and_receive("Hamlib rotctld", self.hamlib_socket, "p\n")
        if response is None:
            return None

        values = self._extract_floats(response)
        if len(values) < 2:
            print(f"Warning: Could not parse Hamlib angles from: {response!r}")
            return None

        return values[0], values[1]

    def read_signal_strength(self) -> Optional[float]:
        """Read SDR++ signal strength using \\get_level STRENGTH\\n."""
        response = self._send_and_receive(
            "SDR++ rigctl",
            self.sdr_socket,
            "\\get_level STRENGTH\n",
        )
        if response is None:
            return None

        values = self._extract_floats(response)
        if not values:
            print(f"Warning: Could not parse SDR++ RSSI/SNR from: {response!r}")
            return None

        return values[0]

    def close(self) -> None:
        """Close both sockets if they were opened."""
        self._close_socket("Hamlib rotctld", self.hamlib_socket)
        self._close_socket("SDR++ rigctl", self.sdr_socket)
        self.hamlib_socket = None
        self.sdr_socket = None

    def _connect_socket(self, name: str, port: int) -> Optional[socket.socket]:
        """Create one timeout-protected TCP connection."""
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
        """Send a command and read the short text response with a timeout."""
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
        """Receive available response data without blocking forever."""
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
        """Extract floats even when values are mixed with rigctl status text."""
        matches = re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", text)
        return [float(match) for match in matches]

    @staticmethod
    def _close_socket(name: str, sock: Optional[socket.socket]) -> None:
        """Close one socket and keep shutdown errors non-fatal."""
        if sock is None:
            return

        try:
            sock.close()
            print(f"{name}: socket closed")
        except OSError as exc:
            print(f"Warning: Error while closing {name} socket: {exc}")


class CameraController:
    """HTTP snapshot client for the Yi camera running yi-hack-MStar."""

    def __init__(self, snapshot_url: str = CAMERA_SNAPSHOT_URL, timeout: float = 2.0) -> None:
        self.snapshot_url = snapshot_url
        self.timeout = timeout

    def capture_snapshot(self, output_path: Path) -> bool:
        """Download one high-resolution JPEG snapshot to output_path."""
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
    """Bind antenna state, RF state, and Yi camera images into a dataset."""

    def __init__(self, camera: CameraController, output_dir: Path = CAPTURE_DIR) -> None:
        self.camera = camera
        self.output_dir = output_dir

    def capture_angle_sample(
        self,
        logical_az: float,
        logical_el: float,
        command_az: Optional[float],
        command_el: Optional[float],
        rssi_snr: Optional[float],
        safety_status: str,
    ) -> Optional[Path]:
        """Capture one image and write a JSON sidecar with matching AZ/EL data."""
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
            "camera_rtsp_url": CAMERA_RTSP_URL,
            "image_file": image_path.name,
        }
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

        print(f"Captured sample: {image_path}")
        return image_path


def start_rotctld() -> Optional[subprocess.Popen]:
    """Launch rotctld.exe in the background and give its TCP server time to start."""
    command = [HAMLIB_EXECUTABLE, *HAMLIB_ARGUMENTS]
    print("Starting Hamlib rotctld.exe...")
    print("Command:", " ".join(command))

    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        print(f"Warning: Failed to start rotctld.exe: {exc}")
        return None

    time.sleep(2)
    if process.poll() is not None:
        print(f"Warning: rotctld.exe exited early with code {process.returncode}")
        return process

    print("Hamlib rotctld.exe is running.")
    return process


def ask_for_center_angles() -> Tuple[float, float]:
    """Prompt the operator for center AZ/EL and fall back to safe defaults."""
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


def build_safety_limits(center_az: float, center_el: float) -> dict[str, float]:
    """Create a +/-20 degree safe operating box around the chosen center."""
    return {
        "AZ_MIN": center_az - SAFE_WINDOW_DEGREES,
        "AZ_MAX": center_az + SAFE_WINDOW_DEGREES,
        "EL_MIN": center_el - SAFE_WINDOW_DEGREES,
        "EL_MAX": center_el + SAFE_WINDOW_DEGREES,
    }


def is_inside_limits(azimuth: float, elevation: float, limits: dict[str, float]) -> bool:
    """Return True only if the requested position is inside the safe box."""
    return (
        limits["AZ_MIN"] <= azimuth <= limits["AZ_MAX"]
        and limits["EL_MIN"] <= elevation <= limits["EL_MAX"]
    )


def clamp_to_limits(azimuth: float, elevation: float, limits: dict[str, float]) -> Tuple[float, float]:
    """Clamp an unsafe AZ/EL position to the nearest safe boundary."""
    safe_az = min(max(azimuth, limits["AZ_MIN"]), limits["AZ_MAX"])
    safe_el = min(max(elevation, limits["EL_MIN"]), limits["EL_MAX"])
    return safe_az, safe_el


def main() -> None:
    process: Optional[subprocess.Popen] = None
    controller: Optional[NetworkController] = None

    try:
        process = start_rotctld()

        center_az, center_el = ask_for_center_angles()
        limits = build_safety_limits(center_az, center_el)

        print(f"Using center position: AZ={center_az:.3f}, EL={center_el:.3f}")
        print(
            "Safe limits: "
            f"AZ [{limits['AZ_MIN']:.3f}, {limits['AZ_MAX']:.3f}], "
            f"EL [{limits['EL_MIN']:.3f}, {limits['EL_MAX']:.3f}]"
        )

        controller = NetworkController(timeout=SOCKET_TIMEOUT)
        controller.connect_all()
        collector = DatasetCollector(CameraController(timeout=SOCKET_TIMEOUT))

        sdr_unavailable_reported = False

        print("Starting safety monitor loop. Press Ctrl+C to stop.")
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

            if controller.sdr_socket is None:
                if not sdr_unavailable_reported:
                    print("RSSI/SNR: unavailable because SDR++ rigctl is not connected.")
                    sdr_unavailable_reported = True
            else:
                strength = controller.read_signal_strength()
                if strength is None:
                    print("RSSI/SNR: unavailable")
                else:
                    print(f"RSSI/SNR: {strength:.3f}")

            if current_angles is not None:
                collector.capture_angle_sample(
                    logical_az=current_az,
                    logical_el=current_el,
                    command_az=command_az,
                    command_el=command_el,
                    rssi_snr=strength,
                    safety_status=safety_status,
                )

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down hardware layer...")
    finally:
        if controller is not None:
            controller.close()

        if process is not None:
            print("Stopping Hamlib rotctld.exe...")
            process.terminate()
            try:
                process.wait(timeout=5)
                print("Hamlib rotctld.exe stopped.")
            except subprocess.TimeoutExpired:
                print("Warning: rotctld.exe did not stop within 5 seconds.")


if __name__ == "__main__":
    main()
