# Vision-Based LEO Satellite Tracking and Calibration System
*(基于计算机视觉的低轨卫星追踪与校准系统)*

## 1. Project Overview
This project is a Master-level thesis focused on developing an AI-driven, vision-assisted closed-loop satellite tracking ground station. The hardware architecture is strictly centralized on a single Windows PC. The system controls an industrial SPID MD-02 (AZ/EL) rotator and processes RF signals via an SDRplay RSP1B receiver. 
To compensate for mechanical backlash and resolver/encoder inaccuracies, a camera is mounted directly on the antenna bracket. The PC processes the live video stream to perform "Movement Detection" and "Absolute Position Detection", acting as a high-precision optical encoder to correct the motor's logical coordinates.

## 2. Hardware & Software Stack
- **Rotator**: SPID SPX AZ/EL-01 with MD-02 Controller.
- **RF Receiver**: SDRplay RSP1B + Nooelec LNA.
- **Vision Sensor**: Xiaoyi Yi Outdoor FullHD 1080P LED network camera mounted on the antenna frame, running the `roleoroleo/yi-hack-MStar` custom firmware.
  - Keep WiFi credentials, camera IPs, MQTT broker addresses, and other site-specific network details outside versioned project files.
  - High-resolution RTSP stream pattern: `rtsp://<camera-host>/ch0_0.h264`.
  - High-resolution snapshot endpoint pattern: `http://<camera-host>/cgi-bin/snapshot.sh?res=high&watermark=yes`.
  - MQTT motion-event support can be enabled through a local MQTT broker; MQTT is optional and should be configured locally.
- **Software Interfaces**: Hamlib (`rotctld.exe` on TCP 4533) for motor control; SDR++ (`rigctl` on TCP 4532) for RF signal strength.
- **Core Language**: Python 3.10+ (OpenCV, NumPy).

---

## 3. Agent Roles & Directives

### 🤖 Agent 1: Vision-Based Movement & Position Detector (CV_Agent)
**Role**: Process the live camera feed on the PC to calculate the true physical movement and absolute Az/El angles.
**Tasks**:
1. **Network Camera Acquisition**: Capture imagery from the Yi camera either through the high-resolution RTSP stream (`rtsp://<camera-host>/ch0_0.h264`) or the high-resolution HTTP snapshot endpoint (`http://<camera-host>/cgi-bin/snapshot.sh?res=high&watermark=yes`).
2. **Angle-Image Dataset Collection**: When `Hardware_Agent` issues or confirms an AZ/EL position, save the corresponding snapshot/video frame with timestamp, logical AZ/EL, command AZ/EL, and optional RSSI/SNR metadata.
3. **Movement Detection (Optical Odometry)**: Use Optical Flow (e.g., Lucas-Kanade) or Feature Matching (SIFT/ORB) to track pixel displacement between consecutive frames. Translate this pixel shift into physical angular movement ($\Delta Az$, $\Delta El$) to verify if the motor actually moved the commanded 0.2° step.
4. **Absolute Position Detection**: Use static environmental landmarks as an absolute reference frame to compute the true physical Azimuth and Elevation of the antenna.
5. **MQTT Motion Events**: Optionally subscribe to yi-hack-MStar MQTT motion events from a locally configured broker for event-triggered capture. MQTT is an auxiliary trigger, not the primary angle truth source.
**Context for Prompts**: "Write a Python script using OpenCV that reads the Yi camera RTSP stream or HTTP snapshot endpoint, captures images tagged with Hamlib AZ/EL angles, and later uses visual features to estimate antenna orientation."

### 🤖 Agent 2: Hardware Control & Interactive Setup (Hardware_Agent)
**Role**: Handle hardware initialization, interactive safety configurations, internal process management, and TCP socket communications from scratch.
**Tasks**:
1. **Internal Process Management**: Do NOT write any `.bat` scripts. Instead, use Python's built-in `subprocess.Popen` to launch the Hamlib executable (`C:\Program Files\hamlib-w64-4.7.0\bin\rotctld.exe` with arguments: `-m 901 -r COM7 -s 115200 -t 4533 -T 127.0.0.1 -C timeout=200`) dynamically at the beginning of the Python script. Use `time.sleep(2)` to ensure the server starts properly, and guarantee that the process is safely terminated (`process.terminate()`) in a `finally` block when the script exits.
2. **Interactive Safety Limits**: Upon execution, the script MUST pause and prompt the user to input the target Azimuth and Elevation limits/centers (e.g., `120 163.2`). 
   - If the user simply presses "Enter", the script falls back to default hardcoded safe values.
   - If the user enters new values, the script parses them and updates the operational limits.
3. **TCP Interfaces**: Establish non-blocking TCP socket connections to Hamlib (4533) for motor commands (`P {Az} {El}`) and SDR++ (4532) for RF signal strength reading.
4. **Capture Coordination**: Expose timestamped AZ/EL state after each motor command or position read so `CV_Agent` can bind camera snapshots/video frames to the exact logical antenna angles.
**Context for Prompts**: "Act as Hardware_Agent. Assume NO previous code exists. First, use `subprocess.Popen` to launch `rotctld.exe` internally. Make sure to gracefully kill this subprocess on exit. Second, use `input()` to ask the user for AZ and EL configuration (Format: 'AZ EL'). If the input is empty, use defaults. Otherwise, parse the floats. Finally, implement TCP classes for Hamlib/SDR++ and provide timestamped AZ/EL metadata for camera capture."

### 🤖 Agent 3: Error Compensation & Master Controller (Core_Agent)
**Role**: Fuse the logical encoder data, visual odometry data, and execute precise tracking.
**Tasks**:
1. **Error Calculation**: Continuously compare the logical angle reported by Hamlib (MD-02 encoder) with the true physical angle calculated by the `CV_Agent`. 
2. **Backlash Compensation**: Implement a regression model to map the resolver error patterns and output a correction offset.
3. **Closed-Loop Tracking**: Add the calculated visual offset to the baseline tracking trajectory and send the corrected command to the antenna via `Hardware_Agent`.
**Context for Prompts**: "Design a Python control loop that compares a motor's logical encoder angle with a camera's visually calculated true angle. Use the difference to train a simple regression model that outputs an error compensation value for future motor commands."
