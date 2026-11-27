# S.A.M. (Servo Arm Manipulator)

An Eye-to-Hand Computer Vision and AI-Controlled 5-Axis Robotic Arm system built with OpenCV, MediaPipe Hands, PySerial, Google Gemini API, and Speech Recognition.

---

## System Architecture

```
                       +-------------------------+
                       |      External Webcam    |
                       +------------+------------+
                                    | Video Stream (OpenCV)
                                    v
                       +-------------------------+
                       |         app.py          |
                       |  - Flask Mission Control|
                       |  - 4 White Dots Tracker |
                       |  - 4 Cubes HSV Filter   |
                       |  - MediaPipe Hand Model |
                       |  - Visual Servoing IBVS |
                       +------+-----------+------+
                              |           |
            Voice ("give it") |           | Serial (<J1,J2,J3,J4,J5>)
                              v           v
            +-------------------+       +-------------------+
            | Laptop Microphone |       |  Arduino Uno/Mega |
            | SpeechRecognition |       | sam_firmware.ino  |
            +-------------------+       +---------+---------+
                                                  | PWM Signals
                                                  v
                                        +-------------------+
                                        |  5-Axis Servo Arm |
                                        | (Base, Shoulder,  |
                                        |  Elbow, Wrist,    |
                                        |  Gripper)         |
                                        +-------------------+
```

---

## 1. Hardware Pinout & Wiring

| Axis | Servo Function | Arduino Pin | Servo Model | Safe Angle Range | Pulse Width Range |
|---|---|---|---|---|---|
| Joint 1 | Base Rotation (Pan) | Pin 8 | Standard Servo | 0° – 180° | 550 µs – 2450 µs |
| Joint 2 | Shoulder (Pitch/Elevation) | Pin 9 | Standard Servo | 0° – 180° | 550 µs – 2450 µs |
| Joint 3 | Elbow (Pitch/Reach) | Pin 10 | MG995 Metal Gear | 0° – 180° | 600 µs – 2400 µs |
| Joint 4 | Wrist Pitch (Flexion) | Pin 11 | MG90S Micro | 0° – 180° | 560 µs – 2400 µs |
| Joint 5 | Wrist Roll (Rotation) | Pin 12 | Standard Servo | 0° – 160° | 550 µs – 2450 µs |
| Joint 6 | Gripper (End-Effector Claw) | Pin 13 | MG90S Micro | 20° (Closed) – 160° (Open) | 560 µs – 2400 µs |

> [!CAUTION]
> **Power Supply & Common Ground Requirement**:
> - **NEVER power the servos (especially the high-torque MG995 on J3) from the Arduino 5V pin!**
> - The MG995 draws 1.2A to 2.5A during motion/stall. Drawing this from the Arduino will collapse the 5V rail, trigger a brownout reset (BOR), drop the serial link, or destroy the onboard linear regulator.
> - **Use a dedicated external regulated 5V or 6V (4A to 6A) DC power supply** for the servos.
> - **Connect the External Power GND to the Arduino GND (Common Ground)**. Without a shared ground reference, the PWM control signals will float, causing servos to jitter, twitch, or fail to respond.

---

## 2. Software Installation

Install all required Python dependencies:

```bash
pip install -r requirements.txt
```

If you encounter issues installing `pyaudio` on Windows, install it via `pipwin`:
```bash
pip install pipwin
pipwin install pyaudio
```

### (Optional) Gemini AI Integration
Set your Gemini API key to enable high-level cognition and intention logging:
```bash
# Windows PowerShell
$env:GEMINI_API_KEY="your_api_key_here"

# Linux / macOS
export GEMINI_API_KEY="your_api_key_here"
```

---

## 3. Flash Arduino Firmware

1. Open Arduino IDE.
2. Open [`sam_firmware/sam_firmware.ino`](file:///c:/Users/VARUN/Desktop/ai%20blah%20%20blah/servo%20arm/sam_firmware/sam_firmware.ino).
3. Select your board (Arduino Uno / Mega / Nano) and COM Port.
4. Upload the sketch.

> [!TIP]
> **Power-Safe Low-Current Tricks Included in Firmware:**
> - **Move One Joint at a Time**: The firmware finishes each joint's motion completely, stops, waits 200ms, and only then starts the next joint. Peak current stays safely under 2 - 3 Amps so the board never brown-outs or crashes.
> - **Sweep, Do Not Jump**: Each servo moves 1° at a time via a for-loop with a 15ms step delay, eliminating stall current spikes and servo jitter.
> - **Staggered Boot Sequence**: On startup, each servo attaches and homes sequentially with 200ms pauses to prevent inrush current surges on USB power.

---

## 4. Running the System

Start the Web Mission Control application:

```bash
python app.py
```

Then open your browser and navigate to:
**[http://localhost:5000](http://localhost:5000)**

### Mission Control Features
- **Live Camera Feed**: 480p low-latency MJPEG video stream with target overlays.
- **3D Digital Twin**: Interactive real-time 3D servo arm visualizer powered by Three.js.
- **Manual Joint Controls**: Direct angle sliders for Joint 1 (Pan), Joint 2 (Shoulder), Joint 3 (Elbow), Joint 4 (Wrist), and Joint 5 (Gripper).
- **Computer Vision Targeting**: Automatic detection and tracking for Red, Green, Blue, and Black cubes, plus 4-dot gripper visual servoing.
- **Voice Commands**: Listen for "give it to me" delivery trigger.
- **Gemini AI Integration**: High-level cognition and intention logging.

