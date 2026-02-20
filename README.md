<div align="center">

# 🤖 Vision-Controlled Robotic Arm

**A 3D-printed robotic arm that mimics your movements in real time using just a camera.**

![Demo of the Robotic Arm](media/demo.gif)

![Status](https://img.shields.io/badge/Status-Complete-success)
![Python](https://img.shields.io/badge/Python-3.x-blue)
![C++](https://img.shields.io/badge/C++-Arduino-blue)
![Hardware](https://img.shields.io/badge/Raspberry_Pi_5-red)
![License](https://img.shields.io/badge/License-MIT-green)

</div>

---

> **🌐 Language Note:** This README is written in **English**. However, the full technical report ([PDF](RIA_G7_Relatorio_Final_Braco_Robotico.pdf)) is in **Portuguese**. All source code files have two versions — an English version and a Portuguese version (files ending in `_PT`). Both versions are functionally identical; only comments and variable names differ. Use whichever you prefer.

---

## 📖 What Is This Project?

This is a **humanoid robotic arm** that imitates your arm and hand movements in real time. Instead of buttons or joysticks, you simply stand in front of a camera — the system recognizes your gestures and the robot copies them automatically.

The arm has **7 independent degrees of freedom**: 5 fingers that open and close, an elbow that goes up and down, and a rotating base that turns left and right. The entire mechanical structure was 3D-printed using the open-source [InMoov](https://inmoov.fr/) project as a foundation.

**This README is a complete tutorial.** If you follow every step, you'll be able to build your own robotic arm — even without much experience in programming or electronics.

---

## 🎬 Demo

|       Full System Demo       |            Finger Control             |           Base Rotation           |
| :--------------------------: | :-----------------------------------: | :-------------------------------: |
| ![Full demo](media/demo.gif) | ![Finger test](media/finger-test.gif) | ![Base test](media/base-test.gif) |

---

## 📁 Repository Structure

Before starting, it's useful to know where everything is:

```
HumanoidRoboticArmVision/
│
├── 📄 README.md                          ← You are here! The full tutorial
├── 📄 REQUIREMENTS.txt                   ← Software versions and libraries
├── 📄 LICENSE                            ← MIT License
├── 📄 RIA_G7_Relatorio_Final_Braco_Robotico.pdf  ← Full technical report (Portuguese)
│
├── 📂 code/
│   ├── 📂 Arduino/Final_Ard/
│   │   ├── 📂 MotorDriver/               ← Arduino MEGA firmware (English)
│   │   ├── 📂 MotorDriver_PT/            ← Arduino MEGA firmware (Portuguese)
│   │   └── 📂 Arduino_Uno_CNC_Final/     ← Arduino Uno firmware (stepper motor)
│   │
│   ├── 📂 RPi/Final_RPI/
│   │   ├── ArmController.py              ← Computer vision for Raspberry Pi (English)
│   │   └── ArmController_PT.py           ← Computer vision for Raspberry Pi (Portuguese)
│   │
│   └── 📂 PC/
│       ├── VisionDebugger_PC.py           ← Debug tool for PC (English)
│       ├── VisionDebugger_PC_PT.py        ← Debug tool for PC (Portuguese)
│       ├── hand_landmarker.task           ← MediaPipe model (hand)
│       └── pose_landmarker_lite.task      ← MediaPipe model (body)
│
├── 📂 Codigos_PDFs/                       ← Source code as PDF (report annexes, Portuguese)
│
└── 📂 media/                              ← Project images and GIFs
```

> **Note:** Each code file exists in **two versions** — English and Portuguese (`_PT` suffix). They are functionally identical; only the comments and variable names change.

---

## 🧾 Bill of Materials

Here's everything you need to build the arm. Some components can be swapped for equivalents.

### Electronics & Motors

| Component                     | Reference                        | Qty | Notes                                            |
| :---------------------------- | :------------------------------- | :-: | :----------------------------------------------- |
| Servo Motor (elbow)           | **DS5160** (60 kgf·cm)           |  1  | Needs to be powerful — supports the arm's weight |
| Servo Motor (fingers + wrist) | **MG996R** (9-11 kgf·cm)         |  6  | 5 for fingers + 1 for wrist rotation             |
| Stepper Motor (base)          | **17HS4401S** (NEMA 17)          |  1  | Precise rotation of the base                     |
| Servo Driver                  | **PCA9685** (16-channel, I2C)    |  1  | Controls all servos using just 2 pins            |
| Stepper Driver                | **A4988**                        |  1  | Plugs into the CNC Shield                        |
| CNC Shield                    | **CNC Shield V3**                |  1  | Mounts directly on the Arduino Uno               |
| Raspberry Pi                  | **Raspberry Pi 5** (8GB)         |  1  | The "brain" — processes the vision               |
| Camera                        | **Raspberry Pi Camera Module 3** |  1  | Dedicated camera for the RPi                     |
| Arduino                       | **Arduino MEGA 2560**            |  1  | Central motor controller                         |
| Arduino                       | **Arduino Uno**                  |  1  | Dedicated to the stepper motor                   |

### Structure & Mechanics

| Component    | Specification                      |   Qty   | Notes                                        |
| :----------- | :--------------------------------- | :-----: | :------------------------------------------- |
| PLA Filament | 1.75mm, any color                  |  ~500g  | For 3D printing all parts                    |
| Springs      | 3/16″ x 1-3/4″ (4.8mm x 44.5mm)    |    5    | Finger return mechanism                      |
| Tendons      | Braided fishing line, 0.8mm, 200LB | 5x 50cm | Pull the fingers closed                      |
| Teflon Tubes | ID 1.5mm x OD 2.5mm                | ~160cm  | Guide the tendons                            |
| Bearing      | (commercial, not printed)          |    1    | Connects the fixed base to the rotating part |
| Magnets      | Ø 2.5mm, height 1mm                |    5    | Magnetic attachment                          |

### Power

| Component      | Specification          | Notes                                  |
| :------------- | :--------------------- | :------------------------------------- |
| Power Supply 1 | **7V / 3A**            | For the servo motors (via PCA9685)     |
| Power Supply 2 | **12-13V / 2A**        | For the stepper motor (via CNC Shield) |
| USB Cables     | USB-A to USB-B         | To power the Arduinos                  |
| RPi 5 Power    | **Official 27W USB-C** | For the Raspberry Pi 5                 |

### Other

| Component                     | Notes                              |
| :---------------------------- | :--------------------------------- |
| Heat shrink tubing            | To insulate soldered joints        |
| Hookup wire                   | For connections between components |
| Polyimide tape (Kapton)       | Heat protection                    |
| Assorted screws (M3, M4)      | To secure printed parts            |
| Ecoflex™ 00-10 (RTV Silicone) | Optional — for fingertip grip pads |

---

## 🔧 Step 1: 3D Printing the Parts

The arm's structure is based on the open-source **InMoov** project. You need to download the STL files and print them.

### Where to Download the Models

| Part                 | Link                                                           |
| :------------------- | :------------------------------------------------------------- |
| 🖐️ Hand and Forearm  | [InMoov — Hand and Forearm](https://inmoov.fr/hand-and-forarm) |
| 🖐️ Hand (I2 version) | [InMoov — Hand I2](https://inmoov.fr/hand-i2)                  |

> **💡 Tip:** The **Hand I2** version is the most recent and includes design improvements for the fingers. We recommend using this one.

### Print Settings

We used a **Bambu Lab A1** printer, but any FDM printer will work with these settings:

| Parameter              |  InMoov Parts   | Custom Parts (Base) |
| :--------------------- | :-------------: | :-----------------: |
| **Material**           |       PLA       |         PLA         |
| **Nozzle Temperature** |      220°C      |        220°C        |
| **Bed Temperature**    |      65°C       |        65°C         |
| **Layer Height**       |     0.20mm      |       0.20mm        |
| **Infill Density**     |     **30%**     |       **25%**       |
| **Wall Loops**         |        2        |          2          |
| **Speed**              | Standard (100%) |   Standard (100%)   |

### ⚠️ Common Issue: Warping

During printing, part edges may lift off the bed (called _warping_). If this happens:

1. **Enable "Brim"** in your slicer — this creates a thin rim around the part that increases bed adhesion.
2. **You don't need** to raise the bed temperature or lower the speed. With Brim enabled, standard settings work just fine.

![3D printed parts](media/printed-parts.jpg)

---

## 🔩 Step 2: Mechanical Assembly

After printing all the parts, it's time to assemble.

### Hand and Forearm Assembly

Follow the official InMoov instructions:

- 📘 [Assembly guide — Hand and Forearm](https://inmoov.fr/hand-and-forarm)
- 📘 [Assembly guide — Hand I2](https://inmoov.fr/hand-i2)

Each InMoov page has step-by-step photos and videos explaining how to thread the tendons (fishing line), install the return springs, and route the teflon tubes.

### Base Assembly (Custom Part)

The base **is not part of InMoov** — we designed it ourselves in SolidWorks for this project. It includes:

- **Main structure** — houses the stepper motor, Raspberry Pi, and both Arduinos.
- **Motor spacers** — ensure the motor shaft reaches the rotary coupling.
- **Arm coupling interface** — two pieces (lower base + upper base) that connect the bearing to the arm.
- **Rotary coupling** — transmits motion from the stepper motor to the rotating platform.
- **Motor mount bracket** — secures the motor to the main structure.

> **💡 Why a custom base?** The InMoov project includes a full torso, but we only needed the arm. So we designed a compact base that houses all the electronics and supports the arm's weight without overloading the motor.

|                Assembly Process                 |               Assembled Arm               |              Base Detail              |
| :---------------------------------------------: | :---------------------------------------: | :-----------------------------------: |
| ![Assembly process](media/assembly-process.jpg) | ![Assembled arm](media/assembled-arm.jpg) | ![Base detail](media/base-detail.jpg) |

---

## ⚡ Step 3: Electronics & Wiring

This is the part that requires the most attention. The system uses **three processors** working together:

```
┌──────────────┐     UART      ┌──────────────────┐     UART      ┌──────────────┐
│ Raspberry Pi │ ──────────►  │  Arduino MEGA    │ ──────────►  │  Arduino Uno │
│   5 (8GB)    │  GPIO14→RX1  │     2560         │  TX1→RX      │              │
│              │              │                  │              │  + CNC Shield │
│ Vision (AI)  │              │ Central Control  │              │ Stepper Motor│
└──────────────┘              └──────────────────┘              └──────────────┘
                                      │
                                      │ I2C (SDA/SCL)
                                      ▼
                              ┌──────────────────┐
                              │    PCA9685       │
                              │  Servo Driver    │
                              │  (16 channels)   │
                              └──────────────────┘
                                      │
                          ┌───────────┼───────────┐
                          ▼           ▼           ▼
                      MG996R Servos  DS5160 Servo  ...
                      (fingers/wrist) (elbow)
```

> **💡 Why three processors instead of one?** The Raspberry Pi needs all its processing power for the AI-based vision. If it also had to control motors at the same time, everything would lag and the movements would be jerky. By splitting the tasks, each processor does what it's best at — no delays.

### Wiring Table

| From (Source)  | To (Destination) |  Type  |     Source Pins     |  Dest. Pins   | What It Does                      |
| :------------- | :--------------- | :----: | :-----------------: | :-----------: | :-------------------------------- |
| Raspberry Pi 5 | Arduino MEGA     |  UART  |    GPIO 14 (TX)     | Pin 19 (RX1)  | Sends vision data                 |
| Arduino MEGA   | Arduino Uno      |  UART  |    Pin 18 (TX1)     |  Pin 0 (RX)   | Sends base commands ('0','1','2') |
| Arduino MEGA   | PCA9685          |  I2C   | SDA (20) / SCL (21) |   SDA / SCL   | Controls the 7 servo motors       |
| Arduino Uno    | CNC Shield V3    | Shield |    Pins 2, 5, 8     | STEP, DIR, EN | Controls the stepper motor        |
| 7V/3A Supply   | PCA9685          |  Wire  |          —          |  V+ terminal  | Powers the servo motors           |
| 12V/2A Supply  | CNC Shield       |  Wire  |          —          | 12V terminal  | Powers the stepper motor          |

> **⚠️ IMPORTANT:** Connect the **GND** (ground) of all components together. Without a common ground, serial communication will not work.

### The PCA9685 Module — Simplified Servo Control

The PCA9685 is a key component. Without it, you'd need 7 PWM pins from the Arduino to control 7 servos — and the Arduino doesn't have that many stable PWM pins. With the PCA9685:

- You control **up to 16 servos** using just **2 pins** (I2C: SDA and SCL).
- The **motor power** comes from an external supply (7V), not from the Arduino. This protects the Arduino from current spikes.

### CNC Shield V3 + A4988 — Stepper Motor

The CNC Shield plugs directly into the Arduino Uno and accepts the A4988 driver. The connections are automatic — just plug it in and wire the stepper motor to the X-axis terminals.

> **💡 Why separate power supplies?** Motors draw a lot of current and generate electrical noise. If they shared the same supply as the Arduinos, the Arduinos could randomly restart or behave erratically. Separate supplies = stable system.

![Wiring overview](media/wiring-overview.jpg)

For the full interactive wiring diagram, visit the [Cirkit Designer project](https://app.cirkitdesigner.com/project/).

---

## 💾 Step 4: Software — Arduino (Firmware)

The firmware is the code that runs inside the Arduinos. You need to upload two different programs.

### 4.1 — Arduino Uno (Base Stepper Motor)

**File:** [`Arduino_Uno_CNC_Final.ino`](code/Arduino/Final_Ard/Arduino_Uno_CNC_Final/Arduino_Uno_CNC_Final.ino)

**What it does:** Receives simple commands (`'0'`, `'1'`, or `'2'`) from the Arduino MEGA and rotates the stepper motor to the correct position (left, center, or right).

**How to upload:**

1. Open the **Arduino IDE**.
2. Connect the **Arduino Uno** to your PC via USB.
3. Go to _Tools > Board > Arduino Uno_.
4. Open the `Arduino_Uno_CNC_Final.ino` file.
5. Click **Upload** (→).

### 4.2 — Arduino MEGA (Servo Control)

**File:** [`MotorDriver.ino`](code/Arduino/Final_Ard/MotorDriver/MotorDriver.ino) (English) or [`MotorDriver_PT.ino`](code/Arduino/Final_Ard/MotorDriver_PT/MotorDriver_PT.ino) (Portuguese)

**What it does:** Receives a data string from the Raspberry Pi (via UART), parses the values, and sends the correct PWM signals to each servo motor through the PCA9685.

**How to upload:**

1. **Install the required library:**
   - In Arduino IDE: _Sketch > Include Library > Manage Libraries_
   - Search for **"Adafruit PWM Servo Driver"** and install it.
2. Connect the **Arduino MEGA** to your PC via USB.
3. Go to _Tools > Board > Arduino Mega 2560_.
4. Open the `.ino` file and click **Upload** (→).

### How They Communicate

The two Arduinos talk to each other via **UART** (serial port). The MEGA acts as "Master" and the Uno as "Slave":

- The MEGA receives a complete data string from the Raspberry Pi.
- It extracts the base value and sends it to the Uno.
- The Uno interprets the command and moves the stepper motor.

---

## 🧠 Step 5: Software — Raspberry Pi (Computer Vision)

This is the most exciting part — the brain of the system.

**File:** [`ArmController.py`](code/RPi/Final_RPI/ArmController.py) (English) or [`ArmController_PT.py`](code/RPi/Final_RPI/ArmController_PT.py) (Portuguese)

### 5.1 — Install Dependencies

In the Raspberry Pi terminal, run:

```bash
pip install mediapipe opencv-python pyserial numpy --break-system-packages
```

> The `--break-system-packages` flag is required on Raspberry Pi OS Bookworm because it's very restrictive with pip packages by default.

### 5.2 — Physical Connection (UART)

Connect the **GPIO 14 (TX)** on the Raspberry Pi to **Pin 19 (RX1)** on the Arduino MEGA with a wire. Also connect **GND** between the two.

> **⚠️ Voltage levels:** The Raspberry Pi operates at 3.3V and the Arduino at 5V. In practice, the Arduino reads 3.3V as HIGH without issues (RPi TX → Arduino RX). If you need to send data from the Arduino to the RPi, use a voltage divider.

### 5.3 — Run

```bash
python3 ArmController.py
```

The system will:

1. Start the camera.
2. Detect your hand and shoulder using AI (MediaPipe).
3. Calculate angles and finger states.
4. Send the commands to the Arduino MEGA via UART.

### 5.4 — Headless Mode (No Monitor)

The Raspberry Pi can run without a monitor. Use the [**Raspberry Pi Connect**](https://www.raspberrypi.com/software/connect/) service to access the screen remotely from any browser. We recommend using a **5G mobile hotspot** for better speed and lower latency.

---

## 🖥️ Step 6: Software — PC (Debugging & Testing)

If you want to test the computer vision **without having the robot assembled**, you can use the PC script.

**File:** [`VisionDebugger_PC.py`](code/PC/VisionDebugger_PC.py) (English) or [`VisionDebugger_PC_PT.py`](code/PC/VisionDebugger_PC_PT.py) (Portuguese)

### 6.1 — Install Dependencies (PC)

```bash
pip install opencv-python mediapipe numpy
```

### 6.2 — Required Model Files

The PC script uses a different MediaPipe API that needs model files. These are already included in the [`code/PC/`](code/PC/) folder:

- `hand_landmarker.task`
- `pose_landmarker_lite.task`

**These files must be in the same folder as the Python script.**

### 6.3 — Run

```bash
python VisionDebugger_PC.py
```

You'll see a window with your webcam feed and the detected points (hand and body) drawn on top. The calculated values (angles, finger states, etc.) appear as text on screen, making calibration and debugging easy.

> **💡 Recommendation:** Use this tool to understand how the vision works before setting everything up on the Raspberry Pi. It's much easier to experiment and debug on a PC.

![PC debugger interface](media/pc-debugger.jpg)

---

## 🧬 How the Vision Works (Simple Explanation)

The system uses Google's **MediaPipe** framework, which contains pre-trained AI models that detect 21 hand points and 33 body points in real time.

### How does it know if a finger is open or closed?

Instead of measuring the _distance_ between the fingertip and the wrist (which changes if you move closer to or further from the camera), the system calculates the **angle** at the finger's middle joint:

- **Large angle (> 160°)** → finger extended → **Open**
- **Small angle (< 160°)** → finger bent → **Closed**

> **💡 Why angles instead of distances?** Angles don't change with your distance from the camera. If you step back, the points get closer together on screen, but the angle at the joint stays the same. This makes the system much more reliable.

### How does it control the base (left/right)?

The system analyzes your **elbow angle** (using the shoulder, elbow, and wrist points):

- Arm folded (angle < 70°) → Base rotates **Left**
- Arm extended (angle > 130°) → Base rotates **Right**
- In between → Base stays at **Center**

### How does it control the elbow (up/down)?

It compares the **height of your wrist** to your shoulder:

- Wrist far below shoulder → Elbow stays **down**
- Wrist at shoulder level → Elbow goes **up**

### How does it control wrist rotation?

Since MediaPipe doesn't provide palm rotation directly, the system compares the relative position of the **thumb** and the **pinky finger**. Depending on your arm position (extended or bent), it automatically switches between using the X or Y axis to calculate rotation, and maps the result to 0°–180°.

![MediaPipe landmarks](media/mediapipe-landmarks.jpg)

---

## 📡 Communication Protocol

The Raspberry Pi sends a formatted string to the Arduino MEGA every video frame. The structure is:

```
$<Base>,<Elbow>,<D1>,<D2>,<D3>,<D4>,<D5>,<Rotation>\n
```

| Field    | Values  | Meaning                                 |
| :------- | :-----: | :-------------------------------------- |
| Base     | 0, 1, 2 | Left, Center, Right                     |
| Elbow    |  0, 1   | Extended, Bent                          |
| D1 to D5 |  0, 1   | State of each finger (0=open, 1=closed) |
| Rotation |  0–180  | Wrist rotation servo angle              |

**Example:**

```
$1,0,1,1,1,1,1,90\n
```

> _Base at center, elbow extended, all fingers closed, wrist rotated to 90°._

The Arduino MEGA uses the `$` symbol to know where a message starts and `\n` to know where it ends. This ensures only complete messages are processed — if a message arrives cut off, it's simply ignored.

---

## 🔥 Common Problems & Solutions

| Problem                           | Likely Cause                        | Solution                                                                      |
| :-------------------------------- | :---------------------------------- | :---------------------------------------------------------------------------- |
| Parts lifting off the print bed   | Warping                             | Enable **Brim** in your slicer                                                |
| Servos jitter or don't move       | Insufficient power                  | Check that the 7V supply is connected to PCA9685 (**V+** terminal, not VCC)   |
| Arduino resets on its own         | Servos drawing current from Arduino | Use separate external power supplies; **never** power servos from the Arduino |
| Serial communication doesn't work | No shared ground                    | Connect **GND** of all devices together                                       |
| `Serial port not found` on RPi    | UART not enabled                    | Enable UART in `raspi-config` → _Interface Options > Serial Port_             |
| MediaPipe is slow on RPi          | Heavy processing                    | Verify multithreading is active (CameraStream class in the code)              |
| Hand not detected                 | Poor lighting                       | Improve ambient lighting; avoid backlight                                     |

---

## 📊 Results

- **Frame rate:** 15–20 FPS on the Raspberry Pi 5, sufficient for real-time control.
- **Latency:** Noticeable but low — suitable for telepresence applications.
- **Finger accuracy:** The angle-based method is robust and works regardless of distance to the camera or hand rotation.
- **Stability:** The UART protocol didn't drop any packets during testing — movements were smooth.

|               System Working                |                Testing Session                |
| :-----------------------------------------: | :-------------------------------------------: |
| ![System working](media/system-working.gif) | ![Testing session](media/testing-session.gif) |

### Current Limitations

- **Fingers** only have two states (open/closed) — no intermediate positions.
- **Shoulder** doesn't include the forward pitch movement — it wasn't implemented.
- **Camera** is not fixed to the robot; it can misalign if the table is bumped.
- No **haptic feedback** — the operator can't feel what the robot touches.

---

## 💡 Ideas for Improvement

If you want to take the project further, here are some suggestions:

1. **Proportional finger control:** Instead of "open" or "closed", map your real finger angle directly to the servo angle. This way the robot copies the exact position.
2. **Integrated camera mount:** Design a part that attaches the camera to the robot's base, eliminating misalignment issues.
3. **More degrees of freedom:** Add the shoulder servo (Forward Pitch) so the arm can reach objects in front of it.
4. **Haptic feedback:** Install pressure sensors on the fingertips and vibration motors in a glove, so the operator "feels" what the robot touches.
5. **Professional base mounting:** Use clamps instead of adhesives to secure the robot to the workbench.

---

## 👥 Authors

| Name                    | Contact                                 |
| :---------------------- | :-------------------------------------- |
| **Henrique Abrantes**   | [GitHub](https://github.com/Bolofofopt) |
| **Christian Rodrigues** | —                                       |
| **Rodrigo Maria**       | —                                       |

Project developed as part of the Robotics and Artificial Intelligence program at Escola Superior Náutica Infante D. Henrique (Portugal).

---

## 📚 References & Credits

- **InMoov** — [inmoov.fr](https://inmoov.fr/) — Gael Langevin's open-source project for the mechanical design of the hand and forearm.
- **MediaPipe** — [Google AI](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker) — Computer vision framework for hand and pose detection.
- **OpenCV** — [opencv.org](https://opencv.org/) — Image processing library.
- **Raspberry Pi** — [raspberrypi.com](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) — Raspberry Pi 5 documentation.
- **Arduino** — [docs.arduino.cc](https://docs.arduino.cc/hardware/mega-2560) — Arduino MEGA 2560 documentation.
- **Cirkit Designer** — [Interactive wiring diagram](https://app.cirkitdesigner.com/project/) — Full schematic.

> For the complete technical analysis (torque calculations, mathematical foundations, detailed engineering decisions), see the [full report in PDF](RIA_G7_Relatorio_Final_Braco_Robotico.pdf) included in this repository (Portuguese).

---

## 📄 License

This project is licensed under the [MIT License](LICENSE) — you are free to use, modify, and distribute it.

---

<div align="center">

**Made with ❤️, PLA, and a lot of patience.**

⭐ If this project helped you, leave a star on the repository!

</div>
