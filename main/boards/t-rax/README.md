# Robot T-Rax - Custom Board & Organic Non-Verbal Droid Architecture

<p align="center">
  <img src="../../../docs/assets/images/t_rax_3d_front_right.png" width="45%" alt="T-Rax Front Right" />
  <img src="../../../docs/assets/images/t_rax_3d_front.png" width="45%" alt="T-Rax Front View" />
</p>
<p align="center">
  <img src="../../../docs/assets/images/t_rax_3d_front_left.png" width="45%" alt="T-Rax Front Left" />
  <img src="../../../docs/assets/images/t_rax_3d_back.png" width="45%" alt="T-Rax Back View" />
</p>

Custom firmware implementation for the **T-Rax Droid Architecture** built upon the **XiaoZhi AI Voice Assistant Framework (ESP32-S3)**.

> **Architecture Overview**: T-Rax operates entirely as a **non-verbal expressional droid**. It replaces human speech output with synthesized R2-D2 acoustic chirps and sweeps, real-time bio-inspired kinematic trajectory planning (easing curves), WS2812 RGB eye status indications, and active collision avoidance via a Time-of-Flight (VL53L0X) laser distance sensor.

---

## 1. Hardware Architecture

* **Microcontroller**: ESP32-S3 SuperMini (4MB Flash, 2MB Quad PSRAM).
* **Audio Subsystem**:
  * Microphone: INMP441 I2S MEMS microphone.
  * Audio Amplifier: MAX98357A I2S DAC/Amplifier (synthesizes R2-D2 acoustic waveforms directly via I2S DMA streaming).
* **Kinematic Actuation**:
  * 2-Axis Servo Neck Assembly (Pan/Tilt - Yaw/Pitch).
  * Dual DC Track Drive Motors controlled via DRV8833 H-Bridge Dual Motor Driver.
* **Range Finding**: VL53L0X Laser Time-of-Flight Distance Sensor (I2C Master Bus).
* **Visual Expressional Status**: WS2812 Single RGB Addressable LED (RMT Peripheral Controller).
* **Display Interface**: Headless configuration (`NoDisplay`).

---

## 2. Pinout Matrix

| Subsystem | Function | Hardware Pin | ESP32-S3 GPIO | Interface Type |
| :--- | :--- | :--- | :--- | :--- |
| **INMP441 Microphone** | Bit Clock | `SCK` | **`GPIO 4`** | I2S0 Input |
| | Word Select | `WS` | **`GPIO 5`** | I2S0 Input |
| | Data Output | `SD` | **`GPIO 6`** | I2S0 Input |
| **MAX98357A DAC/Amp** | Bit Clock | `BCLK` | **`GPIO 7`** | I2S1 Output |
| | Left/Right Clock | `LRCK` | **`GPIO 15`** | I2S1 Output |
| | Data Input | `DIN` | **`GPIO 16`** | I2S1 Output |
| **VL53L0X Distance Sensor** | I2C Data | `SDA` | **`GPIO 8`** | I2C Master Bus |
| | I2C Clock | `SCL` | **`GPIO 9`** | I2C Master Bus |
| **DRV8833 Motor Driver** | Left Motor A | `AIN1` | **`GPIO 10`** | LEDC Hardware PWM |
| | Left Motor B | `AIN2` | **`GPIO 11`** | LEDC Hardware PWM |
| | Right Motor A | `BIN1` | **`GPIO 12`** | LEDC Hardware PWM |
| | Right Motor B | `BIN2` | **`GPIO 13`** | LEDC Hardware PWM |
| **Pan/Tilt Neck Servos** | Pan Axis | `Pan PWM` | **`GPIO 14`** | LEDC Hardware PWM (Servo 1) |
| | Tilt Axis | `Tilt PWM` | **`GPIO 17`** | LEDC Hardware PWM (Servo 2) |
| **WS2812 RGB Eye LED** | Data Input | `DIN` | **`GPIO 48`** | RMT Hardware Channel |
| **User Input** | Boot / Config | `BOOT` | **`GPIO 0`** | GPIO Input Button |

---

## 3. Organic Motion Engine

### 3.1. Non-linear Trajectory Easing
Servo head trajectories utilize organic non-linear easing functions for smooth bio-inspired motion:
- **Cubic Ease-In-Out**: Acceleration and deceleration profiles for smooth head shifts.
  $$f(t) = \begin{cases} 4t^3 & \text{if } t < 0.5 \\ 1 - \frac{(-2t + 2)^3}{2} & \text{if } t \ge 0.5 \end{cases}$$
- **Back Ease-Out**: Overshoot trajectory ($1\text{--}2^\circ$) with elastic recovery for startled reactions.

### 3.2. Micro-Jitter Breathing Simulation
When in IDLE state, the neck servos maintain continuous bio-mimetic micro-oscillations ($0.5^\circ - 1.2^\circ$ amplitude):
$$\theta_{pan}(t) = 90^\circ + 1.2^\circ \cdot \sin(0.4t) + 0.6^\circ \cdot \cos(0.2t)$$
$$\theta_{tilt}(t) = 90^\circ + 0.8^\circ \cdot \sin(0.3t) + 0.4^\circ \cdot \cos(0.5t)$$

### 3.3. PWM Low-Pass Filtering for Track Drive Motors
Low-pass filter smooths motor torque transitions and eliminates current spikes on the DRV8833 driver:
$$PWM_{out}(k) = PWM_{out}(k-1) + \alpha \cdot \big(PWM_{target} - PWM_{out}(k-1)\big) \quad (\alpha = 0.15)$$

---

## 4. Emotional State & Behaviour Matrix (28 Expressional Scenarios)

1. **Curious** (Cyan Breathing, Pan=120, Tilt=110, Whistle)
2. **Focused** (Green Solid, Pan=90, Tilt=100, Beep)
3. **Alert Warning** (Orange Strobe, Pan=90, Tilt=130, Alert Sweep)
4. **Angry** (Red Strobe, Motor Wiggle, Buzz)
5. **Scared** (Purple Strobe, Motor Reverse 30cm, Scream)
6. **Happy** (Green Breathing, Motor Wiggle, Arpeggio)
7. **Disappointed** (Dark Blue, Head Droop, Sad Slide Down)
8. **Target Detected** (Yellow Solid, Target Lock Beep)
9. **Confused** (Magenta Breathing, Tilt Head 45°, Question Chirp)
10. **Surprised** (White Strobe, High Chirp)
11. **Suspicious** (Amber Breathing, Side Glare, Low Chirp)
12. **Loving** (Pink Breathing, Loving Purr)
13. **Victorious** (Cyan Strobe, Motor Wiggle, Fanfare)
14. **Shy** (Light Pink, Turn Head Away, Whimper)
15. **Bored** (Dim Grey, Tilting Droop, Sigh)
16. **Arrogant** (Gold Solid, Upward Tilt, Proud Tune)
17. **Searching** (Blue Strobe, Pan Scan 45-135°, Radar Sweep)
18. **System Fault** (Red Strobe, Glitch Noise)
19. **Low Battery** (Dim Red Breathing, Head Droop, Low Power Beep)
20. **Charging** (Green Breathing, Charging Hum)
21. **Booting** (White Breathing, Power Up)
22. **Sleeping** (LED Off, Head Droop 20°)
23. **Idle** (Autonomous micro-gesture sequence every 5-12s)
24. **Alien Contact Ritual** (~7.5s Story Scenario)
25. **The Great Mosquito Battle** (~8.0s Story Scenario)
26. **Archaeologist Fossil Dig** (~8.5s Story Scenario)
27. **Thunderstorm Terror & Courage** (~9.0s Story Scenario)
28. **Robot System Reboot & Diagnostic** (~9.5s Story Scenario)

---

## 5. Collision Avoidance Subsystem (VL53L0X Laser ToF Sensor)

When distance measurements fall below $150\text{mm}$ ($15\text{cm}$):
1. **Emergency Reflex**:
   - Drive motors initiate a 400ms emergency reverse maneuver.
   - Neck servos raise tilt angle (`Tilt=140°`).
   - WS2812 RGB Eye LED triggers high-frequency Strobe mode (Amber/White).
   - Audio synthesizer plays an emergency startled chirp.
2. **Cooldown Protection**: 2.0-second lockout prevents reflexive oscillation loops.

---

## 6. Build & Flash Instructions

### Build Firmware:
```bash
python3 scripts/build.py t-rax
```

### Flash to Target Hardware (ESP32-S3 SuperMini):
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## 7. Device-Side MCP Tool Catalog (`self.trax.*`)

Robot T-Rax exposes **19+ native MCP tools** for LLM voice control:

| Tool Namespace | Functionality |
| :--- | :--- |
| **`self.trax.wave_greeting`** | Friendly greeting posture (double nod + micro step + mint strobe + power-up chirp). |
| **`self.trax.listen_attentively`** | Attentive listening gesture (head cocked + emerald solid eyes + focused beep). |
| **`self.trax.think_ponder`** | Pondering thinking gesture (head tilt up-left + amber breathing eyes + question chirp). |
| **`self.trax.express_empathy`** | Comforting empathy posture (gentle bow + step closer + rose pink eyes + purr chirp). |
| **`self.trax.express_excitement`** | Excited celebration (head pop + 2 track wiggles + golden strobe + happy chirp). |
| **`self.trax.nod_enthusiastic`** | Fast double nod + step forward + happy chirp. |
| **`self.trax.nod_respectful`** | Slow bow nod + micro retreat step + cyan breathing eyes. |
| **`self.trax.shake_emphatic`** | Defiant head shake + defensive retreat step + angry buzz. |
| **`self.trax.shake_confused`** | Tilted head shake + track hip wiggle + question chirp. |
| **`self.trax.move_forward`** / **`move_backward`** | Drive forward or backward (`200ms` – `2000ms`). |
| **`self.trax.turn_left`** / **`turn_right`** | Rotate on tracks (`200ms` – `1500ms`). |
| **`self.trax.look_around`** | 360-degree environment scan (Left 45° → Right 135° → Up → Center). |
| **`self.trax.set_head_position`** | Direct head Pan/Tilt angle control (`Pan: 45..135°`, `Tilt: 30..140°`). |
| **`self.trax.set_eye_color`** | Custom RGB WS2812 eye LED color & mode (`Breathing`, `Solid`, `Strobe`, `Off`). |
| **`self.trax.dance`** | Victory celebration wiggle dance. |
| **`self.trax.stop_all`** | Emergency halt all motor movements. |
| **`self.trax.play_sound`** | Synthesizes R2D2 acoustic chirps (`1`=Happy, `2`=Angry, `3`=Confused, `4`=Alert, `5`=Scared, `6`=Surprised, `7`=Triumph). |
| **`self.trax.set_state`** | Triggers composite expressional state macros (1..28). |

---

## 8. System Prompt Configuration

For complete R2-D2 Droid personality configuration and non-verbal tool execution rules, see:
👉 **[SYSTEM_PROMPT.md](SYSTEM_PROMPT.md)**

