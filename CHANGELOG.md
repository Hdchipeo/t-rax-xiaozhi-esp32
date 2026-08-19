# 🦖 T-Rax ESP32 Robot: Chronological Project Summary, Features & Debugging Log

**Project Name**: T-Rax Xiaozhi ESP32 (Non-Verbal R2-D2 Droid Robot)  
**Target Hardware**: ESP32-S3 SuperMini (4MB Flash + 2MB PSRAM)  
**Repository**: `https://github.com/Hdchipeo/t-rax-xiaozhi-esp32.git`  
**Generated Date**: 2026-08-20  

---

## 1. 📜 Chronological Summary of Project Evolution & Fixes

Below is the complete chronological timeline of all bugs diagnosed, features implemented, root causes analyzed, and optimizations applied to the codebase.

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                 CHRONOLOGICAL TIMELINE                                 │
└────────────────────────────────────────────────────────────────────────────────────────┘
 [Phase 1: Driver & Crash Fixes]
   ├─ Fix #1: WS2812 RMT Thread Contention (Mutex lock + dedicated eye_led_task)
   ├─ Fix #2: DRV8833 DC Motors LEDC Hardware PWM (5kHz 8-bit PWM + Low-Pass Filter)
   ├─ Fix #3: C++ Scope Error Fix (`idle_task_handle_` syntax)
   └─ Fix #4: LoadProhibited Guru Meditation Crash (`GetLed()` nullptr check)

 [Phase 2: DSP Audio & MCP Server Protocol]
   ├─ Feature #5: Real-time R2-D2 Real-Time PCM Sound Synthesizer (DSP Frequency Sweeps)
   ├─ Fix #6: MCP Tool Endless Loop & Raw JSON Text Readout Suppressor
   ├─ Fix #7: AudioService Compilation Error (`IsIdle()` replacement)
   └─ Feature #8: Driver Enable GPIO 7 HIGH Activation

 [Phase 3: Echo Feedback & Turn-Taking Latency]
   ├─ Fix #9: Acoustic Feedback Self-Triggering Loop Prevention (Post-playback DMA flush)
   └─ Optimization #10: Hardware Rate-Limiting Debounce (1200ms) & Ultra-Fast <800ms Latency

 [Phase 4: Biomimetic Kinematics & 28 Autonomous Scenarios]
   ├─ Feature #11: 40 FPS Biomimetic Organic Servo Kinematics Engine (Dual-Harmonic Breathing)
   └─ Feature #12: Complete 28 Multi-Sensory Choreography Scenarios (Short & 5-10s Narrative)
```

---

### Phase 1: Hardware Driver Fixes & Panic Crash Resolution

#### 1. WS2812 RMT LED Strip Thread Contention Error
- **Log Error**:
  ```text
  E (69844) rmt: rmt_tx_enable(768): channel not in init state
  E (69844) led_strip_rmt: led_strip_rmt_refresh(87): enable RMT channel failed
  ```
- **Root Cause Analysis**: Both `OrganicMoveHead` and `StartEyeLedBreathingTask` were calling `led_strip_refresh()` concurrently on the single ESP32 RMT hardware channel, causing driver state corruption.
- **Fix**: Wrapped all WS2812 operations in `std::mutex led_mutex_` and isolated `led_strip_refresh()` strictly inside `eye_led_task`.

#### 2. DRV8833 DC Motor LEDC Hardware PWM Smoothing
- **Problem**: Digital On/Off GPIO toggling caused harsh motor jerkiness, noise, and mechanical stress.
- **Fix**: Implemented 5kHz 8-bit LEDC Hardware PWM (Timer 1, Channels 2, 3, 4, 5) with a digital Low-Pass Filter (`SmoothDriveMotors(target_left, target_right, duration_ms, alpha)`).

#### 3. C++ Syntax Scope Resolution Error
- **Log Error**: `error: 'board' was not declared in this scope` in `t_rax_board.cc:665`.
- **Fix**: Replaced `&board->idle_task_handle_` with `&idle_task_handle_`.

#### 4. LoadProhibited Guru Meditation Crash
- **Log Error**:
  ```text
  Guru Meditation Error: Core 0 panic'ed (LoadProhibited). Exception was unhandled.
  PC : 0x4201675d Application::HandleStateChangedEvent() at application.cc:927
  EXCVADDR : 0x00000000
  ```
- **Root Cause Analysis**: `TRaxBoard::GetLed()` returns `nullptr` because T-Rax uses a WS2812 RMT strip instead of a single LED object. `application.cc` was dereferencing `GetLed()` without checking for null.
- **Fix**: Added safe `if (led != nullptr)` checks in `main/application.cc` lines 246 & 926.

---

### Phase 2: Audio Synthesis & Protocol Filtering

#### 5. Real-Time R2-D2 Real-Time PCM Audio Synthesizer
- **Feature**: Built a mathematical DSP frequency sweep generator `GenerateFrequencySweepPCM()` that synthesizes 24kHz 16-bit PCM audio on-the-fly and sends PCM buffers directly to MAX98357A I2S audio codec via `codec->OutputData()`.

#### 6. MCP Tool Call Endless Loop & Text Readout Suppressor
- **Problem**: Robot spoke raw JSON tool strings (e.g. `{"state_id": 21}`) via TTS, and LLM server entered an infinite tool-calling loop.
- **Fix**:
  1. Added string prefix filter in `main/application.cc` to suppress raw `% self.trax.` and `{"state` strings from TTS/Display.
  2. Modified `self.trax.set_state` return string to explicitly tell LLM: `"State X applied. Do NOT call this tool again. STOP."`.
  3. Updated `SYSTEM_PROMPT.md` with strict single-call non-verbal directives.

#### 7. `AudioService` Compilation Error Fix
- **Log Error**: `error: 'class AudioService' has no member named 'IsPlaying'`.
- **Fix**: Replaced `!audio_service_.IsPlaying()` with `audio_service_.IsIdle()` in `main/application.cc`.

#### 8. Driver Enable GPIO 7 Output HIGH Activation
- **Fix**: Added `InitializeDriverEnable()` in `t_rax_board.cc` constructor to drive `DRIVER_ENABLE_GPIO` (GPIO 7) HIGH.

---

### Phase 3: Echo Feedback & Turn-Taking Latency Optimization

#### 9. Self-Triggering Acoustic Echo Feedback Loop (`State 21` Repeating Every 1.5s)
- **Log Pattern**:
  ```text
  I (49344) Application: >> Hi,Joy
  I (50044) TRaxBoard: ========== Organic Sequencer State: 21 ==========
  I (50544) TRaxBoard: R2D2 Audio Synthesizer: Playing [BOOT_POWER_UP]
  I (50674) StateMachine: State: speaking -> listening  <-- MIC OPENED WHILE SPEAKER WAS PLAYING!
  I (51674) TRaxBoard: ========== Organic Sequencer State: 21 ========== <-- MIC RE-TRIGGERED ITSELF!
  ```
- **Root Cause Analysis**: The microphone opened immediately while the MAX98357A speaker was playing `BOOT_POWER_UP`. The mic picked up its own R2-D2 sound, triggering another turn.
- **Fix**:
  1. Added a 200ms post-playback DMA flush delay in `PlayR2D2Chirp()`.
  2. Added a 150ms acoustic decay delay before transitioning `kDeviceStateSpeaking -> kDeviceStateListening` in `application.cc`.

#### 10. Hardware Rate-Limiting Debounce & Ultra-Fast <800ms Latency
- **Problem**: Long 10-second debounce made the robot feel unresponsive to user speech.
- **Fix**:
  1. Reduced `STATE_DEBOUNCE_MS` to **1200ms** (1.2s): blocks all 10-15 duplicate tool calls in a single turn burst, while resetting immediately for the next user speech.
  2. Reduced post-MCP transition delay to **150ms**, enabling ultra-fast real-time response times (< 800ms total turnaround).

---

### Phase 4: Biomimetic Servo Kinematics & 28 Autonomous Scenarios

#### 11. Biomimetic Living Organism Servo Kinematics Engine
- **Feature**:
  - **40 FPS Dual-Harmonic Breathing**: Tilt ($\pm 8.0^\circ$) and Pan ($\pm 6.0^\circ$) continuous organic sinusoids.
  - **Attentive Listening Posture**: Head glides up to $100^\circ - 110^\circ$ Tilt with attentive breathing during `kDeviceStateListening`.
  - **Natural Gaze Saccade Planner**: Random curious head shifts every 3–6s.
  - **Seamless $C^1$ Trajectory Transitions**: `OrganicMoveHead()` interpolates from current physical angles using `CubicEaseInOut` with zero motion snap or jerk. Thread-safe using `servo_mutex_` and `is_performing_action_`.

#### 12. Complete Library of 28 Autonomous Scenarios (0 to 27)

| # | Scenario Name | Duration | Description & Choreography |
|---|---|---|---|
| **0** | **Sniff & Explore** | 2.5s | Sniff left $\to$ right $\to$ look up proud with curious whistle. |
| **1** | **Playful Wiggle Dance** | 2.0s | Chassis wiggle left-right + head rhythm + happy arpeggio. |
| **2** | **Sneaky Dino Prowl** | 2.5s | Low stalking head + forward creep + sharp left/right scans. |
| **3** | **Curious Bird Tilt** | 2.2s | Snappy cocked head tilt with `BackEaseOut` + confused question chirp. |
| **4** | **Surprise Check Behind** | 1.8s | Rapid full-range look back left ($145^\circ$) and right ($35^\circ$). |
| **5** | **Lazy Stretch & Yawn** | 2.5s | Slow upward stretch ($140^\circ$) + lazy sigh chirp + relax down. |
| **6** | **Victory Spin & Nod** | 2.0s | $360^\circ$ fast pivot spin + head nods + fanfare chirp. |
| **7** | **Affectionate Nudge** | 1.8s | Step forward nudge + chin up + loving purr chirp. |
| **8** | **Startled Reflex** | 1.8s | Jump back + head snap back + red strobe + alarm scream. |
| **9** | **Sleepy Low Battery** | 2.5s | Head droops slowly + nod awake + pitch drop chirp. |
| **10** | **Curious Bug Hunt** | 2.2s | Nose to ground + zig-zag scan + micro forward steps. |
| **11** | **Dizzy Confused** | 2.2s | Lurching head roll + unsteady backward drift + dizzy whimper. |
| **12** | **Proud Superhero** | 2.2s | Bold forward stride + head high pose + hero triumph fanfare. |
| **13** | **Long-Range Scan** | 2.8s | Head high + slow $120^\circ$ panoramic radar sweep. |
| **14** | **Narrow Gap Probe** | 2.2s | Cautious step forward + peer low left/right + hesitant step back. |
| **15** | **Wall Tracker** | 2.0s | Head turned sideways to listen + parallel wall curve. |
| **16** | **Ceiling Recon** | 2.5s | Look straight up at ceiling ($140^\circ$) + slow 360 pivot turn. |
| **17** | **Trail Hunter** | 2.0s | Nose down to ground + tracking clicks + zig-zag motor drive. |
| **18** | **Peek-a-Boo Peek** | 2.2s | Hide low sideways $\to$ snap head out $\to$ quick retreat + giggle. |
| **19** | **Prank Scare & Laugh** | 2.5s | Fake alarm surprise jump back $\to$ switch to bright yellow laugh. |
| **20** | **Fake Charge & Escape** | 2.2s | Low head charge rush $\to$ 180° spin panic escape. |
| **21** | **Stubborn Refusal** | 2.2s | Rapid head shake + turn face away + wheel twitch + raspberry sound. |
| **22** | **Tail Chasing Craze** | 2.2s | Look back at tail $\to$ high-speed 360° spin chasing tail + barks. |
| **23** | **Alien Contact Ritual** | **7.5s** | High cosmos head raise + 360° antenna alignment + space scan + bow. |
| **24** | **The Great Mosquito Battle** | **8.0s** | Fast bug tracking + missed bite + 180° chase + victorious bite & wiggle. |
| **25** | **Archaeologist Fossil Dig** | **8.5s** | Sniff floor + alternating wheel digging + blow dust + fossil discovery. |
| **26** | **Thunderstorm Terror & Courage** | **9.0s** | Thunder shock + panic reverse + trembling in fear + inner courage roar. |
| **27** | **Robot System Reboot & Diagnostic** | **9.5s** | Power limp drop + charging hum + Pan axis sweep + wheel drive test + ready. |

---

## 2. 🏗️ Hardware Architecture & Pin Map

```
                  ┌───────────────────────────────────────────────┐
                  │          ESP32-S3 SuperMini MCU               │
                  └──────┬──────────┬──────────┬──────────┬───────┘
                         │          │          │          │
         ┌───────────────┘          │          │          └───────────────┐
         │                          │          │                          │
┌────────▼────────┐        ┌────────▼───────┐ ┌▼───────────────┐ ┌────────▼────────┐
│ MAX98357A Amp   │        │ INMP441 Mic    │ │ DRV8833 Motor  │ │ WS2812 RGB LED │
│ GPIO 4 (BCLK)   │        │ GPIO 1 (SCK)   │ │ GPIO 7 (ENABLE)│ │ GPIO 48 (RMT)  │
│ GPIO 5 (LRCK)   │        │ GPIO 2 (WS)    │ │ GPIO 10,11 (L) │ └────────────────┘
│ GPIO 6 (DOUT)   │        │ GPIO 3 (DIN)   │ │ GPIO 12,13 (R) │
└─────────────────┘        └────────────────┘ └────────────────┘
                                                       │
                                              ┌────────▼────────┐
                                              │ Servos (LEDC 0) │
                                              │ GPIO 8 (Pan)    │
                                              │ GPIO 9 (Tilt)   │
                                              └─────────────────┘
```

---

## 3. 📂 Summary of Modified Files

1. **[`main/boards/t-rax/t_rax_board.cc`](file:///Users/dangminhtam/.gemini/antigravity-ide/scratch/t-rax-xiaozhi-esp32/main/boards/t-rax/t_rax_board.cc)**:
   - Contains complete kinematic engine, 28 scenarios, R2-D2 DSP sound synthesizers, LEDC hardware PWM motor smoothing, WS2812 RMT mutex protection, and rate-limiting debounce.
2. **[`main/application.cc`](file:///Users/dangminhtam/.gemini/antigravity-ide/scratch/t-rax-xiaozhi-esp32/main/application.cc)**:
   - Added `GetLed()` nullptr crash guards, raw MCP string suppressors for display/TTS, and 150ms post-MCP acoustic echo decay transition scheduler.
3. **[`main/boards/t-rax/config.h`](file:///Users/dangminhtam/.gemini/antigravity-ide/scratch/t-rax-xiaozhi-esp32/main/boards/t-rax/config.h)**:
   - Defined `#define DRIVER_ENABLE_GPIO GPIO_NUM_7`.
4. **[`main/boards/t-rax/SYSTEM_PROMPT.md`](file:///Users/dangminhtam/.gemini/antigravity-ide/scratch/t-rax-xiaozhi-esp32/main/boards/t-rax/SYSTEM_PROMPT.md)**:
   - Updated system prompt directives for single-call non-verbal tool execution.

---

## 4. 🚀 Build & Flash Instruction

To build and flash the firmware to the T-Rax board:
```bash
python3 scripts/build.py t-rax
```
