### 1. Brief Device Description

The device is a compact barometric variometer with an e‑ink display and haptic (vibratory) feedback, optimized for use with paragliders, hang gliders, sailplanes, and similar applications.

Its core functions are:

- Measurement of **barometric altitude** and **vertical speed (variometer)**;
- Display of:
  - Flight time (stopwatch),
  - Current and relative altitude (with respect to launch point),
  - Maximum climb and sink rate attained during the flight,
  - Ambient temperature,
  - Battery voltage and state-of-charge,
  - Current time and date (from RTC);
- Haptic feedback for climb/sink detection (enabling hands‑free situational awareness);
- Low-power operation: implemented via e‑ink display, ESP32 hardware deep sleep, sensor and Wi‑Fi power gating.

---

### 2. Operational Principles (Software Perspective)

#### 2.1. Hardware Architecture (as per firmware)

- **MCU**: ESP32.
- **Barometric sensor**: BMP3XX (I²C address 0x76 or 0x77).
- **Accelerometer**: BMA423 (I²C address 0x18).
- **Real‑Time Clock (RTC)**: I²C address 0x51 (e.g., PCF8563 or equivalent).
- **Display**: 1.54" monochrome e‑paper GDEH0154D67 (200×200 px, driven by GxEPD2 library).
- **Vibrator motor**: connected to GPIO `PIN_VIBRO = 13`.
- **User buttons**:
  - `BTN_SELECT = 35` — launch calibration / start flight,
  - `BTN_RIGHT = 4` — stop flight / auxiliary function in clock mode,
  - `BTN_BACK = 25` — wake-up / enter deep sleep (on/off).
- **Battery monitoring**: analog ADC input `PIN_BATT = 34`, using a voltage divider (×2 attenuation).

---

#### 2.2. Operating Modes (States)

The firmware implements the following logical states:

- **`SLEEP`**: Deep-sleep mode  
  - Power to the variometer circuitry and vibrator is disabled,  
  - Display is placed in hibernate mode,  
  - I²C bus and Wi‑Fi are powered down,  
  - ESP32 enters hardware deep sleep and wakes **only** upon `BTN_BACK` press (rising edge).

- **`IDLE`**: Clock / standby mode  
  - Battery voltage, current time, and date are shown on the e‑ink display,  
  - After 60 seconds of inactivity, the device automatically transitions to `SLEEP`,  
  - From here, a new flight can be initiated (`SELECT`) or the RTC time can be zeroed (`RIGHT`, for diagnostics only).

- **`RUNNING`**: Active flight mode  
  - Sensors are powered and actively sampled,  
  - A dedicated background task runs at ~50 Hz,  
  - Updates include: altitude, vertical speed, haptic feedback, flight timer, and max/min climb/sink records.

- **`STOPPED`**: Post‑flight state  
  - Sensors remain initialized but data acquisition is paused,  
  - Haptic output is disabled,  
  - Display shows frozen flight statistics,  
  - From here one may either:
    - Begin a new flight (`SELECT`), or
    - Enter `SLEEP` (`BACK`).

---

#### 2.3. Measurement Pipeline and Algorithms

##### 2.3.1. Barometric Altitude

- BMP3XX initialization settings:
  - Temperature oversampling: ×4,
  - Pressure oversampling: ×8,
  - IIR filter coefficient: 3,
  - Output data rate: 50 Hz.
- Each processing cycle:
  ```cpp
  bmp.performReading();
  altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  ```
  where `SEALEVELPRESSURE_HPA = 1012.5` hPa (fixed reference QNH).

Thus, the device reports **barometric altitude relative to a fixed sea‑level pressure**. While absolute “Sea, m” may deviate from true local QNH, relative altitude and variometer outputs remain accurate for flight purposes.

---

##### 2.3.2. Accelerometer and Tilt Compensation

Raw accelerometer data (±4 g range) are converted to units of *g*:
```cpp
ax = rx / 8192.0f; // same for ay, az
```

Tilt‑independent vertical acceleration is derived as follows:

1. Compute the magnitude of the 3‑axis acceleration vector:
   ```cpp
   acc_mag = sqrt(ax*ax + ay*ay + az*az);
   ```
   This value is invariant to device orientation (≈ 1 g at rest).

2. During calibration, a reference magnitude `gMagRef` (device‑specific 1 g baseline) is stored.

3. In flight, vertical linear acceleration (in *g*) is estimated via:
   ```cpp
   acc_linear_g = acc_mag - data.gMagRef;
   ```
   - Positive values indicate >1 g (e.g., active climb),
   - Negative values indicate <1 g (e.g., sink or stall).

4. Conversion to SI units:
   ```cpp
   acc_vert_input = acc_linear_g * 9.80665; // m/s²
   ```

> **Note**: During vibrator activation, accelerometer data are **temporarily ignored** to avoid motion artifacts. A 250 ms cooldown period follows deactivation before accelerometer data are trusted again.

---

##### 2.3.3. Kalman Filter: Fusion of Barometric and Inertial Data

A three‑state `VarioKalman` filter estimates:
- *z* = altitude (m),  
- *vz* = vertical speed (m/s),  
- *bias_z* = accelerometer bias (m/s²).

Inputs per cycle:
- `acc_vert_input` (vertical acceleration from accelerometer),
- `baro_alt` (barometric altitude),
- `dt` (time interval, derived from `micros()`).

Filter operation:
- **Prediction step**: integrates acceleration to update velocity and altitude,
- **Correction step**: adjusts state estimates using barometric altitude to suppress long‑term drift and resolve sign ambiguity,
- Bias modeling: includes a slow-varying bias term for improved robustness.

Sensitivity tuning is performed via two configurable parameters:
- `SENS_BARO_NOISE` — smaller values increase trust in barometer (higher variometer sensitivity, increased noise),
- `SENS_ACCEL_TRUST` — larger values increase reliance on accelerometer dynamics (faster response to transients, less smoothing).

Filter outputs:
- `kalman.getAltitude()` → `data.alt` (smoothed altitude),
- `kalman.getVario()` → `data.vel` (vertical speed, limited to ±25 m/s for outlier rejection).

---

##### 2.3.4. Pre‑Flight Calibration

Triggered by pressing `SELECT` in `IDLE`/`STOPPED`:

1. Sensor power is enabled (`PIN_VARIO_EN = 1`), BMP3XX and BMA423 are initialized.
2. Display shows **“Calibrating…”**.
3. Over ~1 second:
   - 100 accelerometer samples are acquired,
   - Magnitude of each sample is computed,
   - Mean of magnitudes yields `data.gMagRef`,
   - Kalman filter is updated several times with zero acceleration to stabilize altitude estimate.
4. Safeguard: if `gMagRef` falls outside [0.5 g, 1.5 g], fallback value `1.0 g` is used.

Subsequent actions:
- `startAlt` is set to the current smoothed altitude,
- Max/min variometer records are reset,
- Stopwatch starts from zero,
- Background task `varioTask` is launched (if not already active),
- State transitions to `RUNNING`.

**Conclusion**: Calibration must be performed with the device stationary and free of vibration.

---

##### 2.3.5. Flight‑Mode Display (`drawMain()`)

Screen refreshes once per second and shows:

- **Top line**:
  - *Flight timer* (hh:mm:ss since launch, cumulative across pauses),
  - *Ambient temperature* (°C, one decimal),
  - *Battery state*:  
    - Voltage measured via ADC (accounting for ×2 divider),  
    - Linear mapping: 3.3 V → 0 %, 4.2 V → 100 %.

- **Middle section** (in `RUNNING`/`STOPPED`):
  - Header: `Start, m   Sea, m`
  - Left: **Relative altitude** (current altitude − launch altitude, signed),
  - Right: **Barometric altitude above sea level** (integer m).

- **Bottom section**:
  - Label `"Vario"`,
  - (After 5 s into flight, `data.track == true`):  
    `max+X.X min−Y.Y` — peak climb and sink rates to date,
  - Large font: **Current vertical speed** (m/s, one decimal, signed ±).

---

#### 2.4. Haptic Feedback Logic

Implemented in `varioTask` (~50 Hz loop), haptic output is derived from `data.vel`:

- **Strong sink** (`v ≤ SINK_TRH = −5.0 m/s`): continuous vibration (warning).
- **Climb**: discrete pulse patterns based on speed thresholds:

  | Speed Range (m/s) | Pulses per Series |
  |-------------------|-------------------|
  | 0.2 – 0.4         | 1                 |
  | 0.4 – 0.8         | 2                 |
  | 0.8 – 1.4         | 3                 |
  | 1.4 – 2.0         | 4                 |
  | > 2.0             | 0 (no vibration in current firmware) |

  Pulse parameters:
  - Pulse width: `V_PULSE = 50 ms`,
  - Inter‑pulse gap: `V_GAP = 125 ms`,
  - Inter‑series interval: `V_PAUSE = 1000 ms`,
  - Post‑vibration cooldown (before re‑enabling accelerometer): `VIBRO_COOLDOWN_MS = 250 ms`.

- **Weak motion** (`|v| < 0.2 m/s`): no haptic output.

---

#### 2.5. Power Management and Energy Saving

- In `goDeepSleep()`:
  - Variometer task is deleted,
  - Sensor and vibrator power (`PIN_VARIO_EN`) is disabled,
  - Display hibernated,
  - I²C bus released,
  - Wi‑Fi fully deinitialized,
  - Wake‑up configured on `BTN_BACK`,
  - `esp_deep_sleep_start()` invoked.

- In `IDLE` (clock mode):
  - Variometer sensors remain powered off until flight start,
  - e‑ink updates only on state transitions,
  - 60 s inactivity → automatic transition to `SLEEP`.

---

### 3. Pilot’s Operating Instructions

#### 3.1. Controls and Indications (Summary)

| Button   | Function(s)                                                                                     |
|----------|--------------------------------------------------------------------------------------------------|
| **BACK** | - Power on/off (wake from `SLEEP`),<br>- From any state: enter `IDLE` → auto‑sleep after 60 s |
| **SELECT**| - In `IDLE`/`STOPPED`: initiate calibration & start flight (`RUNNING`)                         |
| **RIGHT**| - In `RUNNING`: stop flight → `STOPPED`,<br>- In `IDLE`: reset RTC to 00:00 (diagnostic only) |

**Flight Screen Layout**:
- **Top line**: flight timer, temperature, battery %  
- **Middle**: `Start, m` (relative altitude), `Sea, m` (barometric altitude)  
- **Bottom**: `"Vario"` label, max/min rates, large current vertical speed (±m/s)

---

#### 3.2. Pre‑Flight Preparation

1. **Battery check**:  
   - In `IDLE`, voltage and SoC are displayed;  
   - ≥ 4.2 V ≈ 100 %, ≤ 3.3 V ≈ critically low.

2. **Power on**: Press **BACK** → device wakes and shows clock screen.

3. **Mounting**: Secure the unit with minimal vibration (e.g., in harness pocket or on wrist); ensure you can feel vibrations clearly.

---

#### 3.3. Launch and Calibration

1. With clock screen active, press **SELECT**.  
2. **“Calibrating…”** appears — *do not move the device* (~1–2 s).  
   - Barometric altitude is sampled,  
   - Accelerometer is calibrated to local 1 g.  
3. Upon completion, the display switches to flight mode (`RUNNING`):  
   - Stopwatch starts,  
   - `Start, m` = 0 at current altitude.

---

#### 3.4. In‑Flight Operation

**Key indicators**:
- **Timer**: top-left,
- **Temperature & battery**: top-center/right,
- **Relative altitude (`Start, m`)**: positive → above launch, negative → below,
- **`Sea, m`**: altitude above fixed sea‑level reference (may differ from true QNH),
- **Variometer**: large value (± m/s), with recorded max/min climb/sink.

**Haptic cues**:
- **Continuous vibration** ⇒ strong sink (> 5 m/s), indicating urgent descent.
- **Pulsed vibration** ⇒ climb:
  - 1 pulse: weak lift (0.2–0.4 m/s),
  - 2 pulses: moderate (0.4–0.8 m/s),
  - 3 pulses: strong (0.8–1.4 m/s),
  - 4 pulses: very strong (1.4–2.0 m/s),
  - > 2.0 m/s: no vibration (screen still shows value).
- **No vibration** ⇒ near‑neutral vertical movement (< 0.2 m/s) *or* very strong lift (> 2 m/s), *or* device not in `RUNNING`.

Thus, pilots can primarily rely on haptics: light taps for weak thermals, longer bursts for stronger lift, and sustained vibration for dangerous sink.

---

#### 3.5. Flight Termination and Shutdown

1. **End of flight**: Press **RIGHT** → `RUNNING` → `STOPPED`:  
   - Timer freezes,  
   - Haptics disabled,  
   - Final statistics retained on screen.

2. **Power off**: Press **BACK** → device shows clock screen briefly, then enters `SLEEP`.  
   - Alternatively, remain idle in `IDLE` for ~60 s → automatic deep sleep.

Next power‑on (press **BACK**) returns to clock mode, ready for a new calibration and flight.

---

#### 3.6. Critical Notes

- **Calibration must be static**. Motion during “Calibrating…” corrupts `gMagRef`, leading to biased variometer.
- **Barometric offset**: `Sea, m` uses fixed QNH = 1012.5 hPa; absolute altitude may be offset, but relative and vertical speed remain correct.
- **Vibrator–accelerometer interference**: The firmware intentionally disables accelerometer during vibration to preserve measurement integrity.
- **Sensitivity tuning**: `SENS_BARO_NOISE` and `SENS_ACCEL_TRUST` require firmware recompilation; pilots cannot adjust them on‑device.

--- 

