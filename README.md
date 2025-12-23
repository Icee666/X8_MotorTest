# X8 Motor Test Bench – ArduPilot + Mission Planner

This repository contains a **Lua script** running on the flight controller and a **Python script** running in Mission Planner to automate and monitor endurance / validation tests on an **X8 multicopter powertrain**.

The setup has been designed and tested with:

- Flight controller: **CUAV X7 Pro**
- Firmware: **ArduCopter 4.6.x**
- Frame: **X8 (OCTAQUAD/X)**
- ESC telemetry (current, RPM, temperature) via ArduPilot `esc_telem`
- Additional bench sensors (current, voltage, load cells) injected via MAVLink and exposed in Mission Planner as:
  - `customfield0..3` → LC1..4
  - `customfield4` → total current (MAV_CURR)
  - `customfield5` → total voltage (MAV_VOLT)

> ⚠️ **WARNING – SAFETY**
>
> - This system is intended for **test bench use only**.
> - **NO PROPS** on the motors.
> - Use proper restraints and safety procedures when running tests.

---

## Repository structure

```text
.
├── motortest.lua          # Lua script running on the flight controller
├── x8_motortest_mp.py     # Mission Planner Python script (GCS side)
└── README.md              # This file
```

You can name the Python file as you prefer (e.g. `motortest5_mp.py`), as long as you load it properly in Mission Planner.

---

## Overview

### Lua script: `motortest.lua`

Runs on the **flight controller** (ArduPilot scripting).  
It:

- drives **all 8 motors** (X8) with a predefined **multi-stage profile**:
  - Stage 1: 10 min @ ~10% duty
  - Stage 2: 10 min @ ~20% duty
  - Stage 3: 10 min @ ~40% duty  
  (placeholders – can be tuned)
- starts/stops based on the **user parameter** `SCR_USER4`
  - `SCR_USER4 = 1` → start test
  - `SCR_USER4 = 0` → stop test
- controls motor outputs by mapping `SERVOx_FUNCTION` == `Motor1..Motor8` (33..40)
- reads ESC telemetry via `esc_telem`:
  - per ESC: **RPM**, **current**, **temperature**
  - temperatures are converted from **centi-degrees** to **°C**
- aggregates data into:
  - Top group (ESC 1–4): **TopRPM**, **TopTemp**, **TopC**
  - Bottom group (ESC 5–8): **BotRPM**, **BotTemp**, **BotC**
  - Total current: **TotalC = TopC + BotC**
- classifies the test state:
  - `State = 0` → NORMAL
  - `State = 1` → WARNING
  - `State = 2` → EMERGENCY  
  based on:
  - temperature thresholds (`TEMP_WARN`, `TEMP_CRIT`)
  - total current thresholds (`TOTAL_CURR_WARN`, `TOTAL_CURR_CRIT`)
- does **not** stop the test by itself on EMERGENCY  
  (it only logs and reports state; actual abort logic is in the Python script).

#### Custom DataFlash logging (Lua)

The script writes 4 custom message types to the DataFlash log:

1. **`X8S`** – Summary (1 line per second)
   - fields:  
     `t, Stage, State, TopRPM, BotRPM, TopT, BotT, TopC, BotC, TotalC`
   - `t` [s] from test start  
   - `Stage` index (1,2,3,...)  
   - `State` = 0 (NORMAL), 1 (WARN), 2 (EMERG)  
   - `TopRPM` / `BotRPM` (average rpm per group)  
   - `TopT` / `BotT` (average temp, °C)  
   - `TopC` / `BotC` (sum of currents per group, A)  
   - `TotalC` (A)

2. **`X8R`** – per-ESC RPM
   - fields: `t, R1..R8`

3. **`X8T`** – per-ESC temperature (°C)
   - fields: `t, T1..T8`

4. **`X8C`** – per-ESC current (A)
   - fields: `t, I1..I8`

You can view these in Mission Planner via **DataFlash log review** (`Review a Log`) by selecting `X8S`, `X8R`, `X8T`, `X8C`.

#### GCS messages (Lua)

Every few seconds the script sends a compact status line like:

```text
X8 NORMAL t=0.8min RPM T/B=5200/5100 Temp T/B=45.2/44.8C I=76.5A
```

Where:

- `T/B` = Top / Bottom group
- `State` shown as: `NORMAL`, `WARN`, `EMERG`

---

### Python script: `x8_motortest_mp.py`

Runs in **Mission Planner** (GCS).  
It:

- toggles the Lua test by writing the **parameter** `SCR_USER4`:
  - if `SCR_USER4 == 0` → set to `1` and start the test
  - if `SCR_USER4 != 0` → set to `0` and stop the test
- implements a **multi-stage timing** reference identical to the Lua stages:
  - Stage 1: 10 min
  - Stage 2: 10 min
  - Stage 3: 10 min
- defines **expected values per stage** (placeholders):
  - `expected_rpm`
  - `expected_esc_curr` per ESC
  - `expected_total_curr` for the bench
- reads telemetry from Mission Planner `cs` object:
  - `esc1_rpm..esc8_rpm`
  - `esc1_curr..esc8_curr`
  - `esc1_temp..esc8_temp`
  - `customfield0..3` → LC1..4
  - `customfield4` → total current (MAV_CURR)
  - `customfield5` → total voltage (MAV_VOLT)
- prints **one compact status line per second** with:
  - stage, elapsed time, per-ESC rpm/A/°C
  - total current, total voltage
  - LC1..LC4

#### Anomaly checks (Python)

The Python script implements the **safety logic** and can **abort the test** by setting `SCR_USER4 = 0`.

**Immediate abort conditions** (after a short ramp-up time):

- ESC RPM near zero while the test should be running:
  - `0 < rpm_i < 500 rpm`
- ESC over-temperature:
  - `temp_i >= 80 °C`
- ESC over-current:
  - `curr_i > 100 A`
- Total bench over-current:
  - `total_curr > 100 A` (from Arduino / MAV_CURR)

**Two-level checks (warning + abort):**

Per stage, with expected values:

- **RPM vs expected**
  - warning if deviation > 10%
  - abort if deviation > 30%
- **Temperature vs ESC median**
  - warning if |temp - median| > 10 °C
  - abort if |temp - median| > 20 °C
- **Per-ESC current vs expected**
  - warning if deviation > 10%
  - abort if deviation > 30%
- **Total current vs expected**
  - warning if deviation > 10%
  - abort if deviation > 30%

On abort, the script prints:

```text
ABORT: <reason>
SCR_USER4 set to 0 (test stop requested).
```

and exits the loop.

---

## Installation

### 1. ArduPilot / Lua (`motortest.lua`)

1. Enable scripting in ArduPilot:
   - `SCR_ENABLE = 1` (or higher, depending on firmware)
2. Copy `motortest.lua` to the flight controller SD / onboard storage:
   - Typical path: `/APM/scripts/motortest.lua`
3. Ensure your X8 motor outputs are mapped correctly:
   - `SERVOx_FUNCTION = 33..40` for Motor1..Motor8
4. Ensure ESC telemetry is working:
   - ESC telemetry connected and configured
   - You can see `escX_rpm`, `escX_curr`, `escX_temp` in Mission Planner
5. Reboot the FC.  
   You should see a startup message similar to:
   ```text
   motortest.lua loaded - set SCR_USER4=1 to start X8 test
   ```

### 2. Mission Planner / Python (`x8_motortest_mp.py`)

1. Open **Mission Planner**.
2. Go to **Scripts** tab.
3. Load `x8_motortest_mp.py`.
4. Adjust stage expected values and thresholds in the script if needed:
   - `STAGES[...]`
   - threshold constants near the top of the file.
5. Click **Run** to start/stop the test:
   - If `SCR_USER4 = 0` → the script sets it to `1` and starts monitoring.
   - If `SCR_USER4 != 0` → the script sets it to `0` and requests a test stop.

---

## Usage

### Starting a test

From Mission Planner:

1. Make sure:
   - Motors have **no props**.
   - Bench and power supply are safe.
2. Load `x8_motortest_mp.py` in the Scripts tab.
3. Run the script.
4. The script sets `SCR_USER4=1` and starts the test.
5. Watch the MP console for:
   - per-second status lines,
   - eventual `WARN:` messages,
   - any `ABORT:` reason.

Alternatively, you can start the test by manually setting `SCR_USER4 = 1`, but you will **lose** the automatic aborts from the Python script.

### Stopping a test

- Automatically:
  - If Python detects an anomaly → `SCR_USER4 = 0` (abort).
  - If the Lua script completes all stages → it sets `SCR_USER4 = 0`.
- Manually:
  - Set `SCR_USER4 = 0` from Mission Planner (Full Parameter List).
  - Re-run the Python script while `SCR_USER4 != 0` → it forces it back to `0`.

---

## Logs and analysis

### DataFlash (FC side)

- Download the DataFlash log from the FC (via Mission Planner).
- In **Review a Log**, select:
  - `X8S` for high-level summary (state, averages, total current).
  - `X8R` for per-ESC RPM.
  - `X8T` for per-ESC temperature.
  - `X8C` for per-ESC current.
- Correlate these with standard ArduPilot messages if needed (e.g. ESC status).

### Mission Planner console (PC side)

- The Python script prints:
  - One **status line per second** with stage + ESC + bench telemetry.
  - `WARN:` lines for non-critical anomalies.
  - `ABORT:` lines when thresholds are exceeded and the test is stopped.

You can copy the console output or redirect it to a file for further analysis.

---

## Tuning

The following places are intended to be tuned for your specific hardware:

- In **Lua (`motortest.lua`)**:
  - Stage definition: `STAGES` (duration, duty).
  - State thresholds:
    - `TEMP_WARN`, `TEMP_CRIT`
    - `TOTAL_CURR_WARN`, `TOTAL_CURR_CRIT`

- In **Python (`x8_motortest_mp.py`)**:
  - `STAGES`: `expected_rpm`, `expected_esc_curr`, `expected_total_curr`
  - Abort thresholds:
    - `RPM_NEAR_ZERO_ABORT`
    - `TEMP_CRIT_ABORT`
    - `ESC_CURR_MAX_ABORT`
    - `TOTAL_CURR_MAX_ABORT`
  - Warning/abort fractions:
    - `RPM_EXPECT_WARN_FRAC`, `RPM_EXPECT_ABORT_FRAC`
    - `TEMP_DIFF_WARN`, `TEMP_DIFF_ABORT`
    - `CURR_EXPECT_WARN_FRAC`, `CURR_EXPECT_ABORT_FRAC`

---

## License
