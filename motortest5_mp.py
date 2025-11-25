# x8_motortest_mp.py
# Mission Planner Python script
#
# - Toggles the X8 motor test Lua script via SCR_USER4
# - Multi-stage test (stages are defined mainly in Lua)
# - Monitors ESC telemetry (escX_rpm, escX_curr, escX_temp)
# - Monitors Arduino data mapped to customfield0..5
# - Performs anomaly checks (warnings + aborts)
#
# NOTE: This script runs on the GCS (Mission Planner).
# Motors are driven by the Lua script on the flight controller.
# USE ONLY ON TEST BENCH.

TEST_PARAM_NAME = "SCR_USER4"
SLEEP_MS = 1000  # loop rate: 1 Hz

# ---------------- STAGES (reference for checks) ----------------
# These durations and expected values are only used for anomaly checks.
# The actual PWM duty per stage is controlled by the Lua script.

STAGES = [
    {
        "name": "Stage 1 (10%)",
        "duration_sec": 10 * 60,
        "expected_rpm": 0000.0,
        "expected_esc_curr": 0.0,
        "expected_total_curr": 00.0,
    },
    {
        "name": "Stage 2 (20%)",
        "duration_sec": 10 * 60,
        "expected_rpm": 3500.0,
        "expected_esc_curr": 10.0,
        "expected_total_curr": 80.0,
    },
    {
        "name": "Stage 3 (40%)",
        "duration_sec": 10 * 60,
        "expected_rpm": 7000.0,
        "expected_esc_curr": 20.0,
        "expected_total_curr": 160.0,
    },
]

TOTAL_TEST_DURATION_SEC = sum(s["duration_sec"] for s in STAGES)

# --------------- SAFETY THRESHOLDS (placeholders) --------------

AUTO_STOP_ON_ANOMALY = True

# Grace time after each stage change before strict checks
RAMP_UP_GRACE_SEC = 5

# Immediate abort conditions
RPM_NEAR_ZERO_ABORT   = 500.0     # rpm: motor almost stopped
TEMP_CRIT_ABORT       = 80.0      # degC: critical temperature
ESC_CURR_MAX_ABORT    = 100.0     # A: per ESC (very high)
TOTAL_CURR_MAX_ABORT  = 100.0     # A: total (customfield4)

# Two-level checks (warning + abort)
RPM_EXPECT_WARN_FRAC   = 0.10     # ±10% vs expected
RPM_EXPECT_ABORT_FRAC  = 0.30     # ±30% vs expected

TEMP_DIFF_WARN         = 10.0     # degC from median
TEMP_DIFF_ABORT        = 20.0     # degC from median

CURR_EXPECT_WARN_FRAC  = 0.10     # ±10% vs expected (per ESC)
CURR_EXPECT_ABORT_FRAC = 0.30     # ±30% vs expected (per ESC)

print("=== X8 Motor Test (MP script) ===")


def get_stage_at_time(t_sec):
    """
    Return (stage_index, stage_elapsed_sec) for given time from start.
    stage_index is 0-based.
    """
    accum = 0
    for idx, s in enumerate(STAGES):
        dur = s["duration_sec"]
        if t_sec < accum + dur:
            return idx, t_sec - accum
        accum += dur
    # Beyond total duration: stay at last stage
    return len(STAGES) - 1, STAGES[-1]["duration_sec"]


def read_esc_telemetry():
    """
    Read ESC telemetry from Mission Planner (cs).
    escX_rpm, escX_curr, escX_temp for X = 1..8.
    Returns: (rpms[], currents[], temps[])
    """
    rpms = []
    currents = []
    temps = []
    for i in range(1, 9):
        rpm_attr = "esc%d_rpm" % i
        cur_attr = "esc%d_curr" % i
        tmp_attr = "esc%d_temp" % i
        rpm = float(getattr(cs, rpm_attr, 0.0))
        curr = float(getattr(cs, cur_attr, 0.0))
        tmp = float(getattr(cs, tmp_attr, 0.0))
        rpms.append(rpm)
        currents.append(curr)
        temps.append(tmp)
    return rpms, currents, temps


def read_mav_arduino():
    """
    Read Arduino data mapped to customfield0..5 in Mission Planner.
      customfield0..3 : LC1..4 (generic load channels)
      customfield4    : total current (A)
      customfield5    : total voltage (V)
    Returns: (lc_list[4], total_curr, total_volt)
    """
    lc_fields = ["customfield0", "customfield1", "customfield2", "customfield3"]
    lcs = []
    for name in lc_fields:
        lcs.append(float(getattr(cs, name, 0.0)))

    total_curr = float(getattr(cs, "customfield4", 0.0))
    total_volt = float(getattr(cs, "customfield5", 0.0))

    return lcs, total_curr, total_volt


def print_status_line(t_sec, stage_idx, stage_elapsed, rpms, currents, temps, lcs, total_curr, total_volt):
    """
    Print a compact status line for the console (1 line per second).
    """
    stage = STAGES[stage_idx]
    line = "t=%4ds [%s t=%3ds] " % (t_sec, stage["name"], stage_elapsed)

    # ESC summary: E#: RPM/A/degC
    for i in range(8):
        line += "E%d:%4drpm/%4.1fA/%4.1fC " % (
            i + 1,
            int(rpms[i]),
            currents[i],
            temps[i]
        )

    # Arduino / bench totals
    line += "| I_tot=%.1fA V=%.1fV " % (total_curr, total_volt)
    # LC channels
    line += "| LC: " + ", ".join("L%d=%.1f" % (i + 1, v) for i, v in enumerate(lcs))

    print(line)


def median(values):
    vals = [v for v in values if v == v]  # filter NaN
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def check_and_maybe_abort(t_sec, stage_idx, stage_elapsed, rpms, currents, temps, lcs, total_curr, total_volt):
    """
    Apply anomaly checks.
    Returns True if an abort was requested (sets SCR_USER4 to 0).
    """
    stage = STAGES[stage_idx]
    exp_rpm        = stage.get("expected_rpm", 0.0)        or 0.0
    exp_esc_curr   = stage.get("expected_esc_curr", 0.0)   or 0.0
    exp_total_curr = stage.get("expected_total_curr", 0.0) or 0.0

    def abort(reason):
        print("ABORT: %s" % reason)
        if AUTO_STOP_ON_ANOMALY:
            if Script.ChangeParam(TEST_PARAM_NAME, 0):
                print("SCR_USER4 set to 0 (test stop requested).")
            else:
                print("WARNING: cannot set SCR_USER4 to 0, check params.")
        return True

    # ------------ Immediate abort checks ----------------

    if stage_elapsed >= RAMP_UP_GRACE_SEC:
        # ESC rpm close to zero while test is active
        for i in range(8):
            if rpms[i] > 0 and rpms[i] < RPM_NEAR_ZERO_ABORT:
                return abort("ESC%d low rpm (%.0f < %.0f)" %
                             (i + 1, rpms[i], RPM_NEAR_ZERO_ABORT))

    # ESC critical temperature
    for i in range(8):
        if temps[i] >= TEMP_CRIT_ABORT and temps[i] > 0:
            return abort("ESC%d over temperature (%.1fC >= %.1fC)" %
                         (i + 1, temps[i], TEMP_CRIT_ABORT))

    # ESC over-current
    for i in range(8):
        if currents[i] > ESC_CURR_MAX_ABORT:
            return abort("ESC%d over current (%.1fA > %.1fA)" %
                         (i + 1, currents[i], ESC_CURR_MAX_ABORT))

    # Total bench over-current (Arduino / customfield4)
    if total_curr > TOTAL_CURR_MAX_ABORT:
        return abort("Total current too high (%.1fA > %.1fA)" %
                     (total_curr, TOTAL_CURR_MAX_ABORT))

    # ------------ Two-level checks (warn + abort) --------

    # 1) rpm vs expected for this stage
    if stage_elapsed >= RAMP_UP_GRACE_SEC and exp_rpm > 0:
        for i in range(8):
            frac = abs(rpms[i] - exp_rpm) / exp_rpm
            if frac > RPM_EXPECT_ABORT_FRAC:
                return abort("ESC%d rpm out of range (%.0f vs %.0f, %.0f%%)" %
                             (i + 1, rpms[i], exp_rpm, frac * 100.0))
            elif frac > RPM_EXPECT_WARN_FRAC:
                print("WARN: ESC%d rpm off (%.0f vs %.0f, %.0f%%)" %
                      (i + 1, rpms[i], exp_rpm, frac * 100.0))

    # 2) temperature vs median ESC temperature
    med_temp = median([t for t in temps if t > 0])
    if med_temp is not None:
        for i in range(8):
            if temps[i] <= 0:
                continue
            diff = temps[i] - med_temp
            adiff = abs(diff)
            if adiff > TEMP_DIFF_ABORT:
                return abort("ESC%d temp off median (%.1fC vs %.1fC, diff=%.1fC)" %
                             (i + 1, temps[i], med_temp, diff))
            elif adiff > TEMP_DIFF_WARN:
                print("WARN: ESC%d temp off median (%.1fC vs %.1fC, diff=%.1fC)" %
                      (i + 1, temps[i], med_temp, diff))

    # 3) current vs expected per ESC
    if exp_esc_curr > 0:
        for i in range(8):
            frac = abs(currents[i] - exp_esc_curr) / exp_esc_curr
            if frac > CURR_EXPECT_ABORT_FRAC:
                return abort("ESC%d current out of range (%.1fA vs %.1fA, %.0f%%)" %
                             (i + 1, currents[i], exp_esc_curr, frac * 100.0))
            elif frac > CURR_EXPECT_WARN_FRAC:
                print("WARN: ESC%d current off (%.1fA vs %.1fA, %.0f%%)" %
                      (i + 1, currents[i], exp_esc_curr, frac * 100.0))

    # 4) total current vs expected
    if exp_total_curr > 0:
        frac_tot = abs(total_curr - exp_total_curr) / exp_total_curr
        if frac_tot > CURR_EXPECT_ABORT_FRAC:
            return abort("Total current out of range (%.1fA vs %.1fA, %.0f%%)" %
                         (total_curr, exp_total_curr, frac_tot * 100.0))
        elif frac_tot > CURR_EXPECT_WARN_FRAC:
            print("WARN: total current off (%.1fA vs %.1fA, %.0f%%)" %
                  (total_curr, exp_total_curr, frac_tot * 100.0))

    return False  # no abort requested


# ---------------------- MAIN SCRIPT ----------------------------

current = Script.GetParam(TEST_PARAM_NAME)
if current is None:
    print("ERROR: cannot read param %s" % TEST_PARAM_NAME)
    raise SystemExit

if current < 0.5:
    # Test is OFF -> turn it ON and start monitoring
    if not Script.ChangeParam(TEST_PARAM_NAME, 1):
        print("ERROR: cannot set %s to 1" % TEST_PARAM_NAME)
        raise SystemExit

    print("SCR_USER4=1 (Lua test requested).")
    print("Planned test duration: ~%d minutes." % (TOTAL_TEST_DURATION_SEC // 60))
    print("To stop: set SCR_USER4=0 or let this script abort on anomaly.")

    # Main loop: 1 Hz, until total duration or param goes back to 0
    for t in range(TOTAL_TEST_DURATION_SEC):
        stage_idx, stage_elapsed = get_stage_at_time(t)

        rpms, currents, temps = read_esc_telemetry()
        lcs, total_curr, total_volt = read_mav_arduino()

        print_status_line(t, stage_idx, stage_elapsed, rpms, currents, temps, lcs, total_curr, total_volt)

        if check_and_maybe_abort(t, stage_idx, stage_elapsed, rpms, currents, temps, lcs, total_curr, total_volt):
            break  # abort requested

        Script.Sleep(SLEEP_MS)

        # If Lua or user already set SCR_USER4 to 0, stop logging
        current = Script.GetParam(TEST_PARAM_NAME)
        if current is None:
            print("WARNING: cannot read %s, exiting script loop." % TEST_PARAM_NAME)
            break
        if current < 0.5:
            print("SCR_USER4 is 0, stopping script at t=%ds." % (t + 1))
            break

    # At the end, if SCR_USER4 is still 1, try to force it to 0
    current = Script.GetParam(TEST_PARAM_NAME) or 0
    if current >= 0.5:
        if Script.ChangeParam(TEST_PARAM_NAME, 0):
            print("Test ended: SCR_USER4 set to 0.")
        else:
            print("WARNING: cannot set SCR_USER4 to 0, check params.")
    else:
        print("Test ended: SCR_USER4 was already 0.")

else:
    # Test is ON -> turn it OFF (quick toggle)
    if Script.ChangeParam(TEST_PARAM_NAME, 0):
        print("SCR_USER4 was non-zero, set to 0. Test stop requested.")
    else:
        print("ERROR: cannot set %s to 0" % TEST_PARAM_NAME)

print("=== Script finished ===")