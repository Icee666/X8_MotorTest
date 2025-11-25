-- motortest.lua
-- X8 multi-stage motor test with ESC telemetry logging
-- WARNING: works also when DISARMED.
-- USE ONLY ON APPOSITE TEST BENCH.

local TEST_PARAM_NAME   = "SCR_USER4"   -- user param used as trigger
local UPDATE_PERIOD_MS  = 100           -- 100 ms (10 Hz)

local MAV_SEVERITY_INFO = 6

-- Copter motor functions: Motor1..Motor8
local MOTOR_FUNCTIONS = {33, 34, 35, 36, 37, 38, 39, 40}

-- TEST STAGES (placeholders)
-- 3 stages, 10 minutes each, at 10%, 20%, 40%
local STAGES = {
    {
        name = "Stage 1 (10%)",
        duty = 0.10,
        duration_ms = 10 * 60 * 1000,
    },
    {
        name = "Stage 2 (20%)",
        duty = 0.20,
        duration_ms = 10 * 60 * 1000,
    },
    {
        name = "Stage 3 (40%)",
        duty = 0.40,
        duration_ms = 10 * 60 * 1000,
    },
}

-- internal test state
local motor_channels   = {}
local test_active      = false
local current_stage    = 0          -- 1-based index into STAGES
local stage_start_ms   = nil        -- Duration
local pwm_min          = 1000
local pwm_max          = 2000
local startup_ignored  = false      -- to avoid auto-start on boot
local last_trigger     = 0

-- logging and ESC telemetry
local esc_telem = esc_telem
local rpm_sens  = rpm  -- may be nil if RPM library not configured

-- logging configuration
local LOG_PERIOD_MS     = 1000      -- summary logs every 1s
local DISPLAY_PERIOD_MS = 2000      -- GCS text every 2s

local last_log_ms       = nil       -- Duration
local last_display_ms   = nil       -- Duration
local test_start_ms     = nil       -- Duration

-- test state codes
local STATE_NORMAL    = 0
local STATE_WARNING   = 1
local STATE_EMERGENCY = 2
local state_code      = STATE_NORMAL

-- very simple thresholds (placeholders, tune as needed)
local TEMP_WARN        = 60.0   -- degC
local TEMP_CRIT        = 80.0   -- degC
local TOTAL_CURR_WARN  = 80.0   -- A
local TOTAL_CURR_CRIT  = 100.0  -- A

---------------------------------------------------------
-- MOTOR MAPPING AND PWM
---------------------------------------------------------

-- find output channels mapped to motors
local function init_motors()
    motor_channels = {}
    for _, func in ipairs(MOTOR_FUNCTIONS) do
        local chan = SRV_Channels:find_channel(func)
        if chan then
            motor_channels[#motor_channels + 1] = chan
        end
    end

    if #motor_channels ~= #MOTOR_FUNCTIONS then
        gcs:send_text(MAV_SEVERITY_INFO,
            string.format("motortest: found %d/%d motor channels",
                          #motor_channels, #MOTOR_FUNCTIONS))
    else
        gcs:send_text(MAV_SEVERITY_INFO,
            "motortest: found all 8 motor channels")
    end
end

-- set all motors to a PWM value for timeout_ms milliseconds
local function set_all_motors(pwm, timeout_ms)
    if #motor_channels == 0 then
        return
    end
    local t = timeout_ms or (UPDATE_PERIOD_MS + 50)
    for _, chan in ipairs(motor_channels) do
        SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, t)
    end
end

-- duty (0..1) to PWM between MOT_PWM_MIN and MOT_PWM_MAX
local function pwm_for_duty(duty)
    local span = pwm_max - pwm_min
    if span < 100 then
        span = 400
    end
    if duty < 0 then duty = 0 end
    if duty > 1 then duty = 1 end
    return math.floor(pwm_min + span * duty + 0.5)
end

---------------------------------------------------------
-- ESC TELEMETRY, STATS AND STATE
---------------------------------------------------------

-- return rpm, current, temperature (degC) for ESC index (0-based)
local function get_esc_values(idx)
    local rpm_val  = 0.0
    local curr_val = 0.0
    local temp_val = 0.0

    -- RPM from ESC telemetry (preferred, same source as temps/currents)
    if esc_telem then
        if esc_telem.get_rpm then
            local r = esc_telem:get_rpm(idx)
            if r ~= nil then
                rpm_val = r
            end
        elseif esc_telem.get_erpm then
            local erpm = esc_telem:get_erpm(idx)
            if erpm ~= nil then
                -- if only eRPM is available, use it as-is for now
                rpm_val = erpm
            end
        end
    end

    -- optional fallback: RPM library (if configured and ESC telem gave 0)
    if rpm_val == 0 and rpm_sens and rpm_sens.get_rpm then
        local r2 = rpm_sens:get_rpm(idx)
        if r2 ~= nil then
            rpm_val = r2
        end
    end

    -- current and temperature from ESC telemetry
    if esc_telem then
        if esc_telem.get_current then
            local c = esc_telem:get_current(idx)
            if c ~= nil then
                curr_val = c
            end
        end

        local t = nil
        if esc_telem.get_motor_temperature then
            t = esc_telem:get_motor_temperature(idx)
        end
        if (t == nil) and esc_telem.get_temperature then
            t = esc_telem:get_temperature(idx)
        end
        if t ~= nil then
            -- ESC telemetry is in centi-degrees (0.01 degC)
            local t_c = t * 0.01
            if t_c < -50 or t_c > 200 then
                temp_val = 0.0
            else
                temp_val = t_c
            end
        end
    end

    return rpm_val, curr_val, temp_val
end

-- compute per-ESC arrays and top/bottom averages
local function compute_group_stats()
    local rpm_arr   = {}
    local curr_arr  = {}
    local temp_arr  = {}

    local top_rpm_sum, top_temp_sum, top_curr_sum = 0, 0, 0
    local bot_rpm_sum, bot_temp_sum, bot_curr_sum = 0, 0, 0
    local top_count, bot_count = 0, 0

    for i = 0, 7 do
        local rpm_val, curr_val, temp_val = get_esc_values(i)

        rpm_arr[i+1]  = rpm_val
        curr_arr[i+1] = curr_val
        temp_arr[i+1] = temp_val

        if i <= 3 then
            -- top motors: ESC 0..3 => M1..M4
            top_rpm_sum  = top_rpm_sum  + rpm_val
            top_temp_sum = top_temp_sum + temp_val
            top_curr_sum = top_curr_sum + curr_val
            top_count = top_count + 1
        else
            -- bottom motors: ESC 4..7 => M5..M8
            bot_rpm_sum  = bot_rpm_sum  + rpm_val
            bot_temp_sum = bot_temp_sum + temp_val
            bot_curr_sum = bot_curr_sum + curr_val
            bot_count = bot_count + 1
        end
    end

    local top_rpm, bot_rpm   = 0, 0
    local top_temp, bot_temp = 0, 0
    local top_curr, bot_curr = 0, 0

    if top_count > 0 then
        top_rpm  = top_rpm_sum  / top_count
        top_temp = top_temp_sum / top_count
        top_curr = top_curr_sum
    end
    if bot_count > 0 then
        bot_rpm  = bot_rpm_sum  / bot_count
        bot_temp = bot_temp_sum / bot_count
        bot_curr = bot_curr_sum
    end

    local total_curr = top_curr + bot_curr

    return rpm_arr, curr_arr, temp_arr,
           top_rpm, bot_rpm, top_temp, bot_temp, top_curr, bot_curr, total_curr
end

-- classify overall state from top/bottom temp and total current
local function classify_state(top_temp, bot_temp, total_curr)
    local state = STATE_NORMAL

    if (top_temp >= TEMP_CRIT) or (bot_temp >= TEMP_CRIT) or (total_curr >= TOTAL_CURR_CRIT) then
        state = STATE_EMERGENCY
    elseif (top_temp >= TEMP_WARN) or (bot_temp >= TEMP_WARN) or (total_curr >= TOTAL_CURR_WARN) then
        state = STATE_WARNING
    else
        state = STATE_NORMAL
    end

    return state
end

-- DataFlash logging (per ESC + summary) and compact GCS text
local function log_and_display_stats()
    if not test_active then
        return
    end
    if not logger then
        return
    end

    local now = millis()
    if not test_start_ms then
        test_start_ms = now
    end

    -- now, test_start_ms, last_log_ms, last_display_ms are Duration
    local elapsed_ms = (now - test_start_ms):tofloat()
    local elapsed_sec = elapsed_ms * 0.001

    local rpm_arr, curr_arr, temp_arr,
          top_rpm, bot_rpm, top_temp, bot_temp, top_curr, bot_curr, total_curr =
        compute_group_stats()

    state_code = classify_state(top_temp, bot_temp, total_curr)

    -- 1) DataFlash logs every LOG_PERIOD_MS
    if not last_log_ms then
        last_log_ms = now
    end
    local dt_log_ms = (now - last_log_ms):tofloat()
    if dt_log_ms >= LOG_PERIOD_MS then
        -- X8S: summary (top/bottom averages + state)
        -- fields: t,Stage,State,TopRPM,BotRPM,TopT,BotT,TopC,BotC,TotalC
        logger.write("X8S",
                     "t,Stage,State,TopRPM,BotRPM,TopT,BotT,TopC,BotC,TotalC",
                     "ffffffffff",
                     elapsed_sec,
                     current_stage,
                     state_code,
                     top_rpm,
                     bot_rpm,
                     top_temp,
                     bot_temp,
                     top_curr,
                     bot_curr,
                     total_curr)

        -- X8R: per-ESC RPM
        logger.write("X8R",
                     "t,R1,R2,R3,R4,R5,R6,R7,R8",
                     "fffffffff",
                     elapsed_sec,
                     rpm_arr[1] or 0, rpm_arr[2] or 0, rpm_arr[3] or 0, rpm_arr[4] or 0,
                     rpm_arr[5] or 0, rpm_arr[6] or 0, rpm_arr[7] or 0, rpm_arr[8] or 0)

        -- X8T: per-ESC temperature (degC)
        logger.write("X8T",
                     "t,T1,T2,T3,T4,T5,T6,T7,T8",
                     "fffffffff",
                     elapsed_sec,
                     temp_arr[1] or 0, temp_arr[2] or 0, temp_arr[3] or 0, temp_arr[4] or 0,
                     temp_arr[5] or 0, temp_arr[6] or 0, temp_arr[7] or 0, temp_arr[8] or 0)

        -- X8C: per-ESC current
        logger.write("X8C",
                     "t,I1,I2,I3,I4,I5,I6,I7,I8",
                     "fffffffff",
                     elapsed_sec,
                     curr_arr[1] or 0, curr_arr[2] or 0, curr_arr[3] or 0, curr_arr[4] or 0,
                     curr_arr[5] or 0, curr_arr[6] or 0, curr_arr[7] or 0, curr_arr[8] or 0)

        last_log_ms = now
    end

    -- 2) Compact GCS text every DISPLAY_PERIOD_MS
    if not last_display_ms then
        last_display_ms = now
    end
    local dt_disp_ms = (now - last_display_ms):tofloat()
    if dt_disp_ms >= DISPLAY_PERIOD_MS then
        local state_txt = "NORMAL"
        if state_code == STATE_WARNING then
            state_txt = "WARN"
        elseif state_code == STATE_EMERGENCY then
            state_txt = "EMERG"
        end

        gcs:send_text(
            MAV_SEVERITY_INFO,
            string.format(
                "X8 %s t=%.1fmin RPM T/B=%.0f/%.0f Temp T/B=%.1f/%.1fC I=%.1fA",
                state_txt,
                elapsed_sec / 60.0,
                top_rpm, bot_rpm,
                top_temp, bot_temp,
                total_curr
            )
        )

        last_display_ms = now
    end
end

---------------------------------------------------------
-- TEST STAGES CONTROL
---------------------------------------------------------

-- go to next stage, or end test if no more stages
local function advance_stage()
    current_stage = current_stage + 1
    if current_stage > #STAGES then
        -- all stages completed
        test_active = false
        set_all_motors(pwm_min, 200)
        local ok = param:set(TEST_PARAM_NAME, 0)
        if ok then
            gcs:send_text(MAV_SEVERITY_INFO,
                "motortest: all stages done, motors off, SCR_USER4=0")
        else
            gcs:send_text(MAV_SEVERITY_INFO,
                "motortest: all stages done, motors off (SCR_USER4 unchanged)")
        end
        return
    end

    local stage = STAGES[current_stage]
    stage_start_ms = millis()

    local pwm = pwm_for_duty(stage.duty)
    set_all_motors(pwm, UPDATE_PERIOD_MS + 200)

    gcs:send_text(MAV_SEVERITY_INFO,
        string.format("motortest: %s start (duty=%.0f%%, %.1fmin, pwm=%d)",
                      stage.name, stage.duty * 100.0,
                      stage.duration_ms / 60000.0, pwm))
end

-- stop test and set motors to minimum
local function stop_test(reason)
    test_active = false
    set_all_motors(pwm_min, 200)

    local ok = param:set(TEST_PARAM_NAME, 0)
    if ok then
        gcs:send_text(MAV_SEVERITY_INFO,
            "motortest: stopped (" .. reason .. "), SCR_USER4=0")
    else
        gcs:send_text(MAV_SEVERITY_INFO,
            "motortest: stopped (" .. reason .. "), SCR_USER4 unchanged")
    end
end

---------------------------------------------------------
-- MAIN LOOP
---------------------------------------------------------

function update()
    local trigger = param:get(TEST_PARAM_NAME) or 0

    -- first run after boot: avoid auto-start if SCR_USER4 was already 1
    if not startup_ignored then
        last_trigger = trigger
        startup_ignored = true
        return update, UPDATE_PERIOD_MS
    end

    -- rising edge: SCR_USER4 0 -> non-zero => start test
    if (not test_active) and (last_trigger == 0) and (trigger ~= 0) then
        if #motor_channels == 0 then
            init_motors()
        end

        if #motor_channels == 0 then
            gcs:send_text(MAV_SEVERITY_INFO,
                "motortest: no motor channels, check SERVOx_FUNCTION")
            last_trigger = trigger
            return update, UPDATE_PERIOD_MS
        end

        pwm_min = param:get("MOT_PWM_MIN") or 1000
        pwm_max = param:get("MOT_PWM_MAX") or 2000

        test_active     = true
        current_stage   = 0   -- advance_stage() will set to 1
        test_start_ms   = nil
        state_code      = STATE_NORMAL
        last_log_ms     = nil
        last_display_ms = nil

        gcs:send_text(MAV_SEVERITY_INFO,
            "motortest: X8 multi-stage test start")
        advance_stage()
    end

    if test_active then
        -- user turned SCR_USER4 back to 0: stop immediately
        if trigger == 0 then
            stop_test("SCR_USER4=0")
        else
            -- stage timing
            local now = millis()
            if not stage_start_ms then
                stage_start_ms = now
            end
            local elapsed_ms = (now - stage_start_ms):tofloat()
            local stage = STAGES[current_stage]
            if elapsed_ms >= (stage.duration_ms or 0) then
                advance_stage()
            else
                -- refresh PWM to avoid timeout
                local pwm = pwm_for_duty(stage.duty)
                set_all_motors(pwm, UPDATE_PERIOD_MS + 200)
            end

            -- logging and GCS text
            log_and_display_stats()
        end
    end

    last_trigger = trigger
    return update, UPDATE_PERIOD_MS
end

-- startup message
gcs:send_text(MAV_SEVERITY_INFO,
    "motortest.lua loaded - set SCR_USER4=1 to start X8 test")

-- initial motor mapping
init_motors()

-- register periodic update
return update, UPDATE_PERIOD_MS