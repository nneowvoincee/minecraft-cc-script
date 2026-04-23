-- ==============================================
-- Physical Parameters -- Please fill according to your aircraft
-- ==============================================
local FAN_FORCE_UP = 3732.0        -- Upward force at full fan thrust (Newtons)
local MASS = 241                 -- Aircraft mass (kg)
local GRAVITY = 11        -- Gravity acceleration (m/s²), usually no need to change
local MAX_ACC_UP = FAN_FORCE_UP/MASS

-- Control Parameters
local DELTA = 15   -- The larger the number, the faster the initial acceleration, but may overshoot

local KP, KD = 3, 0.3

local ZONE = 10.0             -- Distance to target within which fine control is enabled (blocks)
local TICK = 0.1


-- ==============================================
-- Initialize sensors & monitor
-- ==============================================
local heightSensor = peripheral.wrap("top")
local velSensor = peripheral.wrap("right")
term.redirect(peripheral.wrap("monitor"))

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

local targetHeight = 200

-- ==============================================
-- Helper functions: get height, velocity, air pressure
-- ==============================================
local function getHeight()
    local h = heightSensor.getHeight()
    return (type(h) == "number") and h or nil
end

local function getVelocity()
    local v = velSensor.getVelocity()
    return (type(v) == "number") and v or nil
end

local function getAirPressure()
    local p = heightSensor.getAirPressure()
    return (type(p) == "number") and p or 1.0   -- Default 1.0
end


-- ==============================================
-- Main control loop
-- ==============================================
local function controlTask()
    while true do
        local h = getHeight()
        local v = getVelocity()
        local v_up = -v
        local inFineZone = false   -- Record whether the previous cycle was in the fine zone

        if h == nil then h = targetHeight end
        if v == nil then v = 0 end

        local error = targetHeight - h
        local airPressure = getAirPressure()

        -- Calculate base thrust required for hovering (feedforward)
        local base_ratio = GRAVITY / (MAX_ACC_UP * airPressure)
        local base_output = base_ratio * 15  -- Floating point, round at the end
        local output, target_acc

        if math.abs(error) <= ZONE then
            -- ===== Fine zone: PD damping control =====
            if not inFineZone then
                -- Entering fine zone, keep output continuous, do not reset PID history (use independent PD variables)
                inFineZone = true
                lastError = error
            end

            local desired_vel = KP * error
            local vel_error = desired_vel - v_up   -- Velocity error

            -- Thrust correction = proportional to velocity error (actually equivalent to P-D cascade)
            local correction = KD * vel_error
            local limited_corr = 15 - base_output
            correction = math.max(-limited_corr, math.min(limited_corr, correction))
            -- Total thrust (feedforward + correction)
            output = base_output + correction + (error/ZONE)*2

             -- Debug display
            term.setCursorPos(1, 6)
            term.clearLine()
            term.write(string.format("cor:%6.1f",
                correction))
        else
            -- ===== Physics prediction braking zone =====
            local current_kinetic = v*v/2
            local required_potential = GRAVITY* error
            local final_required_kinetic = 0
            inFineZone = true

            if (v_up <= 0) then
                current_kinetic = -current_kinetic
            end
            final_required_kinetic = required_potential*DELTA - current_kinetic

            target_acc = final_required_kinetic / math.abs(error)
            local airPressure = getAirPressure()
            local ratio = (target_acc + GRAVITY) / (MAX_ACC_UP * airPressure)
            output = ratio * 15
        end

        output = math.floor(output + 0.5)
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end

        -- Output to propeller (inverted)
        redstone.setAnalogOutput("bottom", 15 - output)

         -- Debug display
        term.setCursorPos(1, 5)
        term.clearLine()
        term.write(string.format("H:%6.1f T:%6.1f V:%5.2f Err:%6.1f Out:%2d",
            h, targetHeight, v, error, output))

        sleep(TICK)
    end
end

-- ==============================================
-- User input task
-- ==============================================
local function inputTask()
    term.setTextColor(colors.yellow)
    print("Hybrid Controller (PID + Physics Brake)")
    print("Target: " .. targetHeight)
    print("Enter new target (>= -64):")
    term.setTextColor(colors.white)

    while true do
        write("New target: ")
        local n = tonumber(read())
        if n and n >= -64 then
            targetHeight = n
            print("Target updated to " .. targetHeight)
        else
            print("Invalid, must be >= -64")
        end
    end
end

parallel.waitForAny(controlTask, inputTask)