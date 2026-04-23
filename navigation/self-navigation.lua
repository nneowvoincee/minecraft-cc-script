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

local defaultHeight = 150       -- standard moving height for long-distance navigation
local CHANNEL_BROADCAST = 10001

-- ==============================================
-- Initialize
-- ==============================================
local heightSensor = peripheral.wrap("top")
local velSensor = peripheral.wrap("right")

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

CHANNEL_BROADCAST = 10001
peripheral.find("modem", rednet.open)

local x, y, z = gps.locate()
if x == nil then error("GPS is not set up.", 0) end
local vector = require("vector")

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

local function getInFrontCompCoor() -- the coordinate of the front computer
    rednet.broadcast("locate");id, message = rednet.receive()
    local a, b, c = message:match("(%S+)%s+(%S+)%s+(%S+)")
    return tonumber(a), tonumber(b), tonumber(c)
end

local function getRelativeAngle(target)
    local pSelf = vector.new(gps.locate())
    local pFront = vector.new(getInFrontCompCoor())

    -- 当前朝向的水平方向向量
    local dirSelf = pFront - pSelf
    -- 目标方向的水平方向向量
    local dirTarget = target - pSelf

    -- 使用 atan2 计算 yaw（MC 坐标系：南=0，西为正）
    local yawSelf = math.atan2(-dirSelf.x, dirSelf.z)   -- 弧度
    local yawTarget = math.atan2(-dirTarget.x, dirTarget.z)

    -- 差值并标准化到 [-π, π]
    local delta = yawTarget - yawSelf
    delta = (delta + math.pi) % (2 * math.pi) - math.pi

    return math.deg(delta)   -- 返回度数，正=右，负=左
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
        write("New target (x z y): ")
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