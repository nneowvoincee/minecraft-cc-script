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

local Y_ZONE = 10.0             -- Distance to target within which fine control is enabled (blocks)
local Y_TICK = 0.1

CHANNEL_BROADCAST = 10001
local navigationAltitude = 150       -- standard moving height for long-distance navigation
local HOR_ZONE = 100
local HOR_TICK = 0.5

local DISABLE_SLEEP_TICK = 2
-- ==============================================
-- Initialize
-- ==============================================
-- sensor
local heightSensor = peripheral.find("top")
local velSensor = peripheral.wrap("right")

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

-- modem + gps
peripheral.find("modem", rednet.open)
local target = vector.new(gps.locate())

if x == nil then error("GPS is not set up.", 0) end

local targetHeight = y  -- set current height as target height (don't move initially

-- redstone relay, for outputting signal
local relay = peripheral.find("redstone_relay")
if relay == nil then error("Missing redstone relay", 0) end

-- displat
local monitor = peripheral.find("monitor")
if monitor == nil then error("Missing monitor", 0) end
term.redirect(monitor)

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

local function isDisable()
    return not redstone.getInput("front")
end

local function getFrontCompCoord() -- the coordinate of the front computer
    rednet.broadcast("locate");
    local id, message = rednet.receive()
    local a, b, c = message:match("(%S+)%s+(%S+)%s+(%S+)")
    return tonumber(a), tonumber(b), tonumber(c)
end

local function getRelativeAngle(pSelf, pFront, target)
    -- 当前朝向的水平方向向量
    local dirSelf = pFront - pSelf
    -- 目标方向的水平方向向量
    local dirTarget = target - pSelf

    -- 使用 atan2 计算 yaw（MC 坐标系：南=0，西为正）
    local yawSelf = math.atan(-dirSelf.x, dirSelf.z)   -- 弧度
    local yawTarget = math.atan(-dirTarget.x, dirTarget.z)

    -- 差值并标准化到 [-π, π]
    local delta = yawTarget - yawSelf
    delta = (delta + math.pi) % (2 * math.pi) - math.pi

    return math.deg(delta)   -- 返回度数，正=右，负=左
end

local function getRelativeDist(pSelf, target)
    return pSelf.length(pSelf - target)
end

local function setLift(output)  -- Output: 15 - max lift force; 0 - no lift force
    relay.setAnalogOutput("top", output)
end

local function turnRight(output)    
    relay.setAnalogOutput("left", 0)
    relay.setAnalogOutput("right", output)
end

local function turnLeft(output)
    relay.setAnalogOutput("left", output)
    relay.setAnalogOutput("right", 0)
end

local function moveForward(output)
    relay.setAnalogOutput("front", output)
end

local function stopHorMove()
    relay.setAnalogOutput("left", 0)
    relay.setAnalogOutput("right", 0)
    relay.setAnalogOutput("front", 0)
end
-- ==============================================
-- Main control loop
-- ==============================================
local function controlTask_hor()
    while true do
        if isDisable() then
            -- manually control
            stopHorMove()
            sleep(DISABLE_SLEEP_TICK)

        else
            -- ==============================================
            -- Main Logic
            -- ==============================================
            local pSelf = vector.new(gps.locate())
            local pFront = vector.new(getFrontCompCoord())
            local angle = getRelativeAngle(pSelf, pFront, target)
            local distance = getRelativeDist(pSelf, target)

            if distance > HOR_ZONE then
                if targetHeight ~= navigationAltitude then
                    targetHeight = navigationAltitude
                end
            elseif targetHeight ~= target.y then
                targetHeight = target.y
            end


            if distance > HOR_ZONE then
                moveForward(15)

                if angle > 0 then
                    if math.abs(angle) >= 90 then
                        turnRight(15)
                    else
                        turnRight(math.floor((angle/90)*15 + 0.5 ))
                    end
                else
                    if math.abs(angle) >= 90 then
                        turnLeft(15)
                    else
                        turnLeft(math.floor((angle/90)*15 + 0.5 ))
                    end
                end
            else
                stopHorMove()
            end

            sleep(HOR_TICK)
        end
    end
end


local function controlTask_y()
    while true do
        if isDisable() then
            -- manually control
            setLift(0)
            sleep(DISABLE_SLEEP_TICK)

        else
            -- ==============================================
            -- Main Logic
            -- ==============================================
            local h = getHeight()
            local v = getVelocity()
            local v_up = -v
            local inFineY_ZONE = false   -- Record whether the previous cycle was in the fine Y_ZONE
            
            if h == nil then h = targetHeight end
            if v == nil then v = 0 end

            local error = targetHeight - h
            local airPressure = getAirPressure()

            -- Calculate base thrust required for hovering (feedforward)
            local base_ratio = GRAVITY / (MAX_ACC_UP * airPressure)
            local base_output = base_ratio * 15  -- Floating point, round at the end
            local output, target_acc

            if math.abs(error) <= Y_ZONE then
                -- ===== Fine Y_ZONE: PD damping control =====
                if not inFineY_ZONE then
                    -- Entering fine Y_ZONE, keep output continuous, do not reset PID history (use independent PD variables)
                    inFineY_ZONE = true
                end

                local desired_vel = KP * error
                local vel_error = desired_vel - v_up   -- Velocity error

                -- Thrust correction = proportional to velocity error (actually equivalent to P-D cascade)
                local correction = KD * vel_error
                local limited_corr = 15 - base_output
                correction = math.max(-limited_corr, math.min(limited_corr, correction))
                -- Total thrust (feedforward + correction)
                output = base_output + correction + (error/Y_ZONE)*2

                -- Debug display
                term.setCursorPos(1, 6)
                term.clearLine()
                term.write(string.format("cor:%6.1f",
                    correction))
            else
                -- ===== Physics prediction braking Y_ZONE =====
                local current_kinetic = v*v/2
                local required_potential = GRAVITY* error
                local final_required_kinetic = 0
                inFineY_ZONE = true

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
            setLift(output)

            sleep(Y_TICK)
        end
    end
end

-- ==============================================
-- User input task
-- ==============================================
local function inputTask()
    while true do
        local id, message = rednet.receive()
        -- Check if message is a string and starts with "target"
        if type(message) == "string" then
            local x, y, z = message:match("^target%s+([%d.-]+)%s+([%d.-]+)%s+([%d.-]+)$")
            if x then
                target = vector.new(tonumber(x), tonumber(y), tonumber(z))
                -- Optional: print confirmation
                term.setTextColor(colors.green)
                print("New target: " .. target)
                term.setTextColor(colors.white)
            end
        end
    end
end

parallel.waitForAny(controlTask_hor, controlTask_y, inputTask)