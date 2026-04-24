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
local heightSensor = peripheral.warp("left")
local vecSpeedSensor = peripheral.wrap("right")
local horSpeedSensor1 = peripheral.wrap("bottom")
local horSpeedSensor2 = peripheral.wrap("top")

if not heightSensor then error("Height sensor not found on left", 0) end
if not vecSpeedSensor then error("vecSpeedSensor not found on right", 0) end
if not horSpeedSensor1 then error("horSpeedSensor1 not found on bottom", 0) end
if not horSpeedSensor2 then error("horSpeedSensor2 not found on top", 0) end

-- modem + gps
peripheral.find("modem", rednet.open)
local x, y, z = gps.locate()
if x == nil then error("GPS is not set up.", 0) end
local target = vector.new(x,y,z)

local targetHeight = y  -- set current height as target height (don't move initially

-- redstone relay, for outputting signal
local relay = peripheral.find("redstone_relay")
if relay == nil then error("Missing redstone relay", 0) end

-- display
local monitor = peripheral.find("monitor")
if monitor == nil then error("Missing monitor", 0) end
term.redirect(monitor)

-- "global" variable shared between functions
local current_h, current_v, current_position, front_position, target_angle

-- ==============================================
-- Helper functions: get height, velocity, air pressure
-- ==============================================
local function getHeight()
    local h = heightSensor.getHeight()
    return (type(h) == "number") and h or nil
end

local function getVecVelocity()
    local v = vecSpeedSensor.getVelocity()
    return (type(v) == "number") and v or nil
end

local function getHorVelocity()
    local v1 = horSpeedSensor1.getVelocity()
    local v2 = horSpeedSensor2.getVelocity()
    return vector.new(v1/GRAVITY, 0, v2/GRAVITY)
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

local function getRelativeAngle(pSelf, front_position, target)
    -- 当前朝向的水平方向向量
    local dirSelf = front_position - pSelf
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
    relay.setAnalogOutput("bottom", output)
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
            current_position = vector.new(gps.locate())
            front_position = vector.new(getFrontCompCoord())
            target_angle = getRelativeAngle(current_position, front_position, target)
            local distance = getRelativeDist(current_position, target)

            if distance > HOR_ZONE then
                if targetHeight ~= navigationAltitude then
                    targetHeight = navigationAltitude
                end
            elseif targetHeight ~= target.y then
                targetHeight = target.y
            end


            if distance > HOR_ZONE then
                moveForward(15)

                if target_angle > 0 then
                    if math.abs(target_angle) >= 90 then
                        turnRight(15)
                    else
                        turnRight(math.floor((target_angle/90)*15 + 0.5 ))
                    end
                else
                    if math.abs(target_angle) >= 90 then
                        turnLeft(15)
                    else
                        turnLeft(math.floor((target_angle/90)*15 + 0.5 ))
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
            current_h = getHeight()
            current_v = -getVecVelocity()
            local inFineY_ZONE = false   -- Record whether the previous cycle was in the fine Y_ZONE
            
            if current_h == nil then current_h = targetHeight end
            if current_v == nil then current_v = 0 end

            local error = targetHeight - current_h
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
                local vel_error = desired_vel - current_v   -- Velocity error

                -- Thrust correction = proportional to velocity error (actually equivalent to P-D cascade)
                local correction = KD * vel_error
                local limited_corr = 15 - base_output
                correction = math.max(-limited_corr, math.min(limited_corr, correction))
                -- Total thrust (feedforward + correction)
                output = base_output + correction + (error/Y_ZONE)*2

            else
                -- ===== Physics prediction braking Y_ZONE =====
                local current_kinetic = current_v*current_v/2
                local required_potential = GRAVITY* error
                local final_required_kinetic = 0
                inFineY_ZONE = true

                if (current_v <= 0) then
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
                -- term.setTextColor(colors.green)
                -- print("New target: " .. target.x .. " " .. target.y .. " " .. target.z)
                -- term.setTextColor(colors.white)
            end
        end
    end
end

-- ==============================================
-- *** NEW: Display task (runs every 0.5 seconds) ***
-- ==============================================
local function displayTask()
    while true do
        local w, h = monitor.getSize()
        if not w then w, h = 20, 10 end   -- fallback

        term.clear()
        term.setCursorPos(1, 1)

        -- 1. Current coordinates
        if current_position then
            term.write(string.format("Pos: %.1f, %.1f, %.1f", current_position.x, current_position.y, current_position.z))
        else
            term.write("Pos: N/A")
        end

        -- 2. Horizontal speed (local frame: forward / right)
        local v1_raw = horSpeedSensor1.getVelocity()
        local v2_raw = horSpeedSensor2.getVelocity()
        local v_fwd = (type(v1_raw) == "number" and v1_raw or 0) / GRAVITY
        local v_rgt = (type(v2_raw) == "number" and v2_raw or 0) / GRAVITY
        term.setCursorPos(1, 2)
        term.write(string.format("HSpd: Fwd %.2f, Rgt %.2f", v_fwd, v_rgt))

        -- 3. ETA
        local eta_str = "NaN"
        if current_position then
            local dist = getRelativeDist(current_position, target)
            if dist and dist > 0 then
                local toTarget = target - current_position
                local hDir = vector.new(toTarget.x, 0, toTarget.z)   -- horizontal only
                local len = hDir:length()
                if len > 0 then
                    hDir = hDir / len
                    -- world velocity from ship‑local speeds and yaw
                    local yaw_rad = math.rad(current_yaw or 0)
                    local fwd_world = vector.new(-math.sin(yaw_rad), 0, math.cos(yaw_rad))
                    local rgt_world = vector.new(-math.cos(yaw_rad), 0, -math.sin(yaw_rad))
                    local world_vel = fwd_world * v_fwd + rgt_world * v_rgt
                    local proj = world_vel:dot(hDir)
                    if proj > 0 then
                        eta_str = string.format("%.1f s", dist / proj)
                    end
                end
            end
        end
        term.setCursorPos(1, 3)
        term.write("ETA: " .. eta_str)

        -- 4. Green dot for target direction (on monitor edge)
        if target_angle ~= nil then
            local cx = math.floor(w/2) + 1
            local cy = math.floor(h/2) + 1
            local rad = math.rad(target_angle)
            local dirx = math.sin(rad)
            local diry = -math.cos(rad)   -- forward = up on screen
            -- scale to reach border
            local sx = (w/2) / (math.abs(dirx) + 0.001)
            local sy = (h/2) / (math.abs(diry) + 0.001)
            local s = math.min(sx, sy)
            local dx = dirx * s
            local dy = diry * s
            local dotx = math.max(1, math.min(w, math.floor(cx + dx + 0.5)))
            local doty = math.max(1, math.min(h, math.floor(cy + dy + 0.5)))
            term.setCursorPos(dotx, doty)
            term.setTextColor(colors.green)
            term.write("O")
            term.setTextColor(colors.white)
        end

        -- 5. Arrow from centre in direction of horizontal movement (local)
        local cx = math.floor(w/2) + 1
        local cy = math.floor(h/2) + 1
        local max_spd = math.max(math.abs(v_fwd), math.abs(v_rgt), 0.01)
        local scale = math.min(w/2, h/2) / max_spd
        local ax = v_rgt * scale   -- right is +x on screen
        local ay = -v_fwd * scale  -- forward is -y (up)
        local ex = math.max(1, math.min(w, math.floor(cx + ax + 0.5)))
        local ey = math.max(1, math.min(h, math.floor(cy + ay + 0.5)))
        local steps = math.max(math.abs(ex - cx), math.abs(ey - cy))
        if steps > 0 then
            for i = 0, steps do
                local px = math.floor(cx + (ex - cx) * i / steps + 0.5)
                local py = math.floor(cy + (ey - cy) * i / steps + 0.5)
                if px >= 1 and px <= w and py >= 1 and py <= h then
                    term.setCursorPos(px, py)
                    if i == steps then
                        term.write(">")
                    else
                        term.write("*")
                    end
                end
            end
        end

        sleep(0.5)
    end
end

parallel.waitForAny(controlTask_hor, controlTask_y, inputTask, displayTask)