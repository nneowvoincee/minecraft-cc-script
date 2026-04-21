-- ==============================================
-- PID Controller with Kinematic Braking
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u, i_limit)
    local pid = {
        k = 0,
        u = initial_u or 0,
        e_prev = 0.0,
        e_prev2 = 0.0,
        i_accum = 0.0,
        i_limit = i_limit or 8.0,
        d_filtered = 0.0,
        d_alpha = 0.3,
    }
    function pid:step(err)
        if self.k == 0 then
            self.e_prev = err
            self.e_prev2 = err
            self.i_accum = 0.0
            self.d_filtered = 0.0
        end

        local p_term = kp * (err - self.e_prev)

        self.i_accum = self.i_accum + ki * tick * err
        if self.i_accum > self.i_limit then self.i_accum = self.i_limit
        elseif self.i_accum < -self.i_limit then self.i_accum = -self.i_limit end

        local raw_d = kd * (err - 2 * self.e_prev + self.e_prev2) / tick
        self.d_filtered = self.d_alpha * raw_d + (1 - self.d_alpha) * self.d_filtered

        local du = p_term + self.i_accum + self.d_filtered

        self.u = self.u + du
        if self.u > 15 then self.u = 15
        elseif self.u < 0 then self.u = 0 end

        self.e_prev2 = self.e_prev
        self.e_prev = err
        self.k = self.k + 1

        return self.u
    end
    return pid
end

-- ==============================================
-- Main Program
-- ==============================================
local heightSensor = peripheral.wrap("top")
local velSensor = peripheral.wrap("right")

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

-- 物理参数（需要根据实际飞行器校准）
local maxThrustAccel = 4.0    -- 最大推力时的加速度 (m/s²) 向上
local gravityAccel = 9.8       -- 重力加速度 (m/s²)
local maxUpAccel = maxThrustAccel - gravityAccel
local maxDownAccel = gravityAccel

local maxSpeed = 15           -- 最大巡航速度 (m/s)

-- PID 参数（速度环）
local KP, KI, KD = 2.5, 0.15, 1.0
local TICK = 0.1
local I_LIMIT = 10.0

local targetHeight = 100

local function getSafeHeight()
    local h = heightSensor.getHeight()
    return (type(h) == "number") and h or nil
end

local function getSpeed()
    local raw = velSensor.getVelocity()
    if type(raw) == "number" then
        return -raw / 10.0   -- 向上为正
    else
        return nil
    end
end

local initialU = 7.5
local control = Pid.createPid(KP, KI, KD, TICK, initialU, I_LIMIT)

-- ===== PID 任务（带运动学制动） =====
local function pidTask()
    while true do
        local h = getSafeHeight()
        local v = getSpeed()

        if h ~= nil and v ~= nil then
            local error = targetHeight - h

            local targetSpeed

            if error > 0 then
                -- 需要向上运动
                -- 计算从当前速度 v 制动到 0 所需的距离（仅当 v > 0 时有效）
                local brakeDist = 0
                if v > 0 then
                    brakeDist = (v * v) / (2 * maxUpAccel)
                end
                if error <= brakeDist then
                    -- 进入减速区，期望速度按 sqrt(2*a*d) 计算，且不超过当前速度（避免瞬间跳变）
                    targetSpeed = math.sqrt(2 * maxUpAccel * error)
                    if targetSpeed > v then targetSpeed = v end   -- 防止计算误差导致加速
                else
                    targetSpeed = maxSpeed
                end
            else
                -- 需要向下运动
                local absErr = -error
                local brakeDist = 0
                if v < 0 then
                    brakeDist = (v * v) / (2 * maxDownAccel)
                end
                if absErr <= brakeDist then
                    targetSpeed = -math.sqrt(2 * maxDownAccel * absErr)
                    if targetSpeed < v then targetSpeed = v end
                else
                    targetSpeed = -maxSpeed
                end
            end

            local speedError = targetSpeed - v

            -- 调试显示
            term.setCursorPos(1, 5)
            term.clearLine()
            term.write(string.format("H:%6.1f T:%6.1f V:%5.2f TV:%5.2f ErrV:%5.2f u:%5.1f",
                h, targetHeight, v, targetSpeed, speedError, control.u))

            local output = control:step(speedError)
            output = math.floor(output + 0.5)
            if output > 15 then output = 15
            elseif output < 0 then output = 0 end

            redstone.setAnalogOutput("bottom", 15 - output)
        else
            term.setCursorPos(1, 5)
            term.write("Sensor offline, waiting...")
        end
        sleep(TICK)
    end
end

-- ===== 输入任务 =====
local function inputTask()
    term.setTextColor(colors.yellow)
    print("Kinematic Braking Altitude Hold")
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

parallel.waitForAny(pidTask, inputTask)