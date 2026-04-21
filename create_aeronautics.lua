-- ==============================================
-- PID Controller with Velocity Feedforward
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

-- 控制参数
local KP, KI, KD = 3.0, 0.15, 1.5   -- 速度环 PID
local TICK = 0.1
local I_LIMIT = 10.0

local targetHeight = 100

-- 速度规划参数
local maxSpeed = 5.0        -- 最大允许速度 (m/s)
local decelDist = 15.0      -- 开始减速的距离 (m)

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

-- 初始化 PID（基于初始速度误差为0，但也可以基于初始推力估计）
local initialU = 7.5   -- 中等推力起步
local control = Pid.createPid(KP, KI, KD, TICK, initialU, I_LIMIT)

-- ===== PID 任务（速度环） =====
local function pidTask()
    while true do
        local h = getSafeHeight()
        local v = getSpeed()

        if h ~= nil and v ~= nil then
            local error = targetHeight - h

            -- 计算期望速度（梯形曲线）
            local targetSpeed
            local absErr = math.abs(error)
            if absErr > decelDist then
                targetSpeed = (error > 0) and maxSpeed or -maxSpeed
            else
                targetSpeed = (error / decelDist) * maxSpeed
            end

            local speedError = targetSpeed - v

            -- 调试显示
            term.setCursorPos(1, 5)
            term.clearLine()
            term.write(string.format("H:%6.1f T:%6.1f V:%5.2f TV:%5.2f ErrV:%5.2f u:%5.1f",
                h, targetHeight, v, targetSpeed, speedError, control.u))

            -- 速度 PID 计算推力
            local output = control:step(speedError)
            output = math.floor(output + 0.5)
            if output > 15 then output = 15
            elseif output < 0 then output = 0 end

            -- 输出（注意反转）
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
    print("Velocity-based PID Altitude Hold")
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