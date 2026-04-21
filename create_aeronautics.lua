-- ==============================================
-- Incremental PID with Integral Clamping
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u, i_limit)
    local pid = {
        k = 0,
        u = initial_u or 0,
        e_prev = 0.0,
        e_prev2 = 0.0,
        i_accum = 0.0,
        i_limit = i_limit or 5.0,   -- 默认积分限幅 ±5
    }
    function pid:step(err)
        if self.k == 0 then
            self.e_prev = err
            self.e_prev2 = err
            self.i_accum = 0.0
        end

        -- 比例增量
        local p_term = kp * (err - self.e_prev)

        -- 积分累积（带限幅）
        self.i_accum = self.i_accum + ki * tick * err
        if self.i_accum > self.i_limit then self.i_accum = self.i_limit
        elseif self.i_accum < -self.i_limit then self.i_accum = -self.i_limit end

        -- 微分项
        local d_term = kd * (err - 2 * self.e_prev + self.e_prev2) / tick

        local du = p_term + self.i_accum + d_term

        self.u = self.u + du

        -- 最终输出钳位
        if self.u > 15 then self.u = 15
        elseif self.u < 0 then self.u = 0 end

        -- 更新历史
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
local sensor = peripheral.wrap("top")
if not sensor then error("Height sensor not found on top", 0) end

-- PID 参数（可根据实际响应调整）
local KP, KI, KD = 2.5, 0.1, 4.0
local TICK = 0.1
local I_LIMIT = 6.0        -- 积分限幅，可调

local targetHeight = 100
local descentThreshold = -8

local function getSafeHeight()
    local h = sensor.getHeight()
    return (type(h) == "number") and h or nil
end

-- 初始化 PID
local currentHeight = getSafeHeight()
if currentHeight == nil then currentHeight = targetHeight end
local initialError = targetHeight - currentHeight
local initialU = math.max(0, math.min(15, (initialError * 0.5) + 7.5))
local control = Pid.createPid(KP, KI, KD, TICK, initialU, I_LIMIT)

-- ===== PID 任务 =====
local function pidTask()
    while true do
        local h = getSafeHeight()
        if h ~= nil then
            local error = targetHeight - h

            -- 显示调试信息（第5行）
            term.setCursorPos(1, 5)
            term.clearLine()
            term.write(string.format("H:%6.1f T:%6.1f Err:%6.1f u:%5.1f I:%5.1f",
                h, targetHeight, error, control.u, control.i_accum))

            -- 大幅下降强制关停
            if error < descentThreshold then
                control.u = 0
                control.i_accum = 0
                redstone.setAnalogOutput("bottom", 15)
            else
                local output = control:step(error)
                output = math.floor(output + 0.5)
                if output > 15 then output = 15
                elseif output < 0 then output = 0 end
                redstone.setAnalogOutput("bottom", 15 - output)
            end
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
    print("PID Altitude Hold (Integral Clamping)")
    print("Target: " .. targetHeight)
    print("Enter new target (>= -64):")
    term.setTextColor(colors.white)

    while true do
        write("New target: ")
        local input = read()
        local n = tonumber(input)
        if n and n >= -64 then
            targetHeight = n
            print("Target updated to " .. targetHeight)
        else
            print("Invalid, must be >= -64")
        end
    end
end

parallel.waitForAny(pidTask, inputTask)