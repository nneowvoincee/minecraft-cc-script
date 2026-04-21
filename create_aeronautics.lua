-- ==============================================
-- PID with Dead Zone + Open-Loop Boost
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
local sensor = peripheral.wrap("top")
if not sensor then error("Height sensor not found on top", 0) end

local KP, KI, KD = 2.0, 0.1, 1.0   -- PID 仅在小误差区使用，可相对温和
local TICK = 0.1
local I_LIMIT = 6.0

local targetHeight = 100
local deadZone = 30.0               -- 大误差阈值（可调）

local function getSafeHeight()
    local h = sensor.getHeight()
    return (type(h) == "number") and h or nil
end

local currentHeight = getSafeHeight() or targetHeight
local initialError = targetHeight - currentHeight
local initialU = math.max(0, math.min(15, (initialError * 0.5) + 7.5))
local control = Pid.createPid(KP, KI, KD, TICK, initialU, I_LIMIT)

-- PID 任务（死区 + 开环）
local function pidTask()
    while true do
        local h = getSafeHeight()
        if h ~= nil then
            local error = targetHeight - h

            term.setCursorPos(1, 5)
            term.clearLine()
            term.write(string.format("H:%6.1f T:%6.1f Err:%6.1f u:%5.1f I:%5.1f",
                h, targetHeight, error, control.u, control.i_accum))

            if error > deadZone then
                -- 大幅上升：满推力
                control.u = 15
                control.i_accum = 0
                redstone.setAnalogOutput("bottom", 0)
            elseif error < -deadZone then
                -- 大幅下降：零推力
                control.u = 0
                control.i_accum = 0
                redstone.setAnalogOutput("bottom", 15)
            else
                -- 小误差区：PID 精细调节
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

-- 输入任务
local function inputTask()
    term.setTextColor(colors.yellow)
    print("PID Altitude Hold (Dead Zone + Boost)")
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