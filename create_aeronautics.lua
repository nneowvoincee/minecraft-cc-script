-- ==============================================
-- PID Controller with Anti-Windup (English)
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u, out_min, out_max)
    local pid = {
        kp = kp,
        ki = ki,
        kd = kd,
        tick = tick,
        k = 0,
        u = initial_u or 0,
        e = {},
        out_min = out_min or 0,
        out_max = out_max or 15,
    }
    function pid:step(err)
        self.e[self.k] = err
        if self.k == 0 then
            self.e[-1] = 0.0
            self.e[-2] = 0.0
        end

        -- Incremental PID terms
        local p_term = self.kp * (self.e[self.k] - self.e[self.k - 1])
        local i_term = self.ki * self.tick * self.e[self.k]
        local d_term = self.kd * (self.e[self.k] - 2 * self.e[self.k - 1] + self.e[self.k - 2]) / self.tick
        local du = p_term + i_term + d_term

        local u_new = self.u + du

        -- Clamping with basic anti-windup:
        -- If output saturates, do not let the integral term accumulate further.
        if u_new > self.out_max then
            u_new = self.out_max
        elseif u_new < self.out_min then
            u_new = self.out_min
        end

        self.u = u_new
        self.k = self.k + 1
        return self.u
    end
    return pid
end

-- ==============================================
-- Main Program
-- ==============================================
local sensor = peripheral.wrap("top")
if not sensor then
    error("Height sensor not found on top", 0)
end

-- Softer PID gains for asymmetric gravity
local KP = 0.4
local KI = 0.005
local KD = 4.0
local TICK = 0.1

local targetHeight = 100

-- Bumpless start: read current output and invert to internal scale (0 = idle, 15 = max pull)
local startPower = redstone.getAnalogOutput("bottom") or 0
local startPowerInternal = 15 - startPower
local control = Pid.createPid(KP, KI, KD, TICK, startPowerInternal, 0, 15)

-- ===== Background PID control task =====
local function pidTask()
    while true do
        local currentHeight = sensor.getHeight()
        local error = targetHeight - currentHeight
        local output = control:step(error)

        -- Clamp to valid range (already done inside step, but safe to do again)
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end

        output = math.floor(output + 0.5)

        -- Inverted output on bottom: 0 = full pull up, 15 = engine off
        local invertedOutput = 15 - output
        redstone.setAnalogOutput("bottom", invertedOutput)

        sleep(TICK)
    end
end

-- ===== Command input task =====
local function inputTask()
    term.setTextColor(colors.yellow)
    print("PID Altitude Hold (Anti-Windup, Inverted Bottom)")
    print("Target: " .. targetHeight)
    print("Enter new target height (>= -64):")
    term.setTextColor(colors.white)

    while true do
        write("New target: ")
        local input = read()
        local newTarget = tonumber(input)
        if newTarget and newTarget >= -64 then
            targetHeight = newTarget
            print("Target updated to " .. targetHeight .. " m")
        else
            print("Invalid input. Must be a number >= -64.")
        end
    end
end

-- Run both tasks concurrently
parallel.waitForAny(pidTask, inputTask)