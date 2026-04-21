-- ==============================================
-- Incremental PID Controller
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u)
    local pid = {
        k = 0,
        u = initial_u or 0,
        e = {},
    }
    function pid:step(err)
        self.e[self.k] = err
        if self.k == 0 then
            self.e[-1] = 0.0
            self.e[-2] = 0.0
        end
        local du = kp * (self.e[self.k] - self.e[self.k - 1]) +
                   ki * tick * self.e[self.k] +
                   kd * (self.e[self.k] - 2 * self.e[self.k - 1] + self.e[self.k - 2]) / tick
        self.u = self.u + du
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

local KP, KI, KD = 1.0, 0.017, 2.0
local TICK = 0.1

local targetHeight = 100

-- Read current output on BOTTOM to initialize PID without a jump
local startPower = redstone.getAnalogOutput("bottom") or 0
-- But since logic is inverted, we need to invert it back to PID's internal scale (0=idle, 15=max)
local startPowerInternal = 15 - startPower
local control = Pid.createPid(KP, KI, KD, TICK, startPowerInternal)

-- ===== Background PID task (inverted, bottom output) =====
local function pidTask()
    while true do
        local currentHeight = sensor.getHeight()
        local error = targetHeight - currentHeight
        local output = control:step(error)

        -- Clamp
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end
        output = math.floor(output + 0.5)

        -- Invert for this specific thruster: 0 = full pull, 15 = off
        local invertedOutput = 15 - output

        redstone.setAnalogOutput("bottom", invertedOutput)

        sleep(TICK)
    end
end

-- ===== Input task (unchanged) =====
local function inputTask()
    term.setTextColor(colors.yellow)
    print("PID Altitude Hold Active (Inverted Bottom Output)")
    print("Current target: " .. targetHeight)
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

-- Run both tasks
parallel.waitForAny(pidTask, inputTask)