-- ==============================================
-- Incremental PID Controller (Improved)
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u)
    local pid = {
        k = 0,
        u = initial_u or 0,
        e_prev = 0.0,
        e_prev2 = 0.0,
    }
    function pid:step(err)
        if self.k == 0 then
            self.e_prev = err
            self.e_prev2 = err
        end

        local du = kp * (err - self.e_prev) +
                   ki * tick * err +
                   kd * (err - 2 * self.e_prev + self.e_prev2) / tick

        self.u = self.u + du

        -- Clamp internal state to prevent integral windup
        if self.u > 15 then self.u = 15
        elseif self.u < 0 then self.u = 0 end

        -- Update error history
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
if not sensor then
    error("Height sensor not found on top", 0)
end

local KP, KI, KD = 1.0, 0.017, 2.0
local TICK = 0.1

local targetHeight = 100

-- ===== Robust initialization to avoid output jump =====
local function getSafeHeight()
    local h = sensor.getHeight()
    if type(h) == "number" then return h else return nil end
end

local currentHeight = getSafeHeight()
if currentHeight == nil then
    currentHeight = targetHeight
    print("Warning: Could not read initial height. Assuming target height.")
end

-- Estimate initial PID output based on error
local initialError = targetHeight - currentHeight
local initialU = math.max(0, math.min(15, (initialError * 0.5) + 7.5))

local control = Pid.createPid(KP, KI, KD, TICK, initialU)

-- ===== Background PID task (inverted, bottom output) =====
local function pidTask()
    while true do
        local h = getSafeHeight()
        if h ~= nil then
            local error = targetHeight - h
            local output = control:step(error)

            -- Clamp and round to integer
            output = math.floor(output + 0.5)
            if output > 15 then output = 15
            elseif output < 0 then output = 0 end

            -- Invert for this specific thruster: 0 = full pull, 15 = off
            local invertedOutput = 15 - output
            redstone.setAnalogOutput("bottom", invertedOutput)
        end
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