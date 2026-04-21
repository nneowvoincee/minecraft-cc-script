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
local TICK = 0.1   -- control interval in seconds

-- Shared variable for target height (can be updated by input task)
local targetHeight = 100

-- Initialize PID with current redstone output (bumpless start)
local startPower = redstone.getAnalogOutput("right") or 0
local control = Pid.createPid(KP, KI, KD, TICK, startPower)

-- ===== Task 1: Background PID control loop =====
local function pidTask()
    while true do
        local currentHeight = sensor.getHeight()
        local error = targetHeight - currentHeight
        local output = control:step(error)

        -- Clamp to [0, 15]
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end
        output = math.floor(output + 0.5)

        redstone.setAnalogOutput("right", output)

        -- Sleep to maintain control frequency
        sleep(TICK)
    end
end

-- ===== Task 2: Command line input =====
local function inputTask()
    term.setTextColor(colors.yellow)
    print("PID Altitude Hold Active")
    print("Current target: " .. targetHeight)
    print("Enter a new target height (>= -64) and press Enter.")
    print("-------------------------------------------")
    term.setTextColor(colors.white)

    while true do
        write("New target: ")
        local input = read()   -- Blocking read, but parallel handles it
        local newTarget = tonumber(input)
        if newTarget and newTarget >= -64 then
            targetHeight = newTarget
            print("Target updated to " .. targetHeight .. " m")
        else
            print("Invalid input. Must be a number >= -64.")
        end
    end
end

-- ===== Run both tasks concurrently =====
parallel.waitForAny(pidTask, inputTask)