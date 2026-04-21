-- ==============================================
-- Incremental PID Controller
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u)
    local pid = {
        k = 0,
        u = initial_u or 0,   -- avoid nil initial output
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
-- Main Program: Altitude Hold with Live Input
-- ==============================================

-- 1. Hardware initialization
local sensor = peripheral.wrap("top")
if not sensor then
    error("Height sensor not found on top", 0)
end

-- 2. PID parameters (tune as needed)
local KP, KI, KD = 1.0, 0.017, 2.0
local TICK = 0.1   -- control loop interval in seconds

-- 3. Initial target height (must be >= -64)
local targetHeight = 100

-- Read current redstone output for bumpless PID start
local startPower = redstone.getAnalogOutput("right") or 0
local control = Pid.createPid(KP, KI, KD, TICK, startPower)

-- 4. Prepare terminal display
term.clear()
term.setCursorPos(1, 1)
term.setCursorBlink(true)

-- Input buffer
local inputBuffer = ""
local inputPrompt = ">>> Enter new target (>= -64): "

-- Function to refresh the fixed three lines
local function updateScreen(currentHeight)
    term.setCursorPos(1, 1)
    term.clearLine()
    print("Target Height: " .. targetHeight .. " m")

    term.setCursorPos(1, 2)
    term.clearLine()
    print("Current Height: " .. currentHeight .. " m")

    term.setCursorPos(1, 3)
    term.clearLine()
    write(inputPrompt .. inputBuffer)
end

-- Initial display
local currentHeight = sensor.getHeight()
updateScreen(currentHeight)

-- 5. Start periodic timer for PID loop
local timerID = os.startTimer(TICK)

-- 6. Main event loop
while true do
    local event, p1, p2, p3 = os.pullEventRaw()

    if event == "timer" and p1 == timerID then
        -- ========== Timer: run PID control ==========
        timerID = os.startTimer(TICK)

        currentHeight = sensor.getHeight()
        local error = targetHeight - currentHeight
        local output = control:step(error)

        -- Clamp to analog redstone range [0, 15]
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end
        output = math.floor(output + 0.5)

        redstone.setAnalogOutput("right", output)
        updateScreen(currentHeight)

    elseif event == "key" then
        -- ========== Keyboard input ==========
        local key = p1
        if key == keys.enter then
            local newTarget = tonumber(inputBuffer)
            if newTarget and newTarget >= -64 then
                targetHeight = newTarget
            end
            inputBuffer = ""
            updateScreen(currentHeight)

        elseif key == keys.backspace then
            if #inputBuffer > 0 then
                inputBuffer = inputBuffer:sub(1, -2)
                updateScreen(currentHeight)
            end

        elseif key == keys.delete then
            inputBuffer = ""
            updateScreen(currentHeight)

        else
            local char = keys.getName(key)
            -- Allow digits, minus, decimal point
            if char and char:match("^[%d%-%.]$") then
                if char == "-" and #inputBuffer == 0 then
                    inputBuffer = inputBuffer .. char
                elseif char:match("%d") or char == "." then
                    inputBuffer = inputBuffer .. char
                end
                updateScreen(currentHeight)
            end
        end

    elseif event == "terminate" then
        term.clear()
        term.setCursorPos(1, 1)
        term.setCursorBlink(false)
        print("Program terminated.")
        break
    end
end