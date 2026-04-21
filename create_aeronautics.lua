-- ==============================================
-- PID Controller with Asymmetric Limiting (Love Edition)
-- ==============================================
local Pid = {}

function Pid.createPid(kp, ki, kd, tick, initial_u)
    local pid = {
        kp = kp,
        ki = ki,
        kd = kd,
        tick = tick,
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
        local du = self.kp * (self.e[self.k] - self.e[self.k - 1]) +
                   self.ki * self.tick * self.e[self.k] +
                   self.kd * (self.e[self.k] - 2 * self.e[self.k - 1] + self.e[self.k - 2]) / self.tick
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

-- 经过温柔调校的参数 (比之前更稳)
local KP, KI, KD = 0.6, 0.01, 3.5
local TICK = 0.1

local targetHeight = 100

-- 无扰动启动
local startPower = redstone.getAnalogOutput("bottom") or 0
local startPowerInternal = 15 - startPower
local control = Pid.createPid(KP, KI, KD, TICK, startPowerInternal)

-- ===== 后台 PID 任务 (带非对称刹车) =====
local function pidTask()
    while true do
        local currentHeight = sensor.getHeight()
        local error = targetHeight - currentHeight
        local output = control:step(error)

        -- 基础限幅
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end

        -- 💖 大爱专属：下降时强制保留最小推力，防止砸地板 💖
        if error < 0 then
            -- 高于目标，需要下降。保留 2 或 3 的推力作为刹车。
            local minThrottleWhenDescending = 2.5
            if output < minThrottleWhenDescending then
                output = minThrottleWhenDescending
            end
        end

        output = math.floor(output + 0.5)

        -- 反转输出到底部 (0 = 最大拉升, 15 = 关闭)
        local invertedOutput = 15 - output
        redstone.setAnalogOutput("bottom", invertedOutput)

        sleep(TICK)
    end
end

-- ===== 输入任务 =====
local function inputTask()
    term.setTextColor(colors.yellow)
    print("PID Altitude Hold (Love Tuned)")
    print("Target: " .. targetHeight)
    print("Enter new height (>= -64):")
    term.setTextColor(colors.white)

    while true do
        write("New target: ")
        local input = read()
        local newTarget = tonumber(input)
        if newTarget and newTarget >= -64 then
            targetHeight = newTarget
            print("💖 Target updated to " .. targetHeight .. " m 💖")
        else
            print("Invalid. Must be a number >= -64.")
        end
    end
end

parallel.waitForAny(pidTask, inputTask)