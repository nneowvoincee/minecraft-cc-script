-- ==============================================
-- Incremental PID Controller with Anti-Windup
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

        -- 微分项
        local d_term = kd * (err - 2 * self.e_prev + self.e_prev2) / tick

        -- 条件积分：只有误差小于 10 米时才累积积分，避免大误差下的积分饱和
        local i_term = 0
        if math.abs(err) < 10 then
            i_term = ki * tick * err
        end

        local du = kp * (err - self.e_prev) + i_term + d_term

        self.u = self.u + du

        -- 输出钳位
        if self.u > 15 then self.u = 15
        elseif self.u < 0 then self.u = 0 end

        -- 更新误差历史
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

-- 推荐参数：稍激进，但通过条件积分避免超调
local KP, KI, KD = 3.0, 0.08, 4.0
local TICK = 0.1

-- 共享变量：两个协程都能读写
local targetHeight = 100
local descentThreshold = -8   -- 需要下降超过8米时直接关闭螺旋桨

-- 获取高度（带保护）
local function getSafeHeight()
    local h = sensor.getHeight()
    if type(h) == "number" then return h else return nil end
end

-- 初始化 PID 内部状态，避免首次跳变
local currentHeight = getSafeHeight()
if currentHeight == nil then
    currentHeight = targetHeight
    print("Warning: Could not read initial height. Assuming target height.")
end

local initialError = targetHeight - currentHeight
local initialU = math.max(0, math.min(15, (initialError * 0.5) + 7.5))

local control = Pid.createPid(KP, KI, KD, TICK, initialU)

-- ===== PID 控制任务 =====
local function pidTask()
    while true do
        local h = getSafeHeight()
        if h ~= nil then
            local error = targetHeight - h

            -- 调试信息显示在固定位置（第5行开始，避免覆盖输入区）
            term.setCursorPos(1, 5)
            term.clearLine()
            term.write(string.format("Height: %6.1f  Target: %6.1f  Err: %6.1f  u: %5.1f", h, targetHeight, error, control.u))

            -- 大幅下降时强制关闭螺旋桨，快速下落
            if error < descentThreshold then
                control.u = 0   -- 重置内部状态，防止积分记忆
                redstone.setAnalogOutput("bottom", 15)   -- 15 = 关闭螺旋桨（0推力）
            else
                local output = control:step(error)
                output = math.floor(output + 0.5)
                if output > 15 then output = 15
                elseif output < 0 then output = 0 end

                local invertedOutput = 15 - output
                redstone.setAnalogOutput("bottom", invertedOutput)
            end
        else
            term.setCursorPos(1, 5)
            term.clearLine()
            term.write("Sensor offline, waiting...")
        end
        sleep(TICK)
    end
end

-- ===== 用户输入任务 =====
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
            -- 更新后立即显示新目标（调试信息行将在下一周期刷新）
        else
            print("Invalid input. Must be a number >= -64.")
        end
    end
end

-- 启动两个并行任务
parallel.waitForAny(pidTask, inputTask)