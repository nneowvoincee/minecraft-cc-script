-- ==============================================
-- 物理参数 —— 请根据你的飞行器填写
-- ==============================================
local FAN_FORCE_UP = 40480.0        -- 风扇满推力时向上的力（牛顿）
local MASS = 6130                 -- 飞行器质量（kg）
local GRAVITY = 11        -- 重力加速度（m/s²），通常无需改动

-- 下降时没有反向推力，仅靠重力，所以下降方向最大净力 = MASS * GRAVITY

-- 控制参数
local PID_ZONE = 20.0             -- 距离目标多少格内启用 PID 精细控制
local MAX_CRUISE_SPEED = 300      -- 最大巡航速度（m/s），与 getVelocity 同量级

-- PID 参数（用于速度环）
local KP, KI, KD = 2.5, 0.15, 1.0
local TICK = 0.1
local I_LIMIT = 10.0

-- ==============================================
-- 初始化传感器
-- ==============================================
local heightSensor = peripheral.wrap("top")
local velSensor = peripheral.wrap("right")

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

local targetHeight = 100

-- ==============================================
-- PID 控制器（速度环）
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

local control = Pid.createPid(KP, KI, KD, TICK, 7.5, I_LIMIT)

-- ==============================================
-- 辅助函数：获取高度、速度、气压
-- ==============================================
local function getHeight()
    local h = heightSensor.getHeight()
    return (type(h) == "number") and h or nil
end

local function getVelocity()
    local v = velSensor.getVelocity()
    return (type(v) == "number") and v or nil
end

local function getAirPressure()
    local p = heightSensor.getAirPressure()
    return (type(p) == "number") and p or 1.0   -- 默认 1.0
end

-- ==============================================
-- 物理计算：根据推力等级（0~15）计算净加速度（向上为正）
-- ==============================================
local function computeNetAcceleration(throttle, airPressure)
    -- throttle: 0~15，0为全推力，15为零推力
    local thrustRatio = 1.0 - (throttle / 15.0)   -- 映射：15->0, 0->1
    local thrustForce = FAN_FORCE_UP * thrustRatio * airPressure
    local netForce = thrustForce - MASS * GRAVITY
    return netForce / MASS   -- 向上加速度 (m/s²)
end

-- 获取当前最大可用加速度（全推力时）和最大反向加速度（完全关闭推力时）
local function getMaxAccelerations()
    local airPressure = getAirPressure()
    local maxUpAccel = computeNetAcceleration(0, airPressure)    -- 全推力
    local maxDownAccel = -computeNetAcceleration(15, airPressure) -- 零推力，只有重力向下，注意符号：向下为正？不，统一向上为正，则零推力加速度 = -GRAVITY
    -- 实际上零推力时净加速度 = 0 - MASS*GRAVITY = -GRAVITY，与气压无关
    maxDownAccel = GRAVITY   -- 下降时最大减速能力（向上为正的减速度）？不，这里我们关注制动能力：
    -- 当需要向下运动时，能提供的最大向上制动力就是最大推力，所以向下运动时的最大减速度（向上方向）为 maxUpAccel（全推力）
    -- 当需要向上运动时，能提供的最大向下减速度就是重力 GRAVITY（关闭推力）
    return maxUpAccel, GRAVITY
end

-- ==============================================
-- 主控制循环
-- ==============================================
local function controlTask()
    while true do
        local h = getHeight()
        local v = getVelocity()
        if h == nil then h = targetHeight end
        if v == nil then v = 0 end

        local error = targetHeight - h   -- 向上为正

        local output

        if math.abs(error) <= PID_ZONE then
            -- ===== PID 精细区 =====
            -- 计算期望速度（线性减速至0）
            local targetSpeed = (error / PID_ZONE) * MAX_CRUISE_SPEED
            local speedError = targetSpeed - v
            output = control:step(speedError)
            output = math.floor(output + 0.5)
            if output > 15 then output = 15
            elseif output < 0 then output = 0 end
        else
            -- ===== 物理预测制动区 =====
            local maxUpAccel, maxDownDecel = getMaxAccelerations()
            -- 注意：maxDownDecel 实际是重力加速度，但当我们向上运动需要减速时，最大向下加速度就是 GRAVITY
            -- 向下运动需要减速时，最大向上加速度是 maxUpAccel

            local targetSpeed

            if error > 0 then
                -- 目标在上方，需要向上运动
                -- 当前速度 v 可能为正（向上）或负（向下，但我们要向上，所以会先反向加速）
                -- 我们关心的是从当前速度 v 到 0 所需的制动距离（仅当 v > 0 时需要考虑提前减速）
                local brakeDist = 0
                if v > 0 then
                    -- 向上运动时，可用最大向下加速度为 GRAVITY（关闭推力）
                    brakeDist = (v * v) / (2 * GRAVITY)
                end
                if error <= brakeDist then
                    -- 进入减速区，期望速度按 sqrt(2*a*d) 计算，a 为最大向下加速度（GRAVITY）
                    targetSpeed = math.sqrt(2 * GRAVITY * error)
                    if targetSpeed > v then targetSpeed = v end
                else
                    targetSpeed = MAX_CRUISE_SPEED
                end
            else
                -- 目标在下方，需要向下运动
                local absErr = -error
                local brakeDist = 0
                if v < 0 then
                    -- 向下运动时，可用最大向上加速度为 maxUpAccel（全推力）
                    brakeDist = (v * v) / (2 * maxUpAccel)
                end
                if absErr <= brakeDist then
                    targetSpeed = -math.sqrt(2 * maxUpAccel * absErr)
                    if targetSpeed < v then targetSpeed = v end
                else
                    targetSpeed = -MAX_CRUISE_SPEED
                end
            end

            local speedError = targetSpeed - v

            -- 将速度误差直接映射到推力：比例控制 + 前馈，简单但有效
            -- 也可用 PID，但这里为了响应迅速，使用 P 加死区
            local kVelToThrottle = 1.5   -- 增益，可调
            local rawOutput = speedError * kVelToThrottle
            -- 增加一个前馈：目标速度的符号决定基础推力方向
            if targetSpeed > 0.5 then
                rawOutput = rawOutput + 10   -- 需要向上，加偏置
            elseif targetSpeed < -0.5 then
                rawOutput = rawOutput - 5    -- 需要向下，减偏置（但推力不能为负）
            end
            output = math.floor(rawOutput + 0.5)
            if output > 15 then output = 15
            elseif output < 0 then output = 0 end

            -- 避免 PID 内部状态在切换时混乱，可选择性重置（非必须）
            -- control.u = output
        end

        -- 输出到螺旋桨（反相）
        redstone.setAnalogOutput("bottom", 15 - output)

        -- 调试显示
        term.setCursorPos(1, 5)
        term.clearLine()
        term.write(string.format("H:%6.1f T:%6.1f V:%5.2f Err:%6.1f Out:%2d",
            h, targetHeight, v, error, output))

        sleep(TICK)
    end
end

-- ==============================================
-- 用户输入任务
-- ==============================================
local function inputTask()
    term.setTextColor(colors.yellow)
    print("Hybrid Controller (PID + Physics Brake)")
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

parallel.waitForAny(controlTask, inputTask)