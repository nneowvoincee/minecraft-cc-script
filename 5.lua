-- ==============================================
-- 物理参数 —— 请根据你的飞行器填写
-- ==============================================
local FAN_FORCE_UP = 40480.0        -- 风扇满推力时向上的力（牛顿）
local MASS = 6130                 -- 飞行器质量（kg）
local GRAVITY = 11        -- 重力加速度（m/s²），通常无需改动
local MAX_ACC_UP = FAN_FORCE_UP/MASS
-- 下降时没有反向推力，仅靠重力，所以下降方向最大净力 = MASS * GRAVITY

-- 控制参数
DELTA = 2   -- 数字越大，初期加速越快，但可能会冲过头

local PID_ZONE = 20.0             -- 距离目标多少格内启用 PID 精细控制
MAX_CRUISE_SPEED = 200

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
-- 主控制循环
-- ==============================================
local function controlTask()
    while true do
        local h = getHeight()
        local v = getVelocity()
        local v_up = -v

        if h == nil then h = targetHeight end
        if v == nil then v = 0 end

        local error = targetHeight - h

        local output

        if math.abs(error) <= PID_ZONE and false then
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
            local current_kinetic = v*v/2
            local required_potential = GRAVITY* error
            local final_required_kinetic = 0

            if (v_up <= 0) then
                current_kinetic = -current_kinetic
            end
            final_required_kinetic = required_potential*DELTA - current_kinetic

            local target_acc = final_required_kinetic / math.abs(error)

            local airPressure = getAirPressure()
            local ratio = (target_acc + GRAVITY) / (MAX_ACC_UP* airPressure)
            output = math.floor(ratio * 15 + 0.5)

            if output > 15 then output = 15
            elseif output < 0 then output = 0 end
            term.setCursorPos(1, 5)
            term.clearLine()
            term.write(string.format("tar_acc:%2.1f,k:%2.1f,r:%2.2f,o:%2d,E:%2d\n",
            target_acc, final_required_kinetic, ratio, output, error))

            term.setCursorPos(1, 6)
            term.clearLine()
            term.write(string.format("v_up:%2d,h:%2d\n", v_up, h))

        end

        -- 输出到螺旋桨（反相）
        redstone.setAnalogOutput("bottom", 15 - output)

        -- 调试显示
        --term.setCursorPos(1, 5)
        --term.clearLine()
        --term.write(string.format("H:%6.1f T:%6.1f V:%5.2f Err:%6.1f Out:%2d",
        --    h, targetHeight, v, error, output))

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