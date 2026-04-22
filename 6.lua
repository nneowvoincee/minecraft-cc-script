-- ==============================================
-- 物理参数 —— 请根据你的飞行器填写
-- ==============================================
local FAN_FORCE_UP = 11068.0        -- 风扇满推力时向上的力（牛顿）
local MASS = 887                 -- 飞行器质量（kg）
local GRAVITY = 11        -- 重力加速度（m/s²），通常无需改动
local MAX_ACC_UP = FAN_FORCE_UP/MASS
-- 下降时没有反向推力，仅靠重力，所以下降方向最大净力 = MASS * GRAVITY

-- 控制参数
local DELTA = 15   -- 数字越大，初期加速越快，但可能会冲过头

-- pid
local KP, KD = 0.3, 1.2

local ZONE = 50.0             -- 距离目标多少格内启用精细控制
local TICK = 0.1


-- ==============================================
-- 初始化传感器
-- ==============================================
local heightSensor = peripheral.wrap("top")
local velSensor = peripheral.wrap("right")

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

local targetHeight = 200

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
        local inFineZone = false   -- 记录上一周期是否在精细区内

        if h == nil then h = targetHeight end
        if v == nil then v = 0 end

        local error = targetHeight - h
        local airPressure = getAirPressure()

        -- 计算悬停所需的基础推力（前馈）
        local base_ratio = GRAVITY / (MAX_ACC_UP * airPressure)
        local base_output = base_ratio * 15  -- 浮点数，最后再取整
        local output, target_acc

        if math.abs(error) <= ZONE then
            -- ===== 精细区：PD 阻尼控制 =====
            if not inFineZone then
                -- 进入精细区，保持输出连续，不清零 PID 历史（使用独立 PD 变量）
                inFineZone = true
                lastError = error
            end

            local desired_vel = KP * error
            local vel_error = desired_vel - v_up   -- 速度误差

            -- 推力修正量 = 速度误差的比例（实际相当于 P-D 串联）
            local correction = KD * vel_error

            -- 合成总推力（前馈 + 修正）
            output = base_output + correction

             --调试显示
            term.setCursorPos(1, 6)
            term.clearLine()
            term.write(string.format("cor:%6.1f",
                correction))
        else
            -- ===== 物理预测制动区 =====
            local current_kinetic = v*v/2
            local required_potential = GRAVITY* error
            local final_required_kinetic = 0
            inFineZone = true

            if (v_up <= 0) then
                current_kinetic = -current_kinetic
            end
            final_required_kinetic = required_potential*DELTA - current_kinetic

            target_acc = final_required_kinetic / math.abs(error)
            local airPressure = getAirPressure()
            local ratio = (target_acc + GRAVITY) / (MAX_ACC_UP * airPressure)
            output = ratio * 15
        end

        output = math.floor(output + 0.5)
        if output > 15 then output = 15
        elseif output < 0 then output = 0 end

        -- 输出到螺旋桨（反相）
        redstone.setAnalogOutput("bottom", 15 - output)

         --调试显示
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