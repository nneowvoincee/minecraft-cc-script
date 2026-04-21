-- ==============================================
-- 物理参数 —— 请根据你的飞行器填写
-- ==============================================
local FAN_FORCE_UP = 40480.0        -- 风扇满推力时向上的力（牛顿）
local MASS = 6130                 -- 飞行器质量（kg）
local GRAVITY = 11        -- 重力加速度（m/s²），通常无需改动
local MAX_ACC_UP = FAN_FORCE_UP/MASS
-- 下降时没有反向推力，仅靠重力，所以下降方向最大净力 = MASS * GRAVITY

-- 控制参数
local DELTA = 15   -- 数字越大，初期加速越快，但可能会冲过头

local ZONE = 10.0             -- 距离目标多少格内启用精细控制
local TICK = 0.1

-- ==============================================
-- 初始化传感器
-- ==============================================
local heightSensor = peripheral.wrap("top")
local velSensor = peripheral.wrap("right")

if not heightSensor then error("Height sensor not found on top", 0) end
if not velSensor then error("Velocity sensor not found on right", 0) end

local targetHeight = 100

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

        local output, target_acc

        if math.abs(error) <= ZONE then
             local v_abs = math.abs(v_up)

            -- 1. 几乎静止，直接保持
            if v_abs < 0.01 then
                target_acc = 0
            else
                -- 2. 计算在当前速度下，匀减速至零所需净加速度（理论值）
                local a_brake = -0.5 * v_up * v_abs / error   -- 保持符号正确

                -- 3. 限幅到推力可实现的净加速度范围
                local max_up_net = MAX_ACC_UP - GRAVITY   -- 最大向上净加速度
                local max_down_net = -GRAVITY             -- 最大向下净加速度（自由落体）

                -- 4. 方向检查：如果计算出的制动加速度方向与所需方向一致且不超过限幅，则采用
                --    否则，如果正在远离目标，则用最大推力朝向目标
                if error * v_up > 0 then
                    -- 正在朝向目标移动，使用制动加速度并限幅
                    target_acc = math.max(max_down_net, math.min(max_up_net, a_brake))
                else
                    -- 正在远离目标，全力朝目标方向加速
                    target_acc = (error > 0) and max_up_net or max_down_net
                end
            end

        else
            -- ===== 物理预测制动区 =====
            local current_kinetic = v*v/2
            local required_potential = GRAVITY* error
            local final_required_kinetic = 0

            if (v_up <= 0) then
                current_kinetic = -current_kinetic
            end
            final_required_kinetic = required_potential*DELTA - current_kinetic

            target_acc = final_required_kinetic / math.abs(error)
        end

        local airPressure = getAirPressure()
        local ratio = (target_acc + GRAVITY) / (MAX_ACC_UP * airPressure)
        output = math.floor(ratio * 15 + 0.5)

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