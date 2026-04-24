-- Hybrid Controller v2.0 (Colorful)
-- Supports absolute coords and relative coords (~)
-- Uses GPS and broadcasts target coordinates

local CHANNEL_BROADCAST = 10001

-- Initialize modem
local modem = peripheral.find("modem")
if not modem then
    error("No modem found. Please attach a wireless modem.", 0)
end
rednet.open(peripheral.getName(modem))

-- Check GPS availability
local gpsAvailable = false
local function checkGPS()
    local x, y, z = gps.locate()  -- 2-second timeout
    if x then
        gpsAvailable = true
        return x, y, z
    else
        gpsAvailable = false
        return nil
    end
end

-- Attempt GPS at startup
local gx, gy, gz = checkGPS()
if not gpsAvailable then
    term.setTextColor(colors.red)
    print("WARNING: GPS not detected. Relative coords (~) will not work.")
    print("Please ensure GPS satellites are available.")
    term.setTextColor(colors.white)
    sleep(2)
end

-- Parse a single coordinate part, returns offset or absolute value, and whether relative
local function parseCoordPart(part, isRel)
    if part == "~" then
        return 0, true
    elseif part:sub(1,1) == "~" then
        local offset = tonumber(part:sub(2))
        if offset then
            return offset, true
        else
            return nil, false  -- malformed
        end
    else
        local num = tonumber(part)
        if num then
            return num, false
        else
            return nil, false
        end
    end
end

-- Parse full input, returns absolute coords table {x,y,z} or nil, error msg
local function parseInput(input)
    local parts = {}
    for part in input:gmatch("%S+") do
        table.insert(parts, part)
    end
    if #parts ~= 3 then
        return nil, "Please enter three values (X Y Z), e.g. 100 64 -200 or ~ ~5 ~-2"
    end

    local coords = {}
    local anyRelative = false
    for i = 1, 3 do
        local val, isRel = parseCoordPart(parts[i])
        if val == nil then
            return nil, "Invalid coordinate part: " .. parts[i]
        end
        coords[i] = val
        if isRel then
            anyRelative = true
        end
    end

    if anyRelative then
        -- Need absolute reference coordinates
        if not gpsAvailable then
            -- Try re-checking GPS
            local x, y, z = gps.locate()
            if not x then
                return nil, "GPS is not available, cannot resolve relative coordinates."
            end
            gpsAvailable = true
            gx, gy, gz = x, y, z
        else
            -- Use latest position for better accuracy
            local x, y, z = gps.locate()
            if x then
                gx, gy, gz = x, y, z
            end
        end
        return { gx + coords[1], gy + coords[2], gz + coords[3] }, nil
    else
        return coords, nil  -- absolute
    end
end

-- Draw cool header
local function drawHeader()
    term.setBackgroundColor(colors.black)
    term.clear()
    term.setCursorPos(1,1)
    term.setTextColor(colors.lightBlue)
    print("+-----------------------------+")
    print("|  HYBRID CONTROLLER   v2.0   |")
    print("+-----------------------------+")
    print("| Dynamic Target Broadcaster  |")
    print("| Supports Relative Coords ~  |")
    print("+-----------------------------+")
end

-- Helper color functions
local function printSuccess(msg)
    term.clear()
    term.setCursorPos(1,11)
    print(string.rep("-", 36))
    term.setCursorPos(1,12)
    term.setTextColor(colors.green)
    print(msg)
    term.setTextColor(colors.white)
    term.setCursorPos(1,1)
end

local function printError(msg)
    term.clear()
    term.setCursorPos(1,11)
    print(string.rep("-", 36))
    term.setCursorPos(1,12)
    term.setTextColor(colors.red)
    print(msg)
    term.setTextColor(colors.white)
    term.setCursorPos(1,1)
end

-- Main loop
local targetX, targetY, targetZ = nil, nil, nil
drawHeader()
print()

while true do
    drawHeader()
    print()
    if targetX then
        term.setTextColor(colors.yellow)
        write("Current target: (")
        term.setTextColor(colors.white)
        write(targetX .. ", " .. targetY .. ", " .. targetZ .. ")")
    end
    print()

    term.setTextColor(colors.cyan)
    write("Enter new target (X Y Z) or relative (e.g. ~ ~ ~), q to quit: ")
    term.setTextColor(colors.white)
    local input = read()
    if not input then break end
    input = input:match("^%s*(.-)%s*$") -- trim
    if input == "q" or input == "exit" then
        print("Exiting.")
        break
    end


    local coords, err = parseInput(input)
    if not coords then
        printError("Error: " .. err)
    else
        targetX, targetY, targetZ = coords[1], coords[2], coords[3]
        local msg = "target " .. targetX .. " " .. targetY .. " " .. targetZ
        rednet.broadcast(msg)
        printSuccess(">>> Broadcasted: " .. msg)
    end
end

rednet.close(peripheral.getName(modem))