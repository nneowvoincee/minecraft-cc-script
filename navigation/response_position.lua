CHANNEL_BROADCAST = 10001
peripheral.find("modem", rednet.open)
local id, message

while true do
    id, message = rednet.receive()
    if message == "locate" then
        rednet.send(id, ("%f %f %f"):format(gps.locate()))
        --print(("%f %f %f"):format(gps.locate()))
    end
end