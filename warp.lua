con = peripheral.isPresent("right")
if con then
    method = peripheral.getMethods("right")
    for m, i in ipairs(method) do
        print(m .. ":" .. i)
    end
else
        print(con)
end