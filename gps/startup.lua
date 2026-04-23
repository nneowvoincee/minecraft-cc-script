-- https://tweaked.cc/guide/gps_setup.html

local X, Y, Z = 0, 0, 0  --modify this depend on the coordinate of your computer
shell.run("gps", "host", X, Y, Z)
