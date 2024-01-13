in_file = open("path.jerryio.txt", 'r')
out_file = open("skills_code.txt", "w")

prevX = 0
prevY = 0
prevDeg = 0

for line in in_file.readlines():
    splt = line.split(",")
    cX = float(splt[0])
    cY = float(splt[1])

    x = cX - prevX
    y = cY - prevY
        
    
    output = "moveBot(" + str(round(x, 2)) + ", " + str(round(y, 2)) + ", " + "3000, " + "fwd = false);\n"
    
    out_file.write(output)
    prevX = cX
    prevY = cY

# "x, y, theta, backwards"