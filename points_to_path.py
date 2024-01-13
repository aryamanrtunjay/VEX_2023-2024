in_file = open("skills.txt", 'r')
out_file = open("skills_code.txt", "w")

prevX = 0
prevY = 0
prevDeg = 0

for line in in_file.readlines():
    splt = line.split(", ")
    cX = float(splt[0])
    cY = float(splt[1])
    cDeg = float(splt[2])

    x = cX - prevX
    y = cY - prevY
    deg = cDeg - prevDeg
    
    bwd = False
    if(len(splt) == 4):
        bwd = True
        
    
    output = "moveBot(" + str(round(x, 2)) + ", " + str(round(y, 2)) + ", " + str(round(deg, 2)) + ", 3000, " + str(bwd).lower() + ");\n"
    
    out_file.write(output)
    prevX = cX
    prevY = cY
    prevDeg = cDeg