import math
import os.path

type_of_path = "Skills2Push"

in_file = open("jerryio/" + type_of_path + ".txt", 'r')
out_file = open("prosPaths/" + type_of_path + ".txt", "w")

startX = 0
startY = 0
startHeading = 0
positions = []
first = True
backwards = False

for line in in_file.readlines():
    splt = line.split(",")
    if len(splt) == 4:
        # pdb.set_trace()
        if first:
            startX = float(splt[0])/2.54
            startY = float(splt[1])/2.54
            startHeading = float(splt[3])
            first = False
        else:
            pos = []
            x = (float(splt[0])/2.54)
            y = round(float(splt[1])/2.54)
            pos.append(x - startX)
            pos.append(y - startY)
            pos.append(float(splt[3]) - startHeading)

            pos.append((math.sqrt(pos[0] ** 2 + pos[1] ** 2) / (43.19689 * ((360 - pos[2]))/360) * 1000 + 150))
            
            if(abs(pos[2] - (math.atan2(startY - y, startX - x) * 180 / 3.14)) <= 45):
                pos.append("true")
            else:
                pos.append("false")

            startX = float(splt[0])/2.54
            startY = float(splt[1])/2.54
            startHeading = float(splt[3])
            positions.append(pos)
        
for i in positions:
    out_file.write("moveBot(" + str(round(i[0], 3)) + ", " + str(round(i[1], 3)) + ", " + str(round(i[2], 3)) + ", " + str(round(i[3], 3)) + ", " + i[4] + ");\n")

# "x, y, theta, backwards"