import math
filename = "CloseSideElims"

in_file = open("Paths/" + filename + ".txt", 'r')
out_file = open("PathCode/" + filename + ".txt", 'w')

prevX = None
prevY = None
prevHead = 0
for line in in_file.readlines():
    splt = line.split(",")
    if len(splt) == 4:
        moveX = 0
        moveY = 0
        moveHeading = 0
        if(prevX == None):
            prevX = round(float(splt[0]), 2)
            prevY = round(float(splt[1]), 2)
            moveHeading = splt[3]
            out_file.write("chassis.pid_turn_set(" + str(round(float(splt[3]), 2)) + "_deg, TURN_SPEED, false);\n")
        else:
            dist = math.sqrt((round(float(splt[0]), 2) - prevX) ** 2 + (round(float(splt[1]), 2) - prevY) ** 2)/2.54
            if(dist < 5):
                out_file.write("chassis.pid_turn_set(" + str(round(float(splt[3]), 2)) + "_deg, TURN_SPEED, false);\n")
                out_file.write("pros::Task::delay(800);\n")
            else:
                out_file.write("chassis.pid_drive_set(" + str(round(float(dist), 2)) + "_in, DRIVE_SPEED, false);\n")
                time = dist / 76 * 1600
                out_file.write("pros::Task::delay(" + str(round(time, 2)) + ");\n")
            