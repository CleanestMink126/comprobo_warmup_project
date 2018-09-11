"""Makes Neato follow wall ___ ft away using proportional control"""
import numpy as np
import interface
import teleop
#get rid of zeros
#values = np.array(m.ranges)
#no_zeros = values(np.where(values))

def run():
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()
    while mytelC.key != '\x03':
        #check if surround values = d/cos(theta)
        #returns (degree_index int, distance float) follows equation OR (None, None)
        degree_index, d = mylidar.get_wall()
        print(degree_index,d)
        #if point returned, look at d
        if degree_index != None:
            if degree_index > 90:
                print("degree_index" + degree_index)
                if d <= 1.5:
                    print("d" + d)
                    mytelC.myspeedctrl.send_speed(.75,.5)
                elif d > 1.5:
                    mytelC.myspeedctrl.send_speed(.75,-.5)
            elif degree_index <90:
                print("degree_index" + degree_index)
                if d <= 1.5:
                    print("d" + d)
                    mytelC.myspeedctrl.send_speed(.75,-.5)
                elif d > 1.5:
                    mytelC.myspeedctrl.send_speed(.75,.5)
        else:
            mytelC.myspeedctrl.send_speed(0,0)

if __name__ == "__main__":
    run()
