"""Makes Neato follow wall ___ ft away using proportional control"""
import numpy as np
import interface
import teleop
import rospy
#get rid of zeros
#values = np.array(m.ranges)
#no_zeros = values(np.where(values))

def run():
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #check if surround values = d/cos(theta)
        #returns (degree_index int, distance float) follows equation OR (None, None)
        r.sleep()
        degree_index, d = mylidar.get_wall()
        #if point returned, look at d

        if degree_index != None:
            print("d", d)
            if d >= 1.5:
                print("degree_index", degree_index)
                if degree_index >= 180:
                    mytelC.myspeedctrl.send_speed(.25,-.1+((degree_index-360)/100.0))
                elif degree_index < 180:
                    mytelC.myspeedctrl.send_speed(.25,.1+(degree_index/100.0))
            elif d < 1.5:
                print("degree_index", degree_index)
                if degree_index <= 180:
                    if degree_index <= 90:
                        mytelC.myspeedctrl.send_speed(.25,-.1+((degree_index-90)/100.0))
                    else:
                        mytelC.myspeedctrl.send_speed(.25,.1+((degree_index-90)/100.0))
                elif degree_index > 180:
                    if degree_index <= 270:
                        mytelC.myspeedctrl.send_speed(.25,-.1+((degree_index-270)/100.0))
                    else:
                        mytelC.myspeedctrl.send_speed(.25,.1+((degree_index-270)/100.0))
        else:
            mytelC.myspeedctrl.send_speed(.25,0)
    mytelC.myspeedctrl.send_speed(0,0)

if __name__ == "__main__":
    run()
