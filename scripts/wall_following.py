"""Makes Neato follow wall ___ ft away using proportional control"""

#get rid of zeros
values = np.array(m.ranges)
no_zeros = values(np.where(values))

def run():
    mytelC = TeleopC()
    while mytelC.key != '\x03':
        #find minimum

        #check if surround values = d/cos(theta)
        #returns (degree_index int, distance float) follows equation OR (None, None)

        #if point returned, look at d
        if ___ != (None,None):
            if degree_index > 90:
                if d <= 1.5:
                    mytelC.myspeedctrl.send_speed(.75,.5)
                elif d > 1.5:
                    mytelC.myspeedctrl.send_speed(.75,-.5)
            elif degree_index <90:
                if d <= 1.5:
                    mytelC.myspeedctrl.send_speed(.75,-.5)
                elif d > 1.5:
                    mytelC.myspeedctrl.send_speed(.75,.5)
        else:
            mytelC.myspeedctrl.send_speed(1,0)
