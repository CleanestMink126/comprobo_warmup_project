"""Makes Neato follow wall ___ ft away using proportional control"""

#get rid of zeros
values = np.array(m.ranges)
no_zeros = values(np.where(values))

#find minimum


#check if surround values = d/cos(theta)
#returns None if there isnt one, or return a int if point follows equation
