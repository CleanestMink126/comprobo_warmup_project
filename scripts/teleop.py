import tty
import select
import sys
import termios
import interface

if key == "a":
    send_speed(0,0,0,0,0,1)

if key =="s":
    send_speed(0,-1,0,0,0,0)

if key == "w":
    send_speed(0,1,0,0,0,0)

if key == "s":
    send_speed(0,0,0,0,0,-1)






class TeleopC(object):
    def __init__ (self):
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        key=None
        while key != '\x03':
            key = self.getKey()
            print key

if __name__ == "__main__":
    myTeleop = TeleopC()
    myTeleop.run()
