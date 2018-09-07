import tty
import select
import sys
import termios
import interface


class TeleopC(object):
    def __init__ (self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.myspeedctrl = SendSpeed()

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
            if key == "a":
                self.myspeedctrl.send_speed(0,0,0,1)

            elif key =="s":
                self.myspeedctrl.send_speed(-1,0,0,0)

            elif key == "w":
                self.myspeedctrl.send_speed(1,0,0,0)

            elif key == "s":
                self.myspeedctrl.send_speed(0,0,0,-1)






if __name__ == "__main__":
    myTeleop = TeleopC()
    myTeleop.run()
