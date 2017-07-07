#!/usr/bin/env python  
import roslib; roslib.load_manifest('move_base')
import rospy
import  os
import  sys
import  tty, termios
import time
from wheel.srv import *

setSpeedsProxy = rospy.ServiceProxy( "setSpeeds", SetSpeeds )

if __name__ == '__main__':
    print "Reading form keybord"
    print """   i j  k  l  m"""
    print 'press Q to quit'
    while True:
        fd=sys.stdin.fileno()
        old_settings=termios.tcgetattr(fd)
        #old_settings[3]= old_settings[3] & ~termios.ICANON & ~termios.ECHO    
        try:
            tty.setraw(fd)
            ch=sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            #print 'error'  
        if ch=='f':
            	print 'move forward'
		setSpeedsProxy("F", 15, "F", 15)
        elif ch=='b':
            	print 'move back'
		setSpeedsProxy("B", 15, "B", 15)
        elif ch=='l':
            	print "turn left!"
		setSpeedsProxy("B", 15, "F", 15)
        elif ch=='r':
            	print "turn right!"
		setSpeedsProxy("F", 15, "B", 15)
        elif ch=='s':
            	print "stop motor!"
		setSpeedsProxy("F", 0, "F", 0)
        elif ch=='q':
            print "shutdown!"
            break
        elif ord(ch)==0x3:
            print "shutdown"
            break
        print "Reading form keybord"
        print """   i j  k  l m"""
        print 'press Q or ctrl+c to quit'
