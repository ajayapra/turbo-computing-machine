#!/usr/bin/env python
# BEGIN ALL
import sys, select, tty, termios
import rospy
from std_msgs.msg import String


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print "Returning from getKey"
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    key_pub = rospy.Publisher("action_input", String, queue_size=1)
    rospy.init_node("keystroke_pub")
    # BEGIN TERMIOS
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    # END TERMIOS
    print "Publishing keystrokes. Press 'o' to exit..."
    e = "Error"
    message = "messgae intialized"
    try:
        while not rospy.is_shutdown():
            # BEGIN SELECT
            key_pressed = getKey()
            if key_pressed == 't' or key_pressed == 'T':
                message = "t"
                print ("message: %s", message)
            elif key_pressed == 's' or key_pressed == 'S':
                message = "s"
                print ("message %s", message)
            elif key_pressed == 'h' or key_pressed == 'H':
                message = "h"
                print ("message %s", message)
            elif key_pressed == 'o' or key_pressed == 'O':
                break
            publish_msg = message
            key_pub.publish(publish_msg)  # Publishing to topic

    except rospy.ROSInterruptException:
        pass

    finally:
        default_msg = "No key press detected"
        key_pub.publish(default_msg)
        # BEGIN TERMIOS_END
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        # END TERMIOS_END  # END ALL
