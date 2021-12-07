#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from me134_final.msg import Controller, ArmState


PREFIX = "control_to_servo"
MIN_CONTROL = -1.0
MAX_CONTROL = 1.0

MIN_SERVO = 0
MAX_SERVO = 180
ZERO_SERVO = int((MAX_SERVO - MIN_SERVO) / 2.0)

N_JOINTS = 3  # 3 joints
N_JOINTS *= 2  # 2 arms

# Set specific joint servo ranges here. They’ll be respected in cap_and_map
JOINT_SERVO_LIMITS = {vals: (MIN_SERVO, MAX_SERVO) for vals in range(N_JOINTS)}
# e.g. set the range of the first joint
# JOINT_LIMITS[0] = (-10, 10)


def map_between_ranges(value, min0, max0, min1, max1):
    ## quick lifted from: https://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another
    # Figure out how ‘wide’ each range is
    leftSpan = max0 - min0
    rightSpan = max1 - min1

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - min0) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    ret = min1 + (valueScaled * rightSpan)

    print(f"Converted {value} to {ret}")
    return ret


"""
Take in radians for each joint. Clean and cap and output a list of servo positions
"""


class RadiansToServo:
    def __init__(self):
        self.radian_sub = rospy.Subscriber(
            "controller", Controller, self.msg_in, queue_size=10
        )
        self.servo_pub = rospy.Publisher(PREFIX + "/out", ArmState, queue_size=10)

        initial_msg = ArmState()
        initial_msg.left_shoulder = ZERO_SERVO
        initial_msg.left_elbow = ZERO_SERVO
        initial_msg.left_finger = False
        initial_msg.right_shoulder = ZERO_SERVO
        initial_msg.right_elbow = ZERO_SERVO
        initial_msg.right_finger = False

        self.servo_pub.publish(initial_msg)

    def cap_and_map(self, controller_axes_values):
        if len(controller_axes_values) < 4:
            print(
                f"WARNING: Too few joint values! Expected {4} got {len(controller_axes_values)}"
            )

        capped = [
            max(min(MAX_CONTROL, entry), MIN_CONTROL)
            for entry in controller_axes_values
        ]

        mapped = []
        for i in range(len(capped)):
            jmin_limit, jmax_limit = JOINT_SERVO_LIMITS[i]
            if jmin_limit < MIN_SERVO:
                print(
                    f"WARN: Servo {i} min value of {jmin_limit} is below global min {MIN_SERVO}"
                )
            if jmax_limit < MIN_SERVO:
                print(
                    f"WARN: Servo {i} max value of {jmax_limit} is below global min {MAX_SERVO}"
                )
            mapped.append(
                map_between_ranges(
                    capped[i], MIN_CONTROL, MAX_CONTROL, jmin_limit, jmax_limit
                )
            )
        return mapped

    def msg_in(self, controller_msg: Controller):
        # if any(b for b in controller_msg.button_state[4:8]):
        print(f"Heard {controller_msg}")
        ret = ArmState()

        pp = self.cap_and_map(controller_msg.axis_state)

        ret.left_shoulder = pp[0]
        ret.left_elbow = pp[1]
        ret.right_shoulder = pp[2]
        ret.right_elbow = pp[3]

        ret.left_finger = controller_msg.buttons_state[6]
        ret.right_finger = controller_msg.buttons_state[7]

        self.servo_pub.publish(ret)


if __name__ == "__main__":
    try:
        rospy.init_node(PREFIX, anonymous=False)
        node = RadiansToServo()
        rospy.spin()
        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     rate.sleep()
    except rospy.ROSInterruptException:
        pass

