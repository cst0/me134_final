#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134_final.msg import ArmState
from geometry_msgs.msg import Point
import time

from enum import Enum

NO_ACTION = -10
def populate_arm_msg(left_shoulder, left_elbow, left_finger, right_shoulder, right_elbow, right_finger):
    msg = ArmState()
    msg.left_shoulder = left_shoulder
    msg.left_elbow = left_elbow
    msg.left_finger = left_finger
    msg.right_shoulder = right_shoulder
    msg.right_elbow = right_elbow
    msg.right_finger = right_finger
    return msg

pull_up_msg = populate_arm_msg(1,1,True,1,1,True)
lower_down_msg = populate_arm_msg(-1,-1,True,-1,-1,True)

release_gripper_left = populate_arm_msg(NO_ACTION, NO_ACTION, False, NO_ACTION, NO_ACTION, True)
release_intermediate_left = populate_arm_msg(0, 0, False, NO_ACTION, NO_ACTION, True)
reach_intermediate_left = populate_arm_msg(-1, -1, False, NO_ACTION, NO_ACTION, True)

release_gripper_right = populate_arm_msg(NO_ACTION, NO_ACTION, True, NO_ACTION, NO_ACTION, False)
release_intermediate_right = populate_arm_msg(NO_ACTION, NO_ACTION, True, 0, 0, False)
reach_intermediate_right = populate_arm_msg(NO_ACTION, NO_ACTION, True, -1, -1, False)


class State(Enum):
    # initial set of static states
    DUAL_HANG = 0
    RELEASE_FROM_HANG = 1
    REACH_FROM_HANG = 2
    RETURN_TO_HANG = 3

state_to_string = {
    State.DUAL_HANG: "DUAL_HANG",
    State.RELEASE_FROM_HANG: "RELEASE_FROM_HANG",
    State.REACH_FROM_HANG: "REACH_FROM_HANG",
    State.RETURN_TO_HANG: "RETURN_TO_HANG"
    } # TODO    

class StateMachine(object):
    def __init__(self):
        rospy.init_node("StateMachine", anonymous=False)
        self.arm = rospy.Publisher("control_to_servo", ArmState, queue_size=10)
        
        self.state = State.BOTH_ARMS_HANG
        self.eef_pose = None

        while not rospy.is_shutdown():
            if (self.state == State.DUAL_HANG):
                # Scan 
                bar_info = self.scan_for_bar()
                # Determine position we want to move to
                eef_pose = self.get_grasping_pose(bar_info)
                if (eef_pose is not None):
                    self.change_state(State.RELEASE_FROM_HANG)
                else:
                    print(f"No eef found. Rescanning...")

            if (self.state == State.RELEASE_FROM_HANG):
                self.arm.pub(release_gripper_left)
                rospy.sleep(1)
                self.arm.pub(release_intermediate_left)
                rospy.sleep(1)
                self.change_state(State.REACH_FROM_HANG)
                pass
            if (self.state == State.REACH_FROM_HANG):
                self.arm.pub(reach_intermediate_left)
                rospy.sleep(1)
                # TODO: last step of reach with eef goal position in mind
                # ASSUMING WE'VE GRASPED # TODO: check the contact sensor in the palm 
                self.change_state(State.RETURN_TO_HANG)
                pass
            if (self.state == State.RETURN_TO_HANG):
                self.arm.pub(release_gripper_right)
                rospy.sleep(1)
                self.arm.pub(release_intermediate_right)
                rospy.sleep(1)
                self.arm.pub(reach_intermediate_right)
                rospy.sleep(1)
                # TODO: Reach to same eef position as left (but shifted over)
                rospy.sleep(1)
                self.arm.pub(pull_up_msg)
                rospy.sleep(1)
                self.change_state(State.DUAL_HANG)

    def scan_for_bar(self):
        left_side = Point(0,0,0)
        right_side = Point(0,0,0)
        return (left_side, right_side)

    def get_grasping_pose(self, bar_line):
        eef_pose = Point(0,0,0)
        return eef_pose

    def change_state(self, new_state):
        print(f"Changing from state {state_to_string[self.state]} to {state_to_string[new_state]}.")
        self.state = new_state

def main():
    sm = StateMachine()
    rospy.loginfo("spinning central controller (state_machine)")
    sm.spin()
    rospy.loginfo("central controller shutting down.")