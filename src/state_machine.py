#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134_final.msg import ArmState, Bar
from geometry_msgs.msg import Point, TransformStamped
import time
import numpy as np

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

# >>>> Static transforms (no reason to publish)
# CAMERA_TO_RIGHT_ARM = (0., 0., 0.)
# CAMERA_TO_LEFT_ARM = (0., 0., 0.)
# static_rightTransformStamped = TransformStamped()
# static_rightTransformStamped.header.stamp = rospy.Time.now()
# static_rightTransformStamped.header.frame_id = "camera"
# static_rightTransformStamped.child_frame_id = "right_arm"
# static_rightTransformStamped.transform.translation.x = CAMERA_TO_RIGHT_ARM[0]
# static_rightTransformStamped.transform.translation.y = CAMERA_TO_RIGHT_ARM[1]
# static_rightTransformStamped.transform.translation.z = CAMERA_TO_RIGHT_ARM[2]
# static_rightTransformStamped.transform.rotation.x = 0
# static_rightTransformStamped.transform.rotation.y = 0
# static_rightTransformStamped.transform.rotation.z = 0
# static_rightTransformStamped.transform.rotation.w = 1
# static_leftTransformStamped = TransformStamped()
# static_leftTransformStamped.header.stamp = rospy.Time.now()
# static_leftTransformStamped.header.frame_id = "camera"
# static_leftTransformStamped.child_frame_id = "left_arm"
# static_leftTransformStamped.transform.translation.x = CAMERA_TO_LEFT_ARM[0]
# static_leftTransformStamped.transform.translation.y = CAMERA_TO_LEFT_ARM[1]
# static_leftTransformStamped.transform.translation.z = CAMERA_TO_LEFT_ARM[2]
# static_leftTransformStamped.transform.rotation.x = 0
# static_leftTransformStamped.transform.rotation.y = 0
# static_leftTransformStamped.transform.rotation.z = 0
# static_leftTransformStamped.transform.rotation.w = 1
# <<<<<< Static transforms (no reason to publish)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def limit_switch():
    return False, False

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

class StateMachine:
    def __init__(self):
        self.arm = rospy.Publisher("control_to_servo", ArmState, queue_size=10)
        self.detected_bar_sub = rospy.Subscriber("grasp_bar", Bar, self.detected_bar, queue_size=1)
        
        self.state = State.DUAL_HANG
        self.eef_l_pose = None
        self.eef_r_pose = None
        self.bar = None

        self.count_max = 10
        self.slide_counts = list(np.linspace(-1, 1, self.count_max))

    def loop(self):
        rospy.loginfo(f"State is {state_to_string[self.state]}")
        if (self.state == State.DUAL_HANG):
            # Scan 
            # Determine position we want to move to
            if (self.bar is None):
                eef_l_pose, eef_r_pose = self.get_grasping_pose()
                if (eef_l_pose is not None):
                    self.change_state(State.RELEASE_FROM_HANG)
                else:
                    print(f"No eef found. Rescanning...")
            rospy.sleep(0.05)

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
            self.arm.pub(eef_l_pose)
            rospy.sleep(1)

            # >>>> slide it down
            count = 0
            while (not limit_switch()[0]) or (count < self.count_max):
                msg = populate_arm_msg(NO_ACTION, self.slide_counts[count], False, NO_ACTION, NO_ACTION, True)
                self.arm.pub(msg)
                count += 1
                rospy.sleep(0.5)
            # <<<< now stop the slide

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
            self.arm.pub(eef_r_pose)

            # >>>> slide it down
            count = 0
            while (not limit_switch()[1]) or (count < self.count_max):
                msg = populate_arm_msg(NO_ACTION, NO_ACTION, True, NO_ACTION, self.slide_counts[count], False)
                self.arm.pub(msg)
                count += 1
                rospy.sleep(0.5)
            # <<<< now stop the slide

            self.arm.pub(pull_up_msg)
            rospy.sleep(1)
            self.change_state(State.DUAL_HANG)

    def bar_to_cartesian(self):
        if (self.bar is None):
            rospy.logwarn("get_grasping_pose called with no bar.")
            return None, None, None, None
        else:
            l_horizon_rads, l_vertical_rads, l_dist_mm, r_horizon_rads, r_vertical_rads, r_dist_mm = self.bar.l_horizon_rads, self.bar.l_vertical_rads, self.bar.l_dist_mm, self.bar.r_horizon_rads, self.bar.r_vertical_rads, self.bar.r_dist_mm
            lx = l_dist_mm * np.sin(l_vertical_rads) * np.cos(l_horizon_rads)
            ly = l_dist_mm * np.sin(l_vertical_rads) * np.sin(l_horizon_rads)
            lz = l_dist_mm * np.cos(l_vertical_rads)

            rx = r_dist_mm * np.sin(r_vertical_rads) * np.cos(r_horizon_rads)
            ry = r_dist_mm * np.sin(r_vertical_rads) * np.sin(r_horizon_rads)
            rz = r_dist_mm * np.cos(r_vertical_rads)

            return (lx,ly,lz), (rx,ry,rz), l_vertical_rads, r_vertical_rads

    
    def get_grasping_pose(self):
        left, right, l_psi, r_psi = self.bar_to_cartesian()
        if (left is None):
            return None, None

        
        # Use the static transform from the camera (X out of lens, Y up-down, Z left-right) to get the target position in the arm shoulder
        # joint, and reduce the problem to a planar robot.

        # turn shoulder to point to rung
        eef_l_pose = populate_arm_msg(l_psi, 0, False, NO_ACTION, NO_ACTION, True)
        eef_r_pose = populate_arm_msg(NO_ACTION, NO_ACTION, True, r_psi, 0., False)
        
        return eef_l_pose, eef_r_pose

    def change_state(self, new_state):
        rospy.loginfo(f"Changing from state {state_to_string[self.state]} to {state_to_string[new_state]}.")
        self.state = new_state

    def detected_bar(self, msg):
        rospy.loginfo(f"Got bar at {msg}")
        self.bar = msg

def main():
    rospy.init_node("StateMachine")
    sm = StateMachine()
    rospy.loginfo("spinning central controller (state_machine)")
    while not rospy.is_shutdown():
        sm.loop()
        rospy.sleep(0.01)
    # rospy.spin()
    rospy.loginfo("central controller shutting down.")

if __name__ == "__main__":
    main()