#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134_final.msg import ArmState
from adafruit_servokit import ServoKit

# constants representing the servo state
# fmt:off
LEFT_FINGER  = 0
RIGHT_FINGER = 4

LEFT_SHOULDER  = 1
RIGHT_SHOULDER = 5

LEFT_ELBOW  = 2
RIGHT_ELBOW = 3

WIND_TIME = 0.25
WIND_THROTTLE = 0.25
UNWIND_TIME = WIND_TIME
UNWIND_THROTTLE = -WIND_THROTTLE

# fmt:on

class ServoController(object):
    def __init__(self):
        rospy.init_node("ServoController", anonymous=False)
        self.ddynrec = DDynamicReconfigure("")

        self.servo_kit = ServoKit(channels=16)
        self.previous_left_finger = False
        self.previous_right_finger = False

        self.servo_min_pwm: int = 0
        self.servo_max_pwm: int = 0
        self.ddynrec.add_variable("servo_min_pwm", "servo_min_pwm", 0, 0, 2 ** 12)
        self.ddynrec.add_variable("servo_max_pwm", "servo_max_pwm", 0, 0, 2 ** 12)

        self.add_variables_to_self()
        self.ddynrec.start(self.dyn_rec_callback)

        self.simulation_mode = True
        self.i2c_bus = None
        self.pca = None
        try:
            from board import SCL, SDA  # type:ignore
            import busio  # type:ignore
            from adafruit_pca9685 import PCA9685  # type:ignore

            self.i2c_bus = busio.I2C(SCL, SDA)
            self.pca = PCA9685(self.i2c_bus)
            self.pca.frequency = 60

            self.simulation_mode = False
        except ImportError:
            print(
                "WARN: Could not set up hardware dependencies. Assuming we're in sim mode and going from here."
            )


        self.left_shoulder_duty = 0
        self.left_elbow_duty = 0
        self.right_shoulder_duty = 0
        self.right_elbow_duty = 0
        self.left_finger_duty = 0
        self.right_finger_duty = 0

        rospy.Subscriber("control_to_servo", ArmState, self.arm_control_cb)

    def add_variables_to_self(self):
        var_names = self.ddynrec.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        del level
        var_names = self.ddynrec.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config

    def arm_control_cb(self, msg: ArmState):
        # fmt:off
        # retrieve values and ensure they're between -1 and 1
        left_shoulder =  max(-1, min(1, msg.left_shoulder))
        left_elbow  = max(-1, min(1, msg.left_elbow))
        right_shoulder = max(-1, min(1, msg.right_shoulder))
        right_elbow        = max(-1, min(1, msg.right_elbow))

        left_finger = msg.left_finger
        right_finger = msg.right_finger

        # convert each of these -1 to 1 values into the range of pwm values
        left_shoulder = ((left_shoulder + 1) / 2) * (self.servo_max_pwm - self.servo_min_pwm)
        left_elbow = ((left_elbow + 1) / 2) * (self.servo_max_pwm - self.servo_min_pwm)
        right_shoulder = ((right_shoulder + 1) / 2) * (self.servo_max_pwm - self.servo_min_pwm)
        right_elbow = ((right_elbow + 1) / 2) * (self.servo_max_pwm - self.servo_min_pwm)

        left_shoulder = max(self.servo_min_pwm, min(self.servo_max_pwm, left_shoulder))
        left_elbow = max(self.servo_min_pwm, min(self.servo_max_pwm, left_elbow))
        right_elbow = max(self.servo_min_pwm, min(self.servo_max_pwm, right_elbow))
        right_shoulder = max(self.servo_min_pwm, min(self.servo_max_pwm, right_shoulder))

        # >>>>>> JS: use the old values if the msg gives high negative values (intentional placeholder)
        left_shoulder = self.left_shoulder_duty if msg.left_shoulder < -2 else left_shoulder
        left_elbow = self.left_elbow_duty if msg.left_elbow < -2 else left_elbow
        right_shoulder = self.right_shoulder_duty if msg.right_shoulder < -2 else right_shoulder
        right_elbow = self.right_elbow_duty if msg.right_elbow < -2 else right_elbow
        # <<<<<<

        # send pwm values
        if self.pca is not None:
            self.pca.channels[LEFT_SHOULDER].duty_cycle = int(left_shoulder)
            self.pca.channels[LEFT_ELBOW].duty_cycle  = int(left_elbow)
            self.pca.channels[RIGHT_SHOULDER].duty_cycle = int(right_shoulder)
            self.pca.channels[RIGHT_ELBOW].duty_cycle = int(right_elbow)

            self.pca.channels[LEFT_FINGER].duty_cycle = int(2**11)
            self.pca.channels[RIGHT_FINGER].duty_cycle = int(2**11)

            # if (self.previous_left_finger == left_finger):
            #     pass # do nothing
            # elif (self.previous_left_finger and not left_finger):
            #     self.servo_kit.continuous_servo[LEFT_FINGER].throttle = WIND_THROTTLE
            #     rospy.sleep(UNWIND_TIME)
            #     self.servo_kit.continuous_servo[LEFT_FINGER].throttle = 0.0
            # elif (not self.previous_left_finger and left_finger):
            #     self.servo_kit.continuous_servo[LEFT_FINGER].throttle = UNWIND_THROTTLE
            #     rospy.sleep(WIND_TIME)
            #     self.servo_kit.continuous_servo[LEFT_FINGER].throttle = 0.0

            # if (self.previous_right_finger == right_finger):
            #     pass # do nothing
            # elif (self.previous_right_finger and not right_finger):
            #     self.servo_kit.continuous_servo[RIGHT_FINGER].throttle = WIND_THROTTLE
            #     rospy.sleep(UNWIND_TIME)
            #     self.servo_kit.continuous_servo[RIGHT_FINGER].throttle = 0.0
            # elif (not self.previous_right_finger and right_finger):
            #     self.servo_kit.continuous_servo[RIGHT_FINGER].throttle = UNWIND_THROTTLE
            #     rospy.sleep(WIND_TIME)
            #     self.servo_kit.continuous_servo[RIGHT_FINGER].throttle = 0.0

            # self.servo_kit.continuous_servo[RIGHT_FINGER].throttle = 0.0
            # self.servo_kit.continuous_servo[LEFT_FINGER].throttle = 0.0
            # STORE PREVIOUS VALUES

            self.left_shoulder_duty = left_shoulder
            self.left_elbow_duty = left_elbow
            self.right_shoulder_duty = right_shoulder
            self.right_elbow_duty = right_elbow
            self.previous_left_finger = left_finger
            self.previous_right_finger = right_finger

        # fmt:on


def main():
    sc = ServoController()
    rospy.loginfo("spinning servo controller")
    rospy.spin()
    rospy.loginfo("servo controller shutting down")


if __name__ == "__main__":
    main()
