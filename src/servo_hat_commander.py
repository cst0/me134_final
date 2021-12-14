#!/usr/bin/env python3

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from me134_final.msg import ArmState

# constants representing the servo state
# fmt:off
LEFT_FINGER  = 0
RIGHT_FINGER = 4

LEFT_SHOULDER  = 8
RIGHT_SHOULDER = 9

LEFT_ELBOW  = 14
RIGHT_ELBOW = 15

# fmt:on


class ServoController(object):
    def __init__(self):
        rospy.init_node("ServoController", anonymous=False)
        self.ddynrec = DDynamicReconfigure("")

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

        rospy.Subscriber("control_to_servo/out", ArmState, self.arm_control_cb)

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
        left_shoulder       = max(-1, min(1, msg.left_shoulder))
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

        # send pwm values
        if self.pca is not None:
            self.pca.channels[LEFT_SHOULDER].duty_cycle = int(left_shoulder)
            self.pca.channels[LEFT_ELBOW].duty_cycle  = int(left_elbow)
            self.pca.channels[RIGHT_SHOULDER].duty_cycle = int(right_shoulder)
            self.pca.channels[RIGHT_ELBOW].duty_cycle = int(right_elbow)
            self.pca.channels[LEFT_FINGER].duty_cycle = int(self.servo_max_pwm if left_finger else self.servo_min_pwm)
            self.pca.channels[RIGHT_FINGER].duty_cycle = int(self.servo_max_pwm if right_finger else self.servo_min_pwm)
        # fmt:on


def main():
    sc = ServoController()
    rospy.loginfo("spinning servo controller")
    rospy.spin()
    rospy.loginfo("servo controller shutting down")


if __name__ == "__main__":
    main()
