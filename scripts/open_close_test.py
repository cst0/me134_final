from adafruit_servokit import ServoKit
import sys
import time
import rospy
import time

# Servo motor for changing the angle up or down
kit = ServoKit(channels=16)

LEFT_FINGER = 2
RIGHT_FINGER = 5

WIND_TIME_I = 0.65
WIND_TIME_II = 0.55
WIND_THROTTLE = -1.0
UNWIND_TIME = 1.04
UNWIND_THROTTLE = -WIND_THROTTLE

if __name__ == "__main__":
    device = int(sys.argv[1])
    cmd = int(sys.argv[2])

    open = cmd > 0
    close = not open

    if (device == 0 and close):
        print("WINDING LEFT")
        kit.continuous_servo[LEFT_FINGER].throttle = WIND_THROTTLE
        time.sleep(WIND_TIME_I)
        kit.continuous_servo[LEFT_FINGER].throttle = -0.5
        time.sleep(WIND_TIME_II)
        kit.continuous_servo[LEFT_FINGER].throttle = 0.0
    elif (device == 0 and open):
        print("UNWINDING LEFT")
        kit.continuous_servo[LEFT_FINGER].throttle = UNWIND_THROTTLE
        time.sleep(UNWIND_TIME)
        kit.continuous_servo[LEFT_FINGER].throttle = 0.0

    if (device == 1 and close):
        print("WINDING RIGHT")
        kit.continuous_servo[RIGHT_FINGER].throttle = WIND_THROTTLE
        time.sleep(WIND_TIME_I)
        kit.continuous_servo[RIGHT_FINGER].throttle = -0.5
        time.sleep(WIND_TIME_II)
        kit.continuous_servo[RIGHT_FINGER].throttle = 0.0
    elif (device == 1 and open):
        print("UNWINDING RIGHT")
        kit.continuous_servo[RIGHT_FINGER].throttle = UNWIND_THROTTLE
        time.sleep(UNWIND_TIME)
        kit.continuous_servo[RIGHT_FINGER].throttle = 0.0

    kit.continuous_servo[RIGHT_FINGER].throttle = 0.0
    kit.continuous_servo[LEFT_FINGER].throttle = 0.0


