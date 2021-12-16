from adafruit_servokit import ServoKit
import sys
import time

# Servo motor for changing the angle up or down
kit = ServoKit(channels=16)

LEFT_SHOULDER = 0
LEFT_ELBOW = 1
LEFT_WRIST = 2

RIGHT_SHOULDER = 3
RIGHT_ELBOW = 4
RIGHT_WRIST = 5

device_dict = {
LEFT_SHOULDER:"LEFT_SHOULDER",
LEFT_ELBOW:"LEFT_ELBOW",
LEFT_WRIST:"LEFT_WRIST",
RIGHT_SHOULDER:"RIGHT_SHOULDER",
RIGHT_ELBOW:"RIGHT_ELBOW",
RIGHT_WRIST:"RIGHT_WRIST"
}


if __name__ == "__main__":
    device = int(sys.argv[1])
    cmd = int(sys.argv[2])

    if device not in device_dict:
        print(f"ERROR: device {device} not recognized")
    elif cmd < 0 or cmd > 180:
        print(f"cmd {cmd} out of range. Position control from 0 to 180.")
    elif device == LEFT_WRIST or device == RIGHT_WRIST:
        device_name = device_dict[device]
        print(f"Moving device {device_name} on servo channel {device} to {cmd}")
        kit.continuous_servo[device].throttle = 0.5
        time.sleep(0.25)
        kit.continuous_servo[device].throttle = 0.0
    else:
        device_name = device_dict[device]
        print(f"Moving device {device_name} on servo channel {device} to {cmd}")
        kit.servo[device].angle = cmd



