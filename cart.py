import math
import time
import threading
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
from adafruit_servokit import ServoKit

stepper_kit = MotorKit()
servo_kit = ServoKit(channels=16)

# Number of columns (x) and rows (y) of jars on cart
jar_num_x = 13
jar_num_y = 7

# Diameter of jars in mm
jar_diam = 60

# Location of arm shoulder axis relative to jars in mm
ori_x = jar_num_x*jar_diam / 2
ori_y = -10

# Length of robot arm sections in mm
arm_1 = 320
arm_2 = 320

# Initialize global stepper variables
stepper_1 = 0
stepper_2 = 0

# Coefficient for converting degrees to steps (pulley tooth count/motor tooth count * steps per revolution/360)
stepper_1_deg_to_step = 100/20 * 200/360
stepper_2_deg_to_step = 80/20 * 200/360

# Time to wait between individual steps in seconds
wait = 0.01


def get_coord(j, origin):
    # Add radius to get center of jar and subtract arm origin offset
    return j*jar_diam + jar_diam/2 - origin
    
def get_step_count_1(x, y):
    global stepper_1
    current_stepper_1 = stepper_1
    if x > 0:
        stepper_1 = 180 - (math.degrees(math.atan(y/x)) + math.degrees(math.acos((x*x + y*y + arm_1*arm_1 - arm_2*arm_2) / (2*math.sqrt(y*y + x*x)*arm_1))))
    elif x < 0:
        stepper_1 = math.degrees(math.atan(y/abs(x))) + math.degrees(math.acos((x*x + y*y + arm_1*arm_1 - arm_2*arm_2) / (2*math.sqrt(y*y + x*x)*arm_1)))
    elif x == 0:
        stepper_1 = math.degrees(math.atan(y/1)) + math.degrees(math.acos((1 + y*y + arm_1*arm_1 - arm_2*arm_2) / (2*math.sqrt(y*y + 1)*arm_1)))    
    
    step_count = stepper_1_deg_to_step * (current_stepper_1 - stepper_1)
    
    print("x=" + str(x) + ", y=" + str(y) + ", angle_1=" + str(stepper_1))
    
    return step_count

            
def get_step_count_2(x, y):
    global stepper_2
    current_stepper_2 = stepper_2
    if x > 0:
        stepper_2 = math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) / (2*arm_1*arm_2)))
    elif x <= 0:
        stepper_2 = 360 - math.degrees(math.acos((arm_2*arm_2 + arm_1*arm_1 - x*x - y*y) / (2*arm_1*arm_2)))
    
    step_count = stepper_2_deg_to_step * (current_stepper_2 - stepper_2)
    
    print("x=" + str(x) + ", y=" + str(y) + ", angle_2=" + str(stepper_2))
    
    return step_count
    
def start_step_threads(step_count_1, step_count_2):
    if __name__ == "__main__":
        t1 = threading.Thread(target=step_1, args=(step_count_1,))
        t2 = threading.Thread(target=step_2, args=(step_count_2,))
        
        t1.start()
        t2.start()
        
        t1.join()
        t2.join()


def step_1(steps):
    if steps > 0:
        for s in range(int(steps)):
            stepper_kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
            time.sleep(wait)
    elif steps < 0:
        for s in range(abs(int(steps))):
            stepper_kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
            time.sleep(wait)
            
def step_2(steps):
    if steps > 0:
        for s in range(int(steps)):
            stepper_kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
            time.sleep(wait)
    elif steps < 0:
        for s in range(abs(int(steps))):
            stepper_kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
            time.sleep(wait)
            
    time.sleep(1)
    servo_kit.servo[0].angle = 90
    time.sleep(0.5)
    servo_kit.servo[0].angle = 0


for x in range(jar_num_x):
    if x % 2 == 0:
        for y in range(jar_num_y):
            # Get next x/y coordinates of jar
            x_coord = get_coord(x, ori_x)
            y_coord = get_coord(y, ori_y)
            # Get number of steps (positive or negative) to next coordinates
            step_count_1 = get_step_count_1(x_coord, y_coord)
            step_count_2 = get_step_count_2(x_coord, y_coord)
            # Start stepper threads simultaneously
            start_step_threads(step_count_1, step_count_2)
            print("---------------------------------------------")
            
    else:
        # Reverse order for odd-numbered columns
        for y in range(jar_num_y-1, -1, -1):
            # Get next x/y coordinates of jar
            x_coord = get_coord(x, ori_x)
            y_coord = get_coord(y, ori_y)
            # Get number of steps (positive or negative) to next coordinates
            step_count_1 = get_step_count_1(x_coord, y_coord)
            step_count_2 = get_step_count_2(x_coord, y_coord)
            # Start stepper threads simultaneously
            start_step_threads(step_count_1, step_count_2)
            print("---------------------------------------------")

# Return steppers to home position
start_step_threads(stepper_1_deg_to_step * (stepper_1), stepper_2_deg_to_step * (stepper_2))
#Maybe create a separate homing function so servo doesn't activate when returning to home


# Release steppers
stepper_kit.stepper1.release()
stepper_kit.stepper2.release()
