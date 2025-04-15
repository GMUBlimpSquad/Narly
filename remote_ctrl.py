from evdev import InputDevice, categorize, ecodes
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
from multiprocessing import Manager
import time


# Open the PS5 controller device
device = InputDevice('/dev/input/event2')  # Replace with actual eventX
DEADZONE = 20


def millis():
    return int(time.time() * 1000)  # Convert to milliseconds


# Define start and end positions
deg_start = 30  # 90 degrees
deg_end = 150  # -90 degrees
jump = 5  # Adjust step size
forward_pos = 90
back_pos = 90
L_pos = 90
R_pos = 90
updown = 0
Lupdown = 0
Rupdown = 0
leftright = 0
time_interval_flap = 30
time_backwing_int =  20
current_time = 0
last_move_time1 = 0
last_move_time2 = 0
last_move_time3 = 0
last_move_time4 = 0



switched = False

def switch_auto():
    if switch_auto == True:
        return True
    else:
        return False

def read_controller(shared_inputs):
    global dpad_down_pressed, dpad_up_pressed, dpad_left_pressed, dpad_right_pressed, cg_backwards, cg_forward, switch_auto
    while True:
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_HAT0Y:
                if event.value == -1:  # D-pad UP pressed
                    dpad_up_pressed = True
                else:  # D-pad UP released
                    dpad_up_pressed = False
                if event.value == 1:  # D-pad DOWN pressed
                    dpad_down_pressed = True
                else:
                    dpad_down_pressed = False  # Released
            if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_HAT0X:
                if event.value == -1:  # D-pad LEFT pressed
                    dpad_left_pressed = True
                else:  # D-pad UP released
                    dpad_left_pressed = False
                if event.value == 1:  # D-pad DOWN pressed
                    dpad_right_pressed = True
                else:
                    dpad_right_pressed = False  # Released
            if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_SOUTH:  # Button A
                if event.value == 1:  # Button Pressed
                    cg_forward = True
                else:
                    cg_forward = False

            if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_EAST:  # Button B
                if event.value == 1:  # Button Pressed
                    cg_backwards = True
                else:
                    cg_backwards = False
            if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TR:  # Buttons
                if event.value == 1 and switched == False:
                    switch_auto = True
                    switched = True
                elif event.value == 1 and switched == True:
                    switch_auto = False
                    switched = False

            time.sleep(0.01)


        