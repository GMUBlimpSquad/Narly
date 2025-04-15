from camera import Detection
import time
import math
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
import atexit
import board
import digitalio
from r_control import read_controller  # your updated function
from collections import deque
import barometer

err_y_history = deque(maxlen=5)
err_z_history = deque(maxlen=5)
err_y_g_history = deque(maxlen=5)
err_z_g_history = deque(maxlen=5)

ofbd_cam = Detection("192.168.0.200", 5001,False,False)

kit.continuous_servo[6].set_pulse_width_range(min_pulse=1000, max_pulse=2300)

# Define limits
PITCH_SCALE = 50  # Scaling for pitch control


DEAD_ZONE = 0.05        # Example value for small error threshold (adjust as needed)

# Define the GPIO pin for the limit switch
LIMIT_SWITCH_PIN = board.D17


err_ball = []

def ballCam():
    bounding_box = ofbd_cam.detect(target = 1, testing= False)
    #Track ball until caught
    if bounding_box is not None:
        return bounding_box
    else:
        return None
    
def goalCam():
    bounding_box = ofbd_cam.detect(target = 2, testing= False)
    #Track ball until caught
    if bounding_box is not None:
        return bounding_box
    else:
        return None



PSENSOR_PIN = board.D27
psensor_switch = digitalio.DigitalInOut(PSENSOR_PIN)
psensor_switch.direction = digitalio.Direction.INPUT
psensor_switch.pull = digitalio.Pull.UP  # Pull-up means LOW when pressed
last_psensor_check = 0
psensor_check_interval = 1000
psensor_trig_count_int = 3000
first_hit_time = 0
psensor_trig_count = 0
recent_hit = 0
gridlock = False
stucksensor = False   #I NEED TO HAVE A WHAT IF SENSOR IS STUCK CODE
last_move_timeB = 0
posB = 90


def psensor_triggered():
    """Returns True if the limit switch is pressed."""
    return not psensor_switch.value  # UP means the switch is pressed


total_stops = 20

def nonlinear_map_cg(error, scale, total_stops=20, center_stop=10, min_stop=2, max_stop=19):
    """
    Map Z error to a CG rail stop (1-20) using cubic scaling.
    Keeps position between min_stop and max_stop to avoid mechanical limits.
    """
    if abs(error) < DEAD_ZONE:
        return center_stop

    # Apply cubic scaling for a smoother nonlinear response
    mapped = round(center_stop - scale * (error ** 3))

    # Clamp to safe operating range
    return max(min_stop, min(max_stop, mapped))


posL = 90
posR = 90
current_time = 0
last_move_timeL = 0
last_move_timeR = 0
cg_int = 10000
cg_rail_stop = 10
last_time_check = 0
ls_int = 50
current_stop = 10
next_stop = 10
last_checkcg_time = 0
no_track = False
first_press = False


#Variables (manual)
last_move_time1 = 0
last_move_time2 = 0
last_move_time3 = 0
last_move_time4 = 0
last_move_time_backward = 0
forward_pos = 90
back_pos = 90
L_pos = 90
R_pos = 90


def stop_cgrail():
    kit.continuous_servo[6].throttle = -0.05

atexit.register(stop_cgrail)

def millis():
    return int(time.time() * 1000)  # Convert to milliseconds

# Set up the limit switch as an input with a pull-up resistor
limit_switch = digitalio.DigitalInOut(LIMIT_SWITCH_PIN)
limit_switch.direction = digitalio.Direction.INPUT
limit_switch.pull = digitalio.Pull.UP  # Pull-up means LOW when pressed



def limit_switch_triggered():
    """Returns True if the limit switch is pressed."""
    return not limit_switch.value  # UP means the switch is pressed


#Variables (forward() function)
flap_int = 40
last_move_time_forward = 0


def forward(freq = 0.7):
    global current_time, last_move_time_forward, forward_pos
    current_time = millis()
    direction = 1
    if current_time - last_move_time_forward >= flap_int:
        last_move_time_forward = current_time  # Update time
        forward_pos = oscillate_wing(direction, freq)
        # Move the servos
        kit.servo[1].angle = forward_pos
        kit.servo[0].angle = 180 - forward_pos

#Variables (uturn() function)
r_pos = 90
b_pos = 30
start_uturn = 0

def uturn():
    global current_time, last_move_time_forward, r_pos, b_pos
    current_time = millis()
    
    if current_time - last_move_time_forward >= flap_int:
        last_move_time_forward = current_time  # Update time

        r_pos = oscillate_wing(direction= 1, freq= 0.9)
        b_pos = 165
        
        # Move the servos
        kit.servo[0].angle = r_pos
        kit.servo[11].angle = b_pos
        kit.servo[1].angle = 90
        
        
    


#Variables for searchLoop
halflap_time_int = 15000


def gridlock_check():
    global gridlock, gridlock_start, psensor_trig_count, first_hit_time, recent_hit, current_time
    psensor_trigger = psensor_triggered()
    if psensor_trigger:
        if current_time - recent_hit >= 10:
            psensor_trig_count += 1
            recent_hit = millis()
#        else:
#            psensor_trig_count = 0

        print(psensor_trig_count)
        if psensor_trig_count == 1:
            first_hit_time = millis()

        if psensor_trig_count == 5 and recent_hit - first_hit_time <= psensor_trig_count_int:
            gridlock = True
            gridlock_start = millis()

now_uturn = False

def gridlock_active():
    global gridlock, last_move_timeB, current_time
    print("Gridlock Activated")

    current_time = millis()
    if current_time - last_move_timeB >= 60:  # Check if it's time to move
        last_move_timeB = current_time  # Update last move time
        
        posB = oscillate_wing(direction=1,freq=0.9)
        kit.servo[11].angle = posB

    if current_time - gridlock_start >= 7000:
        gridlock = False




# Function to map values from one range to another
def map_value(value, fromLow, fromHigh, toLow, toHigh):
    return ((value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow)

# Function to oscillate the wing with fixed frequency and a mapped angle
def oscillate_wing(direction=1, freq = 0.8):
    # Use the fixed frequency to oscillate the wing position
    angle = 90 + direction * 90 * math.sin((2 * freq * math.pi) * time.time())
    
    # Map the angle to the range [5, 10] to control servo movement
    #angle = map_value(angle, 0, 180, 5, 10)
    
    return angle

last_throttle = 0

switch_state = False

def manual_control(shared_inputs):
    global last_move_time1,last_move_time2, last_move_time3, last_move_time4, current_time, forward_pos, back_pos, R_pos, L_pos, last_throttle, current_stop, first_press, switch_state

    switch_state = False

    current_time = millis()
    if shared_inputs.get("dpad_up"):
        if current_time - last_move_time1 >= 80:
            last_move_time1 = current_time  # Update time
            forward_pos = oscillate_wing( direction= 1, freq= 0.8)
            
            # Move the servos
            kit.servo[1].angle = forward_pos
            kit.servo[0].angle = 180 - forward_pos
            kit.servo[11].angle = 90

    if shared_inputs.get("dpad_down"): 
        current_time = millis()
        if current_time - last_move_time2 >= 70:
            last_move_time2 = current_time  # Update time
            back_pos = oscillate_wing(direction=1,freq=0.9)
            # Move the servos
            kit.servo[11].angle = back_pos
            kit.servo[1].angle = 180
            kit.servo[0].angle = 0
    
    if shared_inputs.get("dpad_left"):  
    
        current_time = millis()
        if current_time - last_move_time3 >= 80:
            last_move_time3 = current_time  # Update time
            R_pos = oscillate_wing(direction= 1, freq=0.9)
            back_pos = 165

        # Move the servos
        kit.servo[0].angle = R_pos
        kit.servo[1].angle = 90
        kit.servo[11].angle = back_pos
    

    if shared_inputs.get("dpad_right"): 
        current_time = millis()
        if current_time - last_move_time4 >= 80:
            last_move_time4 = current_time  # Update time
            L_pos = oscillate_wing(direction=1,freq=0.9)
            
            back_pos = 15
        
        # Move the servos
        kit.servo[1].angle = L_pos
        kit.servo[0].angle = 90
        kit.servo[11].angle = back_pos

    throttle = -0.05 # Default
    if shared_inputs.get("cg_forward"):
        throttle = 1
        switch_state = limit_switch_triggered()
        if switch_state and not first_press:
            current_stop -= 1
            first_press = True
            print(current_stop)
        elif not switch_state and first_press:
            first_press = False

    elif shared_inputs.get("cg_backward"):
        throttle = -1
        switch_state = limit_switch_triggered()
        if switch_state and not first_press:
            current_stop += 1
            first_press = True
            print(current_stop)
        elif not switch_state and first_press:
            first_press = False

    if throttle != last_throttle:
        kit.continuous_servo[6].throttle = throttle
        last_throttle = throttle
  



def move_cg_rail(cg_rail_stop):
    global next_stop, current_stop, current_time, first_press, last_time_check, switch_state

    switch_state = False
    time_printing = 0

    if current_stop != cg_rail_stop:
        next_stop = current_stop + 1 if current_stop < cg_rail_stop else current_stop - 1
        if current_time - time_printing >= 2000:
            time_printing = current_time
            print(f"> Moving towards stop: [{next_stop}]/[{cg_rail_stop}]")
        if cg_rail_stop > current_stop:
            direction = -1
        elif cg_rail_stop < current_stop:
            direction = 1
        else:
            direction = -0.05

        kit.continuous_servo[6].throttle = direction
        switch_state = limit_switch_triggered()
        if switch_state and not first_press:
            print("First Press")
            current_stop = next_stop
            first_press = True
            if current_stop != cg_rail_stop:
                return False
            else:
                kit.continuous_servo[6].throttle = -0.05
                print("Destination Reached!")
                return True
        elif not switch_state and first_press:
            print("Released")
            first_press = False   

        return False
    else:
        kit.continuous_servo[6].throttle = -0.05
        print("Destination Reached!")
        return True
    


ground_reading = 0
time_printing = 0

time_getoff_ceilfloor = 0

def ball_search_loop(shared_inputs):
    global posL, posR, current_time, last_move_timeL, last_move_timeR, cg_rail_stop, last_checkcg_time, last_psensor_check, recent_hit, gridlock, last_move_timeB, posB, halflap_time, start_uturn, now_uturn, no_track, gridlock_start, tracking_ball, last_move_time_backward, altitude, altitude_readings, alt_time, ground_reading, time_printing, smoothed_altitude, GOAL_HEIGHT, CEILING_HEIGHT, time_getoff_ceilfloor


    gridlock = False
    no_track = False
    now_uturn = False
    halflap_time = millis()
    ball_on_target = False
    tracking_ball = False
    altitude_readings = []
    altitude = 0
    alt_time = 0
    cg_rail_stop = 10


    ground_reading = barometer.set_ground()
    GOAL_HEIGHT = 20 #CHANGE THIS WHEN YOU KNOW
    CEILING_HEIGHT = 0 #CHANGE THIS WHEN YOU KNOW
    FLOOR_SAFETY_HEIGHT = 3 #CHANGE THIS IF NEEDED



    while True:
        time.sleep(0.01)
        if shared_inputs["stop"]:
            break  # Exit cleanly
        current_time = millis()
        if current_time - alt_time >= 3000:
            alt_time = current_time
            altitude = barometer.get_height(ground_reading)
            if len(altitude_readings) >= 5:
                altitude_readings.pop(0)  # Remove the oldest reading
            altitude_readings.append(altitude)
            smoothed_altitude = sum(altitude_readings) / len(altitude_readings)
            #print(round(smoothed_altitude, 2))

        if shared_inputs.get("switch_auto"):

            current_time = millis()
            if smoothed_altitude > CEILING_HEIGHT:
                move_cg_rail(7)
                continue
            if smoothed_altitude < FLOOR_SAFETY_HEIGHT:
                move_cg_rail(18)
                if current_time - time_getoff_ceilfloor >= 10000:
                    time_getoff_ceilfloor = current_time
                    forward()
                    continue


            err_ball = ballCam()
            if err_ball is not None:
                tracking_ball = True
            else:
                tracking_ball = False
            
            if current_time - last_psensor_check >= psensor_check_interval and not gridlock:
                gridlock_check()

            if gridlock:
                gridlock_active()
                continue
            

            if not tracking_ball and not ball_on_target:

                '''THIS WILL BE THE SEARCHING LOOP'''
                current_time = millis()
                if current_time - last_psensor_check >= psensor_check_interval and not gridlock:
                    gridlock_check()

                if gridlock:
                    gridlock_active()
                    continue

                if current_time - halflap_time < halflap_time_int and not no_track:
                    #print("No ball found. Searching...")
                    forward()
                    continue

                elif not no_track and not now_uturn:
                    start_uturn = current_time
                    now_uturn = True

                if not no_track and now_uturn:
                    #print("No ball after search. Initiating U-turn...")

                    if current_time - start_uturn <= 30000:
                        uturn()
                        continue

                    else:
                        halflap_time = current_time
                        now_uturn = False

            if tracking_ball:
                current_time = millis()
                if current_time - time_printing >= 3000:
                    time_printing = current_time
                    print("NOW TRACKING BALL")
                err_y_ball = err_ball[0]
                err_z_ball = err_ball[1]
                area_blob = err_ball[2]
                err_y_history.append(err_y_ball)
                err_z_history.append(err_z_ball)

                smooth_err_y = sum(err_y_history) / len(err_y_history)
                smooth_err_z = sum(err_z_history) / len(err_z_history)
                #print(smooth_err_y, smooth_err_z)

                if abs(smooth_err_y) < 0.15 and abs(smooth_err_z) < 0.15 and not ball_on_target:
                    print("Ball centered, stopping adjustments.")
                    ball_on_target = True
                    ball_target_time = millis()
                else:

                    ball_on_target = False


                if not ball_on_target:
                
                    '''NEED TO WRITE AN APPROACH BALL / CATCH BALL CODE'''

                    # Send values to servos
                    
                    if smooth_err_y < -0.15:
                        #print("FLAPPING LEFT WING")
                        current_time = millis()
                        if current_time - last_move_timeL >= 80:  # Check if it's time to move
                            last_move_timeL = current_time  # Update last move time
                            posL = oscillate_wing(direction=1,freq=0.7)
                            
                            kit.servo[1].angle = posL
                            kit.servo[0].angle = 90
                            kit.servo[11].angle = 15
                            
                                    
                    if smooth_err_y > 0.15:
                        #print("FLAPPING RIGHT WING")
                        current_time = millis()
                        if current_time - last_move_timeR >= 80:  # Check if it's time to move
                            last_move_timeR = current_time  # Update last move time
                            posR = oscillate_wing(direction= 1, freq= 0.7)

                            kit.servo[0].angle = posR 
                            kit.servo[1].angle = 90
                            kit.servo[11].angle = 165   
                                
                    
                    # Compute CG rail movement for pitch
                    current_time_cg_check = millis()
                    '''
                    if current_time_cg_check - last_checkcg_time >= cg_int:
                        last_checkcg_time = current_time_cg_check
                        if abs(smooth_err_z) > 0.1:
                            current_time = millis()
                            cg_rail_stop = nonlinear_map_cg(smooth_err_z, PITCH_SCALE)
                            if current_time - time_printing >= 3000:
                                print(f"ERROR CAUSING CG RAIL TO MOVE: {smooth_err_z}")
                                print(f"CG RAIL DESTINATION IS: {cg_rail_stop}")
                            cg_move = True
                        else:
                            cg_move = False
                            kit.continuous_servo[6].throttle = -0.05

                    if cg_move and not move_cg_rail(cg_rail_stop):
                        continue
                    else:
                        cg_move = False
                    '''
                else:
                    current_time = millis()
                    if area_blob < 1000:
                        forward()
                        continue
                    else:
                        print("BALL CAPTURED")
                        break



        else:

            manual_control(shared_inputs)

        

    return True #Returns true when it thinks it caught ball
                

'''GOAL SCORING CODE'''

def goalSearch(shared_inputs):
    global altitude, altitude_readings, current_time, alt_time, ground_reading, smoothed_altitude, alt_time, goal_scored


    goal_preset_height = 15
    in_goal_zone = False

    goal_on_target = False
    goal_scored = False
    tracking_goal = False


    print("now attempting to score")

    while True:
        if shared_inputs["stop"]:
            break  # Exit cleanly 
        current_time = millis()
        if current_time - alt_time >= 3000:
            alt_time = current_time
            altitude = barometer.get_height(ground_reading)
            if len(altitude_readings) >= 5:
                altitude_readings.pop(0)  # Remove the oldest reading
            altitude_readings.append(altitude)
            smoothed_altitude = sum(altitude_readings) / len(altitude_readings)
            #print(round(smoothed_altitude, 2))

        

        if shared_inputs.get("switch_auto"):
            current_time = millis()
            err_goal = goalCam()
            if err_goal is not None and in_goal_zone:
                tracking_goal = True
            else:
                tracking_goal = False
            
            '''Possibly have a 'rate of change' variable'''
            '''
            if not in_goal_zone and not tracking_goal:

                if smoothed_altitude - goal_preset_height > 4:

                    cg_rail_stop = 2
                    if not move_cg_rail(cg_rail_stop):
                        continue
                    else:
                        forward()
                        continue

                elif altitude - goal_preset_height < 0:
                    cg_rail_stop = 18
                    if not move_cg_rail(cg_rail_stop):
                        continue
                    else:
                        forward()
                        continue

                else:
                    in_goal_zone = True
                    cg_rail_stop = 10
                    if not move_cg_rail(cg_rail_stop):
                        continue
                    else:
                        kit.continuous_servo[6].throttle = -0.05
            '''

            if in_goal_zone:

                if not tracking_goal:
                    '''Slowly go forward until GOAL is detected'''
                    forward(freq=0.5) #Slower frequency

                    continue

                else:
                    err_y_goal = err_goal[0]
                    err_z_goal = err_goal[1]
                    err_y_g_history.append(err_y_goal)
                    err_z_g_history.append(err_z_goal)

                    smooth_err_y_g = sum(err_y_g_history) / len(err_y_g_history)
                    smooth_err_z_g = sum(err_z_g_history) / len(err_z_g_history)
                    print(smooth_err_y_g, smooth_err_z_g)
                    
                    '''NEED TO WRITE AN APPROACH Goal / Score CODE'''
                    if abs(smooth_err_y_g) < 0.15 and abs(smooth_err_z_g) < 0.15:
                        print("Goal centered, stopping adjustments.")
                        goal_on_target = True
                        goal_target_time = millis()
                    else:
                        goal_on_target = False


                    if not goal_on_target and not goal_scored:
                        # Send values to servos
                        if smooth_err_y_g < -0.15:
                            print("FLAPPING LEFT WING")
                            current_time = millis()
                            if current_time - last_move_timeL >= 80:  # Check if it's time to move
                                last_move_timeL = current_time  # Update last move time
                                posL = oscillate_wing(direction=1,freq=0.7)
                                
                                kit.servo[1].angle = posL
                                kit.servo[0].angle = 90
                                kit.servo[11].angle = 15
                                
                                        
                        if smooth_err_y_g> 0.15:
                            print("FLAPPING RIGHT WING")
                            current_time = millis()
                            if current_time - last_move_timeR >= 80:  # Check if it's time to move
                                last_move_timeR = current_time  # Update last move time
                                posR = oscillate_wing(direction= 1, freq= 0.7)

                                kit.servo[0].angle = posR 
                                kit.servo[1].angle = 90
                                kit.servo[11].angle = 165   
                        
                        # Compute CG rail movement for pitch
                        current_time_cg_check = millis()
                        '''
                        if current_time_cg_check - last_checkcg_time >= cg_int:
                            last_checkcg_time = current_time
                            if abs(smooth_err_z_g) > 0.1:
                                print(f"ERROR CAUSING CG RAIL TO MOVE: {smooth_err_z_g}")
                                current_time = millis()
                                cg_rail_stop = nonlinear_map_cg(smooth_err_z_g, PITCH_SCALE)
                                print(cg_rail_stop)
                                cg_move = True
                            else:
                                cg_move = False
                                kit.continuous_servo[6].throttle = -0.05


                        if cg_move and not move_cg_rail(cg_rail_stop):
                            continue
                        else:
                            cg_move = False
                        '''
                    

                    if goal_on_target:
                        current_time = millis()
                        if current_time - goal_target_time <= 10000:
                            forward()
                            continue
                        else:
                            print("GOAL SCORED")
                            goal_scored = True
                            #shared_inputs("switch_auto") = False
                            break


        else:
            
            manual_control(shared_inputs)

    return True
                    
    

