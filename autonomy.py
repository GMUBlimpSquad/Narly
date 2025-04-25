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
psensor_check_interval = 50
psensor_trig_count_int = 3000
first_hit = 0
rec_hit = 0
psensor_trig_count = 0
recent_hit = 0
stucksensor = False   #I NEED TO HAVE A WHAT IF SENSOR IS STUCK CODE
last_move_timeB = 0
posB = 90


def psensor_triggered():
    """Returns True if the limit switch is pressed."""
    return not psensor_switch.value  # UP means the switch is pressed


total_stops = 20

def nonlinear_map_cg(error, scale=50, total_stops=20, center_stop=10, min_stop=6, max_stop=15):
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
flap_int = 70
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
        kit.servo[8].angle = 80

#Variables (uturn() function)
r_pos = 90
b_pos = 30
start_uturn = 0

def uturn():
    global current_time, last_move_time_forward, r_pos, b_pos, L_pos
    current_time = millis()
    
    if current_time - last_move_time_forward >= flap_int:
        last_move_time_forward = current_time  # Update time

        r_pos = oscillate_wing(direction= 1, freq= 0.9)
        L_pos = 180 - r_pos
        b_pos = 165
        
        # Move the servos
        kit.servo[0].angle = r_pos
        kit.servo[8].angle = b_pos
        kit.servo[1].angle = L_pos
        
        

now_uturn = False

# Function to map values from one range to another
def map_value(value, fromLow, fromHigh, toLow, toHigh):
    return ((value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow)

# Function to oscillate the wing with fixed frequency and a mapped angle
def oscillate_wing(direction=1, freq = 0.6):
    # Use the fixed frequency to oscillate the wing position
    angle = 90 + direction * 100 * math.sin((2 * freq * math.pi) * time.time())
    
    if angle > 180:
        angle = 180
    if angle < 0:
        angle = 0
    
    return angle

last_throttle = 0
lastcheck = 0
switch_state = False
toward_neg = False
toward_plus = False

def manual_control(shared_inputs):
    global last_move_time1,last_move_time2, last_move_time3, last_move_time4, current_time, forward_pos, back_pos, R_pos, L_pos, last_throttle, current_stop, first_press, switch_state, lastcheck, cg_rail_stop, current_stop

    switch_state = False

    current_time = millis()
    if shared_inputs.get("dpad_up"):
        if current_time - last_move_time1 >= 80:
            last_move_time1 = current_time  # Update time
            forward_pos = oscillate_wing( direction= 1, freq= 0.8)
            
            # Move the servos
            kit.servo[1].angle = forward_pos
            kit.servo[0].angle = 180 - forward_pos
            kit.servo[8].angle = 80

    if shared_inputs.get("dpad_down"): 
        current_time = millis()
        if current_time - last_move_time2 >= 70:
            last_move_time2 = current_time  # Update time
            back_pos = oscillate_wing(direction=1,freq=0.9)
            # Move the servos
            kit.servo[8].angle = back_pos
            kit.servo[1].angle = 180
            kit.servo[0].angle = 0
    
    if shared_inputs.get("dpad_left"):  
    
        current_time = millis()
        if current_time - last_move_time3 >= 80:
            last_move_time3 = current_time  # Update time
            R_pos = oscillate_wing(direction= 1, freq=0.9)
            L_pos = 180 - R_pos
            back_pos = 165

        # Move the servos
        kit.servo[0].angle = R_pos
        kit.servo[1].angle = L_pos
        kit.servo[8].angle = back_pos
    

    if shared_inputs.get("dpad_right"): 
        current_time = millis()
        if current_time - last_move_time4 >= 80:
            last_move_time4 = current_time  # Update time
            L_pos = oscillate_wing(direction=1,freq=0.9)
            R_pos = 180 - L_pos
            back_pos = 5
        
        # Move the servos
        kit.servo[1].angle = L_pos
        kit.servo[0].angle = R_pos
        kit.servo[8].angle = back_pos

    
    if shared_inputs.get("cg_forward"):
        current_time = millis()
        if current_time - lastcheck >= 500:
            lastcheck = current_time
            if cg_rail_stop >= 3:
                cg_rail_stop -=1


    elif shared_inputs.get("cg_backward"):
        current_time = millis()
        if current_time - lastcheck >= 500:
            lastcheck = current_time
            if cg_rail_stop <= 17:
                cg_rail_stop += 1

    if current_stop != cg_rail_stop:
        move_cg_rail(cg_rail_stop)
    else:
        kit.continuous_servo[6].throttle = -0.05

    

  

cg_debo = 0
last_dir = 0
time_printing = 0

def move_cg_rail(cg_rail_stop):
    global next_stop, current_stop, current_time, first_press, last_time_check, switch_state, cg_debo, last_dir, time_printing


    if current_stop != cg_rail_stop:
        if cg_rail_stop > current_stop:
            direction = -1
        elif cg_rail_stop < current_stop:
            direction = 1
        else:
            direction = -0.05
        
        next_stop = current_stop + 1 if current_stop < cg_rail_stop else current_stop - 1
        
        if current_time - time_printing >= 1500:
            time_printing = current_time
            print(f"> Moving towards stop: [{next_stop}]/[{cg_rail_stop}]")
        
        
        kit.continuous_servo[6].throttle = direction

        if current_time - cg_debo >= 150:
            cg_debo = current_time
            switch_state = limit_switch_triggered()
            if switch_state and not first_press:
                #print("First Press")
                current_stop = next_stop
                first_press = True
                if current_stop != cg_rail_stop:
                    return False
                else:
                    kit.continuous_servo[6].throttle = -0.05
                    print("Destination Reached!")
                    return True
            elif not switch_state and first_press:
                #print("Released")
                first_press = False   

        return False
    else:
        kit.continuous_servo[6].throttle = -0.05
        
        return True
    

def set_cg_ten():
    global current_stop, cg_rail_stop
    print("Now setting current_stop to 10")
    current_stop = 10
    cg_rail_stop = 10



ground_reading = 0
time_printing = 0
halflap_time_int = 30000

time_getoff_ceilfloor = 0
start_towards_ball = 0

ground_reading = barometer.set_ground()
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
time_getoff_ceilfloor = 0
move_cg = False

GOAL_HEIGHT = 6 #CHANGE THIS WHEN YOU KNOW
CEILING_HEIGHT = 10 #CHANGE THIS WHEN YOU KNOW
FLOOR_SAFETY_HEIGHT = 2 #CHANGE THIS IF NEEDED


def ball_search_loop():
    global posL, posR, current_time, last_move_timeL, last_move_timeR, cg_rail_stop, last_checkcg_time, last_psensor_check, recent_hit, last_move_timeB, posB, halflap_time, start_uturn, now_uturn, no_track, tracking_ball, last_move_time_backward, altitude, altitude_readings, alt_time, ground_reading, time_printing, smoothed_altitude, GOAL_HEIGHT, CEILING_HEIGHT, time_getoff_ceilfloor, rec_hit, first_hit, gridlock, psensor_trig_count, gridlock_start, current_stop, start_towards_ball, move_cg, ball_on_target, psensor_trigger, halflap_time_int
    

    current_time = millis()
    if current_time - alt_time >= 3000:
        alt_time = current_time
        altitude = barometer.get_height(ground_reading)
        if len(altitude_readings) >= 5:
            altitude_readings.pop(0)  # Remove the oldest reading
        altitude_readings.append(altitude)
        smoothed_altitude = sum(altitude_readings) / len(altitude_readings)
        #print(round(smoothed_altitude, 2))

    err_ball = ballCam()
    if err_ball is not None:
        tracking_ball = True
    else:
        tracking_ball = False

    if current_time - last_psensor_check >= psensor_check_interval and not gridlock:
        
        last_psensor_check = current_time
        
        psensor_trigger = psensor_triggered()
        if psensor_trigger:
            
            if first_hit == 0:
                first_hit = millis()
            
            if current_time-first_hit <= 20000 and current_time - rec_hit >= 750:
                rec_hit = current_time
                psensor_trig_count += 1
                
                print(f"Now at : {psensor_trig_count}/2 triggers in 20 seconds")
                
            elif current_time - first_hit >= 20000:
                psensor_trig_count = 1
                first_hit = 0
                rec_hit = 0


    
    if psensor_trig_count == 2 and not gridlock:
        gridlock = True
        psensor_trigger = False
        first_hit = 0
        rec_hit = 0
        gridlock_start = millis()
        psensor_trig_count = 0
            


    if gridlock:
        if current_time - gridlock_start <= 10:
            print("Gridlock Activated")
        current_time = millis()
        if current_time - last_move_timeB >= 50:  # Check if it's time to move
            last_move_timeB = current_time  # Update last move time
            posB = oscillate_wing(direction=1,freq=0.9)
            kit.servo[8].angle = posB
            kit.servo[0].angle = 0
            kit.servo[1].angle = 180
            

        if current_time - gridlock_start >= 15000:
            gridlock = False
            halflap_time = 0

        return False


    if not tracking_ball:
        '''THIS WILL BE THE SEARCHING LOOP'''
        if smoothed_altitude > 7:
            cg_rail_stop = 5
            if move_cg_rail(cg_rail_stop):
                uturn()
            return False
        else: 
            cg_rail_stop = 11
            move_cg_rail(cg_rail_stop)
            
        current_time = millis()
    
        if halflap_time == 0 or current_time - halflap_time >= 50000:
            halflap_time = current_time
        if current_time - halflap_time <= halflap_time_int:
            #print("No ball found. Searching...")
            forward()
            return False

        if not now_uturn:
            start_uturn = current_time
            now_uturn = True

        else:
            #print("No ball after search. Initiating U-turn...")

            if current_time - start_uturn <= 6000:
                uturn()
                return False

            else:
                halflap_time = current_time
                now_uturn = False

    if tracking_ball:
        
        if current_time - time_printing >= 3000:
            time_printing = current_time
            print("NOW TRACKING BALL")
        err_y_ball = err_ball[0]
        err_z_ball = err_ball[1]
        
        err_y_history.append(err_y_ball)
        err_z_history.append(err_z_ball)


        smooth_err_y = sum(err_y_history) / len(err_y_history)
        smooth_err_z = sum(err_z_history) / len(err_z_history)
        #print(smooth_err_y, smooth_err_z)


        if smooth_err_z > 0.1:
            cg_rail_stop = 4
            move_cg_rail(cg_rail_stop)
        elif smooth_err_z < -0.1:
            cg_rail_stop = 17
            move_cg_rail(cg_rail_stop)
        else:
            kit.continuous_servo[6].throttle = -0.05

        
        current_time = millis()
        if current_time - last_move_timeR >= 80:  # Check if it's time to move
            last_move_timeR = current_time  # Update last move time
            posR = oscillate_wing(direction= 1, freq= 0.6)
            kit.servo[0].angle = posR 
            kit.servo[1].angle = 180 - posR
        
        if smooth_err_y < -0.1:
            #print("FLAPPING LEFT WING")
        
            kit.servo[8].angle = 15
                
                        
        elif smooth_err_y > 0.1:
                
            kit.servo[8].angle = 165   
        
        else:
            
            kit.servo[8].angle = 80   

        return False
                        
            
            


        
    

'''GOAL SCORING CODE'''
  
psensor_trigger = False
goal_on_target = False
goal_scored = False
tracking_goal = False
appr_goal = 0
now_uturn = False
start_uturn = 0
gridlock = False




def goalSearch():
    global posL, posR, current_time, last_move_timeL, last_move_timeR, cg_rail_stop, last_checkcg_time, last_psensor_check, recent_hit, last_move_timeB, posB, halflap_time, start_uturn, now_uturn, no_track, tracking_ball, last_move_time_backward, altitude, altitude_readings, alt_time, ground_reading, time_printing, smoothed_altitude, GOAL_HEIGHT, CEILING_HEIGHT, time_getoff_ceilfloor, rec_hit, first_hit, gridlock, psensor_trig_count, gridlock_start, current_stop, start_towards_ball, move_cg, goal_on_target, tracking_goal, appr_goal, psensor_trigger, goal_target_time, last_move_time_forward, forward_pos

    
    current_time = millis()
    if current_time - alt_time >= 3000:
        alt_time = current_time
        altitude = barometer.get_height(ground_reading)
        if len(altitude_readings) >= 5:
            altitude_readings.pop(0)  # Remove the oldest reading
        altitude_readings.append(altitude)
        smoothed_altitude = sum(altitude_readings) / len(altitude_readings)
        #print(round(smoothed_altitude, 2))


    if smoothed_altitude - GOAL_HEIGHT < -0.5:
        cg_rail_stop = 17
        move_cg_rail(cg_rail_stop)
        uturn()
        return False

    else:
        if current_stop != 11:
            cg_rail_stop = 11
            move_cg_rail(cg_rail_stop)

    current_time = millis()
    err_goal = goalCam()
    if err_goal is not None:
        tracking_goal = True
    else:
        tracking_goal = False

    if current_time - last_psensor_check >= psensor_check_interval and not gridlock:
        last_psensor_check = current_time
        
        psensor_trigger = psensor_triggered()

        if psensor_trigger:
            
            if first_hit == 0:
                first_hit = current_time
            
            if current_time-first_hit <= 20000 and current_time - rec_hit >= 750:
                rec_hit = current_time
                psensor_trig_count += 1
                
                print(f"Now at : {psensor_trig_count}/2 triggers in 20 seconds")
            elif current_time - first_hit > 20000:
                psensor_trig_count = 1
                print(f"Now at : {psensor_trig_count}/2 triggers in 20 seconds")
                first_hit = 0
                rec_hit = 0


    
    if psensor_trig_count == 2 and not gridlock:
        gridlock = True
        psensor_trigger = False
        first_hit = 0
        rec_hit = 0
        gridlock_start = millis()
        psensor_trig_count = 0
            

    if gridlock:
        if current_time - gridlock_start <= 10:
            print("Gridlock Activated")
        current_time = millis()
        if current_time - last_move_timeB >= 50:  # Check if it's time to move
            last_move_timeB = current_time  # Update last move time
            posB = oscillate_wing(direction=1,freq=0.9)
            kit.servo[8].angle = posB
            kit.servo[0].angle = 0
            kit.servo[1].angle = 180
            

        if current_time - gridlock_start >= 13000:
            gridlock = False
            appr_goal = 0

        return False


    if not tracking_goal:
        if smoothed_altitude - GOAL_HEIGHT > 4:

            cg_rail_stop = 5
            move_cg_rail(cg_rail_stop)
            uturn()
            return False

        elif smoothed_altitude - GOAL_HEIGHT < -0.5:
            cg_rail_stop = 17
            move_cg_rail(cg_rail_stop)
            uturn()
            return False

        else:
            if current_stop != 12:
                cg_rail_stop = 12
                move_cg_rail(cg_rail_stop)
            
            


        '''Slowly go forward until GOAL is detected'''
        if appr_goal == 0:
            appr_goal = millis()
        if current_time - appr_goal <= 20000:
            forward(freq=0.5) #Slower frequency
            
        else:
            start_uturn = millis()
            now_uturn = True

        if now_uturn and current_time - start_uturn <= 20000:
            uturn()
        else:
            now_uturn = False
            appr_goal = 0

        return False

    else:
        err_y_goal = err_goal[0]
        err_z_goal = err_goal[1]
        err_y_g_history.append(err_y_goal)
        err_z_g_history.append(err_z_goal)
        area_goal = err_goal[2]

        smooth_err_y_g = sum(err_y_g_history) / len(err_y_g_history)
        smooth_err_z_g = sum(err_z_g_history) / len(err_z_g_history)

        #print(smooth_err_y_g, smooth_err_z_g)
        
        if smooth_err_z_g > 0.1:
            cg_rail_stop = 4
            move_cg_rail(cg_rail_stop)
        elif smooth_err_z_g < -0.1:
            cg_rail_stop = 17
            move_cg_rail(cg_rail_stop)
        else:
            kit.continuous_servo[6].throttle = -0.05

        if abs(smooth_err_y_g) < 0.1 and abs(smooth_err_z_g) < 0.1:
            #print("Goal centered, stopping adjustments.")
            goal_on_target = True
            goal_target_time = millis()
        else:
            goal_on_target = False


        if not goal_on_target:

            current_time = millis()
            if current_time - last_move_timeL >= 80:  # Check if it's time to move
                last_move_timeL = current_time  # Update last move time
                posL = oscillate_wing(direction=1,freq=0.5)
                
                kit.servo[1].angle = posL
                kit.servo[0].angle = 180-posL
            
            # Send values to servos
            if smooth_err_y_g < -0.1:
                #print("FLAPPING LEFT WING")
                
                kit.servo[8].angle = 5
                    
                            
            if smooth_err_y_g > 0.1:
               
                kit.servo[8].angle = 165   
            
            

        else:
            current_time = millis()
            direction = 1

            if area_goal > 1500:

                if current_time - last_move_time_forward >= flap_int:
                    last_move_time_forward = current_time  # Update time
                    forward_pos = oscillate_wing(direction, freq=0.6)
                    # Move the servos
                    kit.servo[1].angle = forward_pos
                    kit.servo[0].angle = forward_pos
                    kit.servo[8].angle = 80
            else:
                forward(freq=0.5)
                


    
                    
    



