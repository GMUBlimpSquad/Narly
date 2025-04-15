from evdev import InputDevice, categorize, ecodes
import time


def read_controller(shared_inputs):
    device = InputDevice('/dev/input/event2')  # adjust this based on your device

    switched = False
    try:

        for event in device.read_loop():
            if shared_inputs.get("stop"):
                break

            if event.type == ecodes.EV_ABS and event.code == ecodes.ABS_HAT0Y:
                shared_inputs["dpad_up"] = event.value == -1
                shared_inputs["dpad_down"] = event.value == 1

            elif event.type == ecodes.EV_ABS and event.code == ecodes.ABS_HAT0X:
                shared_inputs["dpad_left"] = event.value == -1
                shared_inputs["dpad_right"] = event.value == 1

            elif event.code == ecodes.BTN_SOUTH:  # forward
                shared_inputs["cg_forward"] = bool(event.value)
                if event.value == 1:
                    shared_inputs["cg_backward"] = False

            elif event.code == ecodes.BTN_EAST:  # backward
                shared_inputs["cg_backward"] = bool(event.value)
                if event.value == 1:
                    shared_inputs["cg_forward"] = False

            elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TR:  # Top Right Bumper (R1)
                if event.value == 1 and not switched:
                    shared_inputs["switch_auto"] = True
                    switched = True
                elif event.value == 1 and switched:
                    shared_inputs["switch_auto"] = False
                    switched = False


    finally:
        device.close()
