import time
import autonomy
from multiprocessing import Process, Manager
from r_control import read_controller
import signal
import sys
import threading

def main():
    manager = Manager()
    shared_inputs = manager.dict({
        "dpad_up": False,
        "dpad_down": False,
        "dpad_left": False,
        "dpad_right": False,
        "cg_forward": False,
        "cg_backward": False,
        "switch_auto": False,
        "stop": False,
    })

    controller_process = Process(target=read_controller, args=(shared_inputs,))
    controller_process.daemon = True
    controller_process.start()

    exit_requested = {"value": False}

    def reset_shared_inputs():
        for key in shared_inputs.keys():
            shared_inputs[key] = False

    def shutdown_handler(signum=None, frame=None):
        if not exit_requested["value"]:
            print("\n[Main] Shutdown signal received.")
        shared_inputs["stop"] = True
        controller_process.join(timeout=3)
        reset_shared_inputs()
        print("[Main] Cleanup complete. Exiting.")
        sys.exit(0)

    # Register shutdown signals
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    def listen_for_exit():
        while True:
            user_input = input()
            if user_input.lower().strip() == 'q':
                exit_requested["value"] = True
                shutdown_handler()
                break

    # Start the input-listening thread
    exit_thread = threading.Thread(target=listen_for_exit, daemon=True)
    exit_thread.start()

    try:
        print("[Main] Starting up, press 'q' then Enter to exit.")
        caught_ball = autonomy.ball_search_loop(shared_inputs)
        if caught_ball:
            scored = autonomy.goalSearch(shared_inputs)

    except KeyboardInterrupt:
        shutdown_handler()

    except Exception as e:
        print(f"[Main] Exception occurred: {e}")
        shutdown_handler()

    finally:
        if not exit_requested["value"]:
            shutdown_handler()

if __name__ == "__main__":
    main()
