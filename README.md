'''

Operating Narly

DEFAULT SETTINGS: 

In autonomy.py
  ofbd_cam = Detection("IP_ADDRESS", 5001,stream=False,testing=False)
    First False is for STREAMING CONTENT TO ALT_RECEIVER
    Second False is for TESTING HSV AND OBJECT COORDINATES




run main.py from RPi alone IF STREAM and TESTING are FALSE
else run alt_recvr.py on laptop and main.py on RPi


main.py uses MULTIPROCESSING to receive controller inputs while also running all code in autonomy.py

barometer.py is seperate code that is just called for barometer readings

r_control





'''
