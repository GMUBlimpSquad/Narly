import cv2
import numpy as np
import socket
import pickle
import struct
import time
import serial
import json
from picamera2 import Picamera2
import os


def map_value(value, fromLow, fromHigh, toLow, toHigh):
    return ((value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) +
            toLow)


class Detection:
    def __init__(self, host, port, stream=False,testing=False):
        self.stream = stream
        self.testing = testing
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={'format': 'BGR888', 'size': (1920, 1080)},
                                                        lores={"size": (240, 240)}, display="lores")
        self.picam2.configure(config)
        # self.picam2.preview_configuration.enable_lores()
        self.picam2.start()
        self.last_det = (time.time(), (0,0,0))
        if stream:
            self.client_socket = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((host, port))
            connection = self.client_socket.makefile('wb')


    def _get_frame(self,):
        frame_im = self.picam2.capture_array()
        return frame_im


    # Detection sould take in amother parameter to specify if it should
    # do goal detection or ball detection
    def detect(self, target, testing):
        return self.blob(target, stream=self.stream, testing=self.testing)


    # TODO Make this a tflite inference if possible
    def object(self, target=0):
        frame = self._get_frame()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (240, 240),
                           interpolation=cv2.INTER_LINEAR)
        data = pickle.dumps(frame, 0)
        size = len(data)
        start_send = time.time()
        self.client_socket.sendall(struct.pack(">L",target)+struct.pack(">L", size) + data)
        # print("Send time: " , time.time()-start_send)
        bounding_box = self.client_socket.recv(48)
        bb = bounding_box.decode().split(',')
        x, y, w, h = (int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3]))
        center_point = (x, y)

        if center_point[0] > 0 and center_point[1] > 0:
            self.last_det = (time.time(), (
                (center_point[0] - frame.shape[1] // 2) / (frame.shape[1] // 2),  # Center X at 0
                (frame.shape[0] // 2 - center_point[1]) / (frame.shape[0] // 2),  # Invert Y and center at 0
                max(w, h)
            ))
            return self.last_det[1]
        else:
            if time.time() - self.last_det[0] > 1:
                return None
            else:
                return self.last_det[1]


    def blob(self, target, stream=False, testing = False):
        frame = self._get_frame()

        frame = cv2.resize(frame, (240, 240),
                                   interpolation=cv2.INTER_LINEAR)
        
        #self.client_socket.sendall(struct.pack(">L", size) + data)
        # Set range for green color and
        # define mask


        #hlsFrame = cv2.GaussianBlur(frame, (5,5), 0)
        hlsFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # TODO this maks needs to be audjustable, and the update should be seen
        # in real time


        # Create an array of different color thresholds then based on that 
        # detect different object
        
        purple_ball_lower = np.array([170, 100, 106], np.uint8)
        purple_ball_upper = np.array([190, 255, 255], np.uint8)

        green_ball_lower = np.array([28, 95, 83], np.uint8)
        green_ball_upper = np.array([83, 255, 228], np.uint8)

        purple_ball_mask = cv2.inRange(hlsFrame, purple_ball_lower, purple_ball_upper)
        green_ball_mask = cv2.inRange(hlsFrame, green_ball_lower, green_ball_upper)
     
        ball_mask = cv2.bitwise_or(purple_ball_mask, green_ball_mask)

        yellow_goal_lower = np.array([80, 100, 100], np.uint8)
        yellow_goal_upper = np.array([100, 255, 255], np.uint8)


        orange_goal_lower = np.array([114, 100, 100], np.uint8)
        orange_goal_upper = np.array([115, 255, 255], np.uint8)

        yellow_goal_mask = cv2.inRange(hlsFrame, yellow_goal_lower, yellow_goal_upper)
        orange_goal_mask = cv2.inRange(hlsFrame, orange_goal_lower, orange_goal_upper)

        
        kernel = np.ones((7, 7), np.uint8)

        yellow_goal_mask = cv2.morphologyEx(yellow_goal_mask, cv2.MORPH_CLOSE, kernel)
        yellow_goal_mask = cv2.dilate(yellow_goal_mask, kernel, iterations=1)

        orange_goal_mask = cv2.morphologyEx(orange_goal_mask, cv2.MORPH_CLOSE, kernel)
        orange_goal_mask = cv2.dilate(orange_goal_mask, kernel, iterations=1)
    


        if target == 1:
            contours, _ = cv2.findContours(ball_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        elif target == 2:
            
            contours, _ = cv2.findContours(yellow_goal_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            frame = cv2.rectangle(frame, (x, y),
                                  (x + w, y + h),
                                  (0, 255, 0), 2)
            center_point = (x + w // 2, y + h // 2)

            if stream:

                if testing:
                    # 🟡 Draw red dot at center
                    cv2.circle(frame, center_point, 5, (0, 0, 255), -1)

                    # 🟡 Label coordinates
                    norm_x = (center_point[0] - 120) / 120
                    norm_y = (120 - center_point[1]) / 120  # Invert Y so up is positive
                    label = f"X: {norm_x:.2f}, Y: {norm_y:.2f}"

                    cv2.rectangle(frame, (center_point[0] + 10, center_point[1] - 20),
                                (center_point[0] + 150, center_point[1]), (0, 0, 0), -1)
                    cv2.putText(frame, label, (center_point[0] + 15, center_point[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                # 🔄 Then convert and send
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                data = pickle.dumps(frame, 0)
                size = len(data)

               # send data via socket
                self.client_socket.sendall(struct.pack(">L", size) + data)

            if target == 1:
                if w*h < 200 or w >= 2*h or h >= 2*w:
                    return None
            # print(x,y,w,h)
            

                return ((center_point[0] - 120) / 120, (120 - center_point[1]) / 120, w*h)
            
            if target == 2:
                if w*h < 100:
                    return None
                
                return ((center_point[0] - 120) / 120, (120 - center_point[1]) / 120, w*h)


        frame = cv2.resize(frame, (240, 240),
                           interpolation=cv2.INTER_LINEAR)

        return None

    def stop(self):
        self.picam2.close()
        self.client_socket.close()




if __name__ == "__main__":

    # Detection
    # That can do off board object detection -> Transfering the Data out 
    # to a server that then responds with the detection/Bounding boxes

    ofod = Detection("192.168.0.200", 5001,True,True) #TRUE FOR STREAMING and TRUE FOR TESTING
    while True:
        bounding = ofod.detect(target = 1, testing=True)
        if bounding is not None:
            print(bounding)
            pass
            



    # On board blob detection -> Which returns bounding box based on
    # the specified color that you want to detect



