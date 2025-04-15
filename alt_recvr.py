import socket
import cv2
import pickle
import struct
import numpy as np
import threading

DATA_FORMAT = 'P'  # P for pickle, J for JPEG
HOST = '0.0.0.0'
PORT = 5001
DATA_BUFFER_SIZE = 4062

frame_lock = threading.Lock()
shared_frame = None


testing = True

def nothing(x):
    pass

def receive_thread(conn):
    global shared_frame
    data = b""
    payload_size = struct.calcsize(">L")

    while True:
        while len(data) < payload_size:
            packet = conn.recv(DATA_BUFFER_SIZE)
            if not packet:
                return
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += conn.recv(DATA_BUFFER_SIZE)

        frame_data = data[:msg_size]
        data = data[msg_size:]
        conn.send("OK".encode())

        if DATA_FORMAT == 'P':
            frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        else:
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        with frame_lock:
            shared_frame = frame

if __name__ == '__main__':
    server_socket = socket.socket()
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print('Waiting for connection...')
    conn, addr = server_socket.accept()
    print(f'Connected to: {addr}')

    # Start receiver thread
    t1 = threading.Thread(target=receive_thread, args=(conn,))
    t1.daemon = True
    t1.start()

    if testing:
        # GUI stuff in main thread
        cv2.namedWindow('image')
        cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
        cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
        cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
        cv2.createTrackbar('HMax', 'image', 179, 179, nothing)
        cv2.createTrackbar('SMax', 'image', 255, 255, nothing)
        cv2.createTrackbar('VMax', 'image', 255, 255, nothing)

    try:
        while True:
            with frame_lock:
                if shared_frame is None:
                    continue
                frame = shared_frame.copy()
            if testing:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                hMin = cv2.getTrackbarPos('HMin', 'image')
                sMin = cv2.getTrackbarPos('SMin', 'image')
                vMin = cv2.getTrackbarPos('VMin', 'image')
                hMax = cv2.getTrackbarPos('HMax', 'image')
                sMax = cv2.getTrackbarPos('SMax', 'image')
                vMax = cv2.getTrackbarPos('VMax', 'image')

                lower = np.array([hMin, sMin, vMin])
                upper = np.array([hMax, sMax, vMax])
                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(frame, frame, mask=mask)

                combined = np.hstack((frame, result))
                cv2.imshow('image', combined)
            else:
                cv2.imshow('Live Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted, closing...")
    finally:
        conn.close()
        cv2.destroyAllWindows()
