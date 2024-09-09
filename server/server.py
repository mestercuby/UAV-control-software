import struct
import cv2
import socket
import threading
import time
import msgpack


class HandleMessageThread(threading.Thread):

    def __init__(self, client_socket, shared):
        super().__init__()
        self.buffer = 1024
        self.header = 64
        self.format = 'utf-8'
        self.client_socket = client_socket
        self.target = None
        self.shared = shared

    def run(self):
        # Receiving Message
        while True:
            try:
                while True:
                    data = b""
                    payload_size = struct.calcsize("L")
                    while len(data) < payload_size:
                        data += self.client_socket.recv(4096)
                    packed_msg_length = data[:payload_size]
                    data = data[payload_size:]
                    msg_length = struct.unpack("L", packed_msg_length)[0]

                    while len(data) < msg_length:
                        data += self.client_socket.recv(4096)
                    message = data[:msg_length].decode(self.format)
                    data = data[msg_length:]
                    print(f"[MESSAGE] {message}")
                    self.shared.update_mission(message)

                    time.sleep(0.1)
            except Exception as e:
                print(e)


class VideoServerThread(threading.Thread):
    def __init__(self, ip=socket.gethostbyname(socket.gethostname()), port=5050, buffer=1024, shared=None,position_estimator=None):
        super().__init__()
        self.buffer = buffer
        self.header = 64
        self.format = 'utf-8'
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((ip, port))
        self.timeout_seconds = 5
        self.shared = shared
        self.position_estimator = position_estimator
        self.client_socket = None
        print("[STARTING] server is starting...")
        print(f"[LISTENING] Server is listening on {ip}")

    def run(self):
        frame_time = 1 / 30

        while True:
            self.socket.listen()
            self.client_socket, addr = self.socket.accept()
            print(f"[NEW CONNECTION] {addr} connected.")
            try:
                while True:
                    start_time = time.time()
                    detections, frame, last_update = self.shared.get_detections()
                    
                    if last_update and time.time() - last_update > self.timeout_seconds:
                        print("Detections stream has stopped.")
                        data_to_send = msgpack.packb({"error": "Detections stream has stopped."})
                        self.client_socket.sendall(data_to_send)
                        break

                    detec_data = msgpack.packb(detections)
                    detec_data_length = struct.pack("L", len(detec_data))

                    Error_msg = self.shared.get_error_msg()
                    _, encoded_frame = cv2.imencode('.jpg', frame)
                    data = encoded_frame.tobytes()

                    message = Error_msg.encode(self.format)
                    msg_length = len(message)
                    send_length = struct.pack("L", msg_length)
                    message_size = struct.pack("L", len(data))

                    elapsed_time = time.time() - start_time
                    sleep_time = frame_time - elapsed_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    self.client_socket.sendall(
                        send_length + message + message_size + data + detec_data_length + detec_data)
                    self.shared.update_error_msg("")
            except Exception as e:
                print(e)
                self.client_socket.close()
                print(f"[DISCONNECTED] {addr} disconnected.")
