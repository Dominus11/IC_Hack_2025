# labelServer

# Libraries

import socket 


'''
ToDos:

- Establish connection to the ESP32
- Generate payloads from terminal (might need to add to script instead for this)
- Establish consistent labels for YoLo data
- Establish consistent format for YoLo payloads
- Serve payloads over WiFi

'''

class LabelServer:
    _destinationIP: int
    _portNumber: int
    _socket: socket.socket


    def __init__(self, destination, port):
        self._destinationIP = destination
        self._portNumber = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect()

    def connect(self):
        self._socket.connect(self._destinationIP, self._portNumber)
        self._socket.send("Attempting to connect".encode())
    
    def encode_label(label: str):
        pass


    def send_label(payload: bytearray):
        pass