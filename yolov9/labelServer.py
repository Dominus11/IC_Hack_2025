# labelServer

# Libraries

import asyncio, websockets, aiohttp


'''
ToDos:

- Establish connection to the ESP32
- Generate payloads from terminal (might need to add to script instead for this)
- Establish consistent labels that can be yielded from YoLo model
- Establish consistent format for YoLo payloads
- Serve payloads over WiFi

'''

class LabelServer:
    _destinationIP: str
    _portNumber: int
    OBSTACLES = ["chair", "dining table"]
    URL = "localhost:3000"


    def __init__(self, destination, port):
        self._destinationIP = destination
        self.portNumber = port
        
    # Start WebSocket server
    async def start_server(self):
        async with websockets.serve(self.send_data, self._destinationIP, self.portNumber):
            print(f"WebSocket server started on ws://{self.destinationIP}")
            await asyncio.Future()  # Keeps the server running

    # Run both server and client concurrently
    async def main(self):
        server_task = asyncio.create_task(self.start_server())
        #client_task = asyncio.create_task(self.receive_data())

        await asyncio.gather(server_task)
        

    
    def encode_label(self, label: str) -> bytes:
        # Send and encode label depending on object we're trying to identify
        # We have three axes to vary: Frequency, Fingers, Pulse duration/rhythm We have two variables to represent (here)

        # I find the most intuitive to vary object being pulse rhythm. We can effectively morse encode this
        # To vary danger I propose frequency
        # To vary direction I propose fingers

        if label == "person":
            return bytes([100])
        elif label in self.OBSTACLES:
            return bytes([200])
        else:
            return bytes([0])


    # WebSocket server function
    def send_data(self,payload):
        async def send_helper(websocket):
            #while True:
                await websocket.send(payload)
            
        return send_helper
            # await asyncio.sleep(2)  # Send every 2 seconds
        

     
    # WebSocket client function
    # WebSocket server function to handle received messages
    async def receive_data(self, websocket):
        async for message in websocket:
            print(f"Received: {message}")
            x, y = map(float, message.split(" "))
            async with aiohttp.ClientSession() as session:
                url = "http://localhost:3000/data"  # Replace with your actual endpoint
                data = {"x": x, "y": y}
                async with session.post(url, json=data) as response:
                    if response.status == 200:
                        print("Data posted successfully")
                    else:
                        print(f"Failed to post data: {response.status}")
    

    