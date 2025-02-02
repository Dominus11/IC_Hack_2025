import asyncio
import websockets

# WebSocket server function
async def send_data(websocket):
    while True:
        message = "Hello from Laptop!"
        print(f"Sending: {message}")
        await websocket.send(message)
        await asyncio.sleep(2)  # Send every 2 seconds

# WebSocket server function to handle received messages
async def receive_data(websocket):
    async for message in websocket:
        print(f"Received: {message}")

# Start WebSocket server
async def start_server():
    async with websockets.serve(handler, "192.168.137.64", 8765):
        print("WebSocket server started on ws://192.168.137.64:8765")
        await asyncio.Future()  # Keeps the server running

# Handler to manage both sending and receiving
async def handler(websocket, path):
    send_task = asyncio.create_task(send_data(websocket))
    receive_task = asyncio.create_task(receive_data(websocket))
    await asyncio.gather(send_task, receive_task)

# Run the server
async def main():
    server_task = asyncio.create_task(start_server())
    await asyncio.gather(server_task)  # Run the server

asyncio.run(main())  # Start everything