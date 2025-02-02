import asyncio
import websockets
import aiohttp

URL = "localhost:3000"

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
        lst = map(lambda s: s.split(), message.split(';'))
        xs = [x for x, _ in lst]
        ys = [y for _, y in lst]
        async with aiohttp.ClientSession() as session:
            url = "http://localhost:3000/data"  # Replace with your actual endpoint
            data = {"xs": xs, "ys": ys}
            async with session.post(url, json=data) as response:
                if response.status == 200:
                    print("Data posted successfully")
                else:
                    print(f"Failed to post data: {response.status}")

# Start WebSocket server
async def start_server():
    async with websockets.serve(handler, "192.168.137.38", 8765):
        print("WebSocket server started on ws://192.168.137.38:8765")
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