import asyncio
import websockets

async def receive_data(websocket, path):
    async for message in websocket:
        print(message)

async def start_server():
    async with websockets.serve(receive_data, '0.0.0.0', 8765, ping_interval=5):
        await asyncio.Future()  # keep the server running

if __name__ == '__main__':
    asyncio.run(start_server())
