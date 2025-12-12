# echo_imu_server.py

import asyncio
import websockets

IMU_WEBSOCKET_PORT = 8001

async def handler(websocket):
    async for message in websocket:
        print(message)  # write to stdout (can be tailed from another terminal)

async def main():
    print(f"IMU echo server running on ws://0.0.0.0:{IMU_WEBSOCKET_PORT}")
    async with websockets.serve(handler, "0.0.0.0", IMU_WEBSOCKET_PORT):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
