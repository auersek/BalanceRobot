#!/usr/bin/env python3
import asyncio
import websockets

HOST = "0.0.0.0"
PORT = 9002

async def handler(ws):
    print("[server] Client connected")
    try:
        async for msg in ws:
            print(f"[server] ← {msg!r}")
            # echo back an acknowledgement
            await ws.send(f"ACK:{msg}")
    except websockets.ConnectionClosed:
        print("[server] Connection closed")

async def main():
    async with websockets.serve(handler, HOST, PORT):
        print(f"[server] Listening on ws://{HOST}:{PORT}")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
