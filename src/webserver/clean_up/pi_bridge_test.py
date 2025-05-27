#!/usr/bin/env python3
import asyncio
import websockets

URI = "ws://localhost:9002"

async def send_hello():
    async with websockets.connect(URI) as ws:
        print("[client] ✅ Connected")
        await ws.send("hello world")
        print("[client] → hello world")
        reply = await ws.recv()
        print(f"[client] ← {reply!r}")

if __name__ == "__main__":
    asyncio.run(send_hello())
