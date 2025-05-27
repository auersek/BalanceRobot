#!/usr/bin/env python3
import asyncio
import serial_asyncio
import websockets

SERIAL_PORT = "/dev/tty.usbserial-110"  # adjust to your USB port
BAUD_RATE   = 115200
WS_URI      = "ws://localhost:9002"

async def forward_serial_to_ws():
    reader, _ = await serial_asyncio.open_serial_connection(
        url=SERIAL_PORT, baudrate=BAUD_RATE)
    async with websockets.connect(WS_URI) as ws:
        print(f"[bridge] Connected to {WS_URI}")
        while True:
            line = await reader.readline()
            text = line.decode(errors="ignore").strip()
            if text:
                print(f"[bridge] → {text}")
                await ws.send(text)

if __name__ == "__main__":
    try:
        asyncio.run(forward_serial_to_ws())
    except KeyboardInterrupt:
        print("\n[bridge] Exiting…")
