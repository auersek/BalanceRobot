#!/usr/bin/env python3
import asyncio
import serial_asyncio
import websockets

# — EDIT THESE TO MATCH YOUR SETUP —
SERIAL_PORT = "/dev/tty.usbserial-0001"  # your ESP32 USB-serial
BAUD_RATE   = 115200
WS_URI      = "ws://localhost:9002"

async def forward_serial_to_ws():
    # 1) open serial port
    reader, _ = await serial_asyncio.open_serial_connection(
        url=SERIAL_PORT, baudrate=BAUD_RATE)

    # 2) connect to our WebSocket server
    async with websockets.connect(WS_URI) as ws:
        print(f"[bridge] Connected to {WS_URI}")
        while True:
            # read one line from ESP32
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
