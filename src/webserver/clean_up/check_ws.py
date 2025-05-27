#!/usr/bin/env python3
import sys
print("Python EXE:", sys.executable)
try:
    import websockets
    print("websockets at:", websockets.__file__)
except ImportError:
    print("❌ websockets NOT found")