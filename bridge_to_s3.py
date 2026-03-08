#!/usr/bin/env python3
"""
Bridge: ESP32-CAM serial monitor → ESP32-S3 toolkit

Reads the ESP32-CAM monitor output, watches for LABEL:<category> lines,
and instantly writes the label to the ESP32-S3 serial port.

Usage:
    python3 bridge_to_s3.py

Requires: pyserial  (pip3 install pyserial)
"""

import serial
import sys
import time
import re
import threading
import select

# ── Configuration ──────────────────────────────────────────────
CAM_PORT  = "/dev/cu.usbserial-1140"   # ESP32-CAM
S3_PORT   = "/dev/cu.usbmodem101"      # ESP32-S3 DevKitC
BAUD      = 115200
# ───────────────────────────────────────────────────────────────

LABEL_RE = re.compile(r'^LABEL:(\S+)', re.MULTILINE)

def main():
    print(f"[bridge] Opening CAM on {CAM_PORT} @ {BAUD}")
    print(f"[bridge] Opening S3  on {S3_PORT} @ {BAUD}")
    print(f"[bridge] Waiting for LABEL: lines...\n")

    try:
        cam = serial.Serial(CAM_PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"[bridge] ERROR: Cannot open CAM port {CAM_PORT}: {e}")
        print(f"[bridge] Make sure idf_monitor is NOT running (it locks the port).")
        print(f"[bridge] Use: idf.py -p {CAM_PORT} flash   (flash only, no monitor)")
        print(f"[bridge] Then run this script to be the monitor + bridge.")
        sys.exit(1)

    try:
        s3 = serial.Serial(S3_PORT, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"[bridge] ERROR: Cannot open S3 port {S3_PORT}: {e}")
        cam.close()
        sys.exit(1)

    print(f"[bridge] Connected! Streaming CAM output below.")
    print(f"[bridge] Type '1' + Enter here to trigger classification.\n")
    print("=" * 60)

    # Thread to read keyboard input and send to CAM
    def keyboard_thread():
        while True:
            try:
                line = input()
                if line.strip():
                    cam.write((line.strip() + '\n').encode('ascii'))
                    cam.flush()
            except (EOFError, KeyboardInterrupt):
                break

    kb = threading.Thread(target=keyboard_thread, daemon=True)
    kb.start()

    try:
        while True:
            line = cam.readline()
            if not line:
                continue

            # Decode and print everything from the CAM
            try:
                text = line.decode('utf-8', errors='replace').rstrip('\r\n')
            except Exception:
                continue

            print(text)

            # Check for LABEL: pattern
            m = LABEL_RE.search(text)
            if m:
                label = m.group(1)
                # Send to S3: lowercase label + newline
                msg = label.lower() + '\n'
                s3.write(msg.encode('ascii'))
                s3.flush()
                print(f"\n>>> SENT TO S3: {label} <<<\n")

    except KeyboardInterrupt:
        print("\n[bridge] Stopped.")
    finally:
        cam.close()
        s3.close()

if __name__ == "__main__":
    main()
