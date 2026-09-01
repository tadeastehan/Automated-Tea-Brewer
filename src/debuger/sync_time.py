#!/usr/bin/env python3
"""
sync_time.py - Automatic Internet / System Time Synchronizer for ESP32 D8563TS RTC

Usage:
    python sync_time.py                  # Auto-detects COM port and syncs time
    python sync_time.py -p COM7          # Syncs time to specific COM port
    python sync_time.py -p COM7 --local  # Syncs using local timezone (instead of UTC)
    python sync_time.py -p COM7 -m       # Syncs time and stays in serial monitor mode
"""

import sys
import time
import argparse
import urllib.request
import json
from datetime import datetime, timezone

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] 'pyserial' is required. Please install it using: pip install pyserial")
    sys.exit(1)


def get_network_time():
    """Fetch current UTC timestamp from worldtimeapi or fall back to system time."""
    print("[1/3] Fetching accurate time...")
    try:
        req = urllib.request.Request(
            "http://worldtimeapi.org/api/timezone/Etc/UTC",
            headers={"User-Agent": "ESP32-Time-Sync/1.0"}
        )
        with urllib.request.urlopen(req, timeout=3) as resp:
            if resp.status == 200:
                data = json.loads(resp.read().decode())
                ts = data["unixtime"]
                dt = datetime.fromtimestamp(ts, tz=timezone.utc)
                print(f"      -> Source: WorldTimeAPI (Internet NTP)")
                print(f"      -> Current UTC Time: {dt.strftime('%Y-%m-%d %H:%M:%S')} (Unix: {ts})")
                return ts, dt
    except Exception:
        pass

    # Fallback to local machine clock
    ts = int(time.time())
    dt = datetime.now(timezone.utc)
    print(f"      -> Source: System Clock (Offline Fallback)")
    print(f"      -> Current UTC Time: {dt.strftime('%Y-%m-%d %H:%M:%S')} (Unix: {ts})")
    return ts, dt


def auto_detect_port():
    """Find connected ESP32 / USB Serial device."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    # Look for common USB Serial / Espressif keywords
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid for k in ["espressif", "usb serial", "ch340", "cp210", "uart", "jtag"]):
            return p.device

    # Default to first available port
    return ports[0].device


def sync_rtc(port, timestamp, baudrate=115200, stay_in_monitor=False):
    print(f"[2/3] Connecting to ESP32 on {port} @ {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Failed to open {port}: {e}")
        return False

    time.sleep(0.5)
    ser.reset_input_buffer()

    cmd = f"rtc set {timestamp}\r\n"
    print(f"[3/3] Sending sync command: {cmd.strip()} ...")
    ser.write(cmd.encode("utf-8"))
    ser.flush()

    # Read response
    start_time = time.time()
    confirmed = False
    print("\n--- ESP32 Console Output ---")
    while time.time() - start_time < 2.5:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            print(f"  ESP32 > {line}")
            if "[OK]" in line or "Setting D8563TS" in line:
                confirmed = True

    print("----------------------------\n")
    if confirmed:
        print(f"✅ SUCCESS: D8563TS RTC time synchronized successfully to Unix timestamp {timestamp}!")
    else:
        print(f"⚠️ Sent command, but did not receive explicit [OK] response. Please verify with 'rtc' in terminal.")

    if stay_in_monitor:
        print("\n--- Entering Interactive Serial Monitor (Press Ctrl+C to exit) ---")
        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore")
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
        except KeyboardInterrupt:
            print("\nExiting monitor.")

    ser.close()
    return confirmed


def main():
    parser = argparse.ArgumentParser(description="Synchronize ESP32 D8563TS RTC time via serial connection")
    parser.add_argument("-p", "--port", type=str, help="Serial COM port (e.g. COM7 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-m", "--monitor", action="store_true", help="Keep serial monitor open after sync")
    parser.add_argument("--local", action="store_true", help="Use local timezone offset instead of UTC")
    args = parser.parse_args()

    port = args.port
    if not port:
        port = auto_detect_port()
        if not port:
            print("[ERROR] No serial ports found. Please connect your ESP32 or specify --port COMx")
            sys.exit(1)
        print(f"[INFO] Auto-detected port: {port}")

    ts, dt = get_network_time()
    if args.local:
        local_now = datetime.now()
        ts = int(local_now.timestamp())
        print(f"[INFO] Using Local Time: {local_now.strftime('%Y-%m-%d %H:%M:%S')} (Unix: {ts})")

    sync_rtc(port, ts, args.baud, args.monitor)


if __name__ == "__main__":
    main()
