#!/usr/bin/env python3
"""
jetson_txrx.py

Jetson side: öffnet /dev/ttyTHS1, liest eingehende Zeilen (von Windows)
und sendet periodisch "Jetson Sendet #n".

Wichtig: Benutzer muss Zugriff auf /dev/ttyTHS1 haben (Gruppe 'dialout'
oder mit sudo starten).
Benutzung:
  python3 jetson_txrx.py --port /dev/ttyTHS1 --baud 115200
"""

import argparse
import threading
import time
import sys
import os
import serial

def has_device_access(path: str) -> bool:
    # einfache Zugriffsprüfung (r/w)
    return os.path.exists(path) and os.access(path, os.R_OK | os.W_OK)

def sender(ser: serial.Serial, stop_event: threading.Event, interval: float):
    n = 1
    while not stop_event.is_set():
        msg = f"Jetson Sendet #{n}\n"
        try:
            ser.write(msg.encode('utf-8'))
            ser.flush()
        except Exception as e:
            print("Send error:", e, file=sys.stderr)
            stop_event.set()
            break
        print("Sent ->", msg.strip())
        n += 1
        stop_event.wait(interval)

def receiver(ser: serial.Serial, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if not line:
                continue
            print("Received <-", line.decode('utf-8', errors='replace').rstrip())
        except Exception as e:
            print("Read error:", e, file=sys.stderr)
            stop_event.set()
            break

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyTHS1")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--interval", type=float, default=1.0)
    args = p.parse_args()

    if not has_device_access(args.port):
        print(f"Keine Berechtigung oder Device fehlt: {args.port}", file=sys.stderr)
        print("Lösung: sudo ausführen oder Benutzer zur Gruppe 'dialout' hinzufügen.", file=sys.stderr)
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print("Could not open port:", e, file=sys.stderr)
        sys.exit(1)

    stop_event = threading.Event()
    t_send = threading.Thread(target=sender, args=(ser, stop_event, args.interval), daemon=True)
    t_recv = threading.Thread(target=receiver, args=(ser, stop_event), daemon=True)

    t_send.start()
    t_recv.start()

    try:
        while t_send.is_alive() and t_recv.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping...")
        stop_event.set()

    t_send.join(timeout=1)
    t_recv.join(timeout=1)
    ser.close()
    print("Closed port.")

if __name__ == "__main__":
    main()
