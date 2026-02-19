#!/usr/bin/env python3
"""
serial_bridge.py

Bridges a local serial port (connected to the gate controller) with a remote
ESP8266 serial server (TCP socket), allowing traffic to flow in both directions
while logging all bytes to the console and optionally to a file.

Usage:
    python serial_bridge.py --port /dev/ttyUSB0 --baud 115200 --host 192.168.3.13 --tcp-port 10001

Press Ctrl+C to stop.
"""

import argparse
import serial
import socket
import threading
import time
import sys
from datetime import datetime


# ── Colour helpers ────────────────────────────────────────────────────────────

RESET  = "\033[0m"
CYAN   = "\033[96m"   # ESP8266 → local (data coming from remote)
YELLOW = "\033[93m"   # local   → ESP8266 (data going to remote)
RED    = "\033[91m"   # errors


def hex_dump(data: bytes, direction: str, colour: str) -> str:
    """Format bytes as a labelled hex string with timestamp."""
    ts  = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    hex_str = " ".join(f"{b:02X}" for b in data)
    return f"{colour}[{ts}] {direction:<20} {hex_str}{RESET}"


# ── Bridge threads ────────────────────────────────────────────────────────────

def serial_to_tcp(ser: serial.Serial, sock: socket.socket, log_file, stop_event: threading.Event):
    """Read from local serial port, forward to TCP socket."""
    while not stop_event.is_set():
        try:
            waiting = ser.in_waiting
            if waiting:
                data = ser.read(waiting)
                sock.sendall(data)
                line = hex_dump(data, "LOCAL → REMOTE", YELLOW)
                print(line)
                if log_file:
                    log_file.write(line + "\n")
                    log_file.flush()
            else:
                time.sleep(0.005)
        except serial.SerialException as e:
            print(f"{RED}Serial error: {e}{RESET}")
            stop_event.set()
        except (BrokenPipeError, OSError) as e:
            print(f"{RED}TCP send error: {e}{RESET}")
            stop_event.set()


def tcp_to_serial(sock: socket.socket, ser: serial.Serial, log_file, stop_event: threading.Event):
    """Read from TCP socket, forward to local serial port."""
    sock.settimeout(0.1)
    while not stop_event.is_set():
        try:
            data = sock.recv(4096)
            if not data:
                print(f"{RED}Remote closed connection.{RESET}")
                stop_event.set()
                break
            ser.write(data)
            line = hex_dump(data, "REMOTE → LOCAL", CYAN)
            print(line)
            if log_file:
                log_file.write(line + "\n")
                log_file.flush()
        except socket.timeout:
            continue
        except OSError as e:
            if not stop_event.is_set():
                print(f"{RED}TCP recv error: {e}{RESET}")
            stop_event.set()
        except serial.SerialException as e:
            print(f"{RED}Serial write error: {e}{RESET}")
            stop_event.set()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Serial ↔ TCP bridge with hex logging")
    parser.add_argument("--port",     required=True,          help="Local serial port (e.g. /dev/ttyUSB0 or COM3)")
    parser.add_argument("--baud",     type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--host",     default="192.168.3.13", help="ESP8266 serial server host")
    parser.add_argument("--tcp-port", type=int, default=10001, help="ESP8266 serial server TCP port")
    parser.add_argument("--log",      default=None,           help="Optional file to save hex log")
    args = parser.parse_args()

    log_file = open(args.log, "w") if args.log else None

    print(f"Opening serial port {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
        )
    except serial.SerialException as e:
        print(f"{RED}Could not open serial port: {e}{RESET}")
        sys.exit(1)

    print(f"Connecting to {args.host}:{args.tcp_port}...")
    try:
        sock = socket.create_connection((args.host, args.tcp_port), timeout=5)
    except (socket.timeout, OSError) as e:
        print(f"{RED}Could not connect to TCP server: {e}{RESET}")
        ser.close()
        sys.exit(1)

    print(f"Bridge running. {YELLOW}YELLOW = local→remote{RESET}, {CYAN}CYAN = remote→local{RESET}")
    print("Press Ctrl+C to stop.\n")

    stop_event = threading.Event()

    t1 = threading.Thread(target=serial_to_tcp, args=(ser, sock, log_file, stop_event), daemon=True)
    t2 = threading.Thread(target=tcp_to_serial, args=(sock, ser, log_file, stop_event), daemon=True)
    t1.start()
    t2.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping bridge...")
        stop_event.set()

    t1.join(timeout=1)
    t2.join(timeout=1)
    ser.close()
    sock.close()
    if log_file:
        log_file.close()
        print(f"Log saved to {args.log}")


if __name__ == "__main__":
    main()
