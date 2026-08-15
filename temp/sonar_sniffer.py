#!/usr/bin/env python3
"""Standalone UDP sniffer for Lite3 state packets.

Prints the raw double array in 0x0901 packets to help locate ultrasonic fields.
Stop any running driver first to free the UDP port.
"""

import socket
import struct
import time

PORT = 43893


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(5.0)
    print(f"Listening on UDP port {PORT}...")

    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            print("Timeout, no packet received in 5s")
            continue
        except KeyboardInterrupt:
            break

        n = len(data)
        if n < 16:
            print(f"Packet too short: {n} bytes")
            continue

        code, value, cmd_type = struct.unpack_from("<IiI", data, 0)
        print(
            f"\n[{time.time():.3f}] len={n} code=0x{code:08X} "
            f"value={value} type={cmd_type}"
        )

        if n <= 20:
            continue

        doubles = []
        for i in range((n - 20) // 8):
            v = struct.unpack_from("<d", data, 20 + i * 8)[0]
            if abs(v) < 1000 and (abs(v) > 1e-3 or v == 0.0):
                doubles.append(f"[{i:2d}]={v:.3f}")
            else:
                doubles.append(f"[{i:2d}]={v:.3e}")

        for i in range(0, len(doubles), 6):
            print("  " + "  ".join(doubles[i : i + 6]))

    sock.close()
    print("\nStopped.")


if __name__ == "__main__":
    main()
