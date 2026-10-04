#!/usr/bin/env python3
"""Exercise enhanced-node signal shutdown without LiDAR hardware."""

import os
import select
import signal
import socket
import subprocess
import sys
import threading
import time


def check_signal(binary, sig, active=False):
    command = [
        binary,
        "_lidar_ip:=127.0.0.1",
        "_msop_port:=24567",
        "_difop_port:=24568",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    sender_stop = threading.Event()
    sender = None
    observed = b""
    try:
        if active:
            def send_mode_packet():
                packet = bytearray(1206)
                packet[1205] = 0x01
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    while not sender_stop.is_set():
                        sock.sendto(packet, ("127.0.0.1", 24567))
                        sender_stop.wait(0.02)

            sender = threading.Thread(target=send_mode_packet)
            sender.start()
            deadline = time.monotonic() + 8.0
            while b"Enhanced driver started" not in observed and time.monotonic() < deadline:
                if process.poll() is not None:
                    break
                readable, _, _ = select.select([process.stdout], [], [], 0.1)
                if readable:
                    observed += os.read(process.stdout.fileno(), 65536)
            sender_stop.set()
            sender.join()
            if b"Enhanced driver started" not in observed:
                raise AssertionError(
                    f"node did not reach active state before {sig.name}: "
                    f"exit={process.poll()}\n{observed.decode(errors='replace')}"
                )
        else:
            time.sleep(1.0)
        if process.poll() is not None:
            raise AssertionError(f"node exited before {sig.name}: {process.returncode}")
        process.send_signal(sig)
        try:
            remaining, _ = process.communicate(timeout=6.0)
        except subprocess.TimeoutExpired as exc:
            raise AssertionError(
                f"node did not exit within 6 seconds after {sig.name}, active={active}"
            ) from exc
        output = (observed + remaining).decode(errors="replace")
        expected = (0,) if active else (1,)
        if process.returncode not in expected:
            raise AssertionError(f"node exited abnormally after {sig.name}: {process.returncode}\n{output}")
        print(f"PASS {sig.name} active={active}: exit={process.returncode}")
    finally:
        sender_stop.set()
        if sender is not None and sender.is_alive():
            sender.join()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: test_node_signal_exit.py PATH_TO_ENHANCED_NODE")
    check_signal(sys.argv[1], signal.SIGINT)
    check_signal(sys.argv[1], signal.SIGTERM)
    check_signal(sys.argv[1], signal.SIGINT, active=True)
    check_signal(sys.argv[1], signal.SIGTERM, active=True)
