#!/usr/bin/env python3
"""Exercise all LiDAR node signal paths without LiDAR hardware."""

import os
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client


def wait_until_ready(process, node_name, timeout=8.0):
    master = xmlrpc.client.ServerProxy(
        os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
    )
    parameter = f"/{node_name}/ready"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return False
        try:
            code, _, value = master.getParam("/signal_exit_test", parameter)
            if code == 1 and value is True:
                return True
        except (ConnectionError, OSError, xmlrpc.client.Error):
            pass
        time.sleep(0.05)
    return False


def check_signal(binary, role, sig, active):
    node_name = f"signal_exit_{role}_{sig.name.lower()}_{int(active)}"
    command = [
        binary,
        f"__name:={node_name}",
        "_lidar_ip:=127.0.0.1",
        "_msop_port:=24567",
        "_difop_port:=24568",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    sender_stop = threading.Event()
    sender = None
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

        if not wait_until_ready(process, node_name, timeout=8.0 if active else 1.0):
            if active:
                output = ""
                if process.poll() is not None:
                    output, _ = process.communicate()
                raise AssertionError(
                    f"{role} did not reach running state before {sig.name}: "
                    f"exit={process.poll()}\n{output}"
                )
            time.sleep(0.2)

        sender_stop.set()
        if sender is not None:
            sender.join()
        if process.poll() is not None:
            output, _ = process.communicate()
            raise AssertionError(
                f"{role} exited before {sig.name}: {process.returncode}\n{output}"
            )
        process.send_signal(sig)
        try:
            output, _ = process.communicate(timeout=6.0)
        except subprocess.TimeoutExpired as exc:
            raise AssertionError(
                f"{role} did not exit within 6 seconds after {sig.name}, active={active}"
            ) from exc
        if process.returncode != 0:
            raise AssertionError(
                f"{role} exited abnormally after {sig.name}: "
                f"{process.returncode}\n{output}"
            )
        print(f"PASS {role} {sig.name} active={active}: exit=0")
    finally:
        sender_stop.set()
        if sender is not None and sender.is_alive():
            sender.join()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(
            "usage: test_node_signal_exit.py "
            "PATH_TO_BASIC_NODE PATH_TO_ENHANCED_NODE PATH_TO_PIPELINE_NODE"
        )
    for binary, role in ((sys.argv[1], "basic"), (sys.argv[2], "enhanced")):
        for active in (False, True):
            check_signal(binary, role, signal.SIGINT, active)
            check_signal(binary, role, signal.SIGTERM, active)
    for sig in (signal.SIGINT, signal.SIGTERM):
        check_signal(sys.argv[3], "pipeline", sig, active=True)
