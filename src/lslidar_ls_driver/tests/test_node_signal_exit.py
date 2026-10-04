#!/usr/bin/env python3
"""Exercise enhanced-node signal shutdown without LiDAR hardware."""

import os
import signal
import subprocess
import sys
import time


def check_signal(binary, sig):
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
        text=True,
        start_new_session=True,
    )
    try:
        time.sleep(1.0)
        if process.poll() is not None:
            raise AssertionError(f"node exited before {sig.name}: {process.returncode}")
        process.send_signal(sig)
        try:
            output, _ = process.communicate(timeout=6.0)
        except subprocess.TimeoutExpired as exc:
            raise AssertionError(f"node did not exit within 6 seconds after {sig.name}") from exc
        if process.returncode not in (0, 1):
            raise AssertionError(f"node exited abnormally after {sig.name}: {process.returncode}\n{output}")
        print(f"PASS {sig.name}: exit={process.returncode}")
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: test_node_signal_exit.py PATH_TO_ENHANCED_NODE")
    check_signal(sys.argv[1], signal.SIGINT)
    check_signal(sys.argv[1], signal.SIGTERM)
