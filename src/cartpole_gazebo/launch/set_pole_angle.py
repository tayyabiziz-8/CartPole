#!/usr/bin/env python3
"""
One-shot node: sets pole_joint to 0.17 rad in Gazebo while paused.
Uses gz transport directly via subprocess.
"""
import subprocess
import time
import sys

def main():
    time.sleep(0.5)
    # Use gz service to move the joint via model pose
    # This uses the direct joint position command
    cmd = [
        'gz', 'topic',
        '-t', '/world/cartpole_world/model/cartpole/joint/pole_joint/cmd_pos',
        '-m', 'gz.msgs.Double',
        '-p', 'data: 0.17'
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    print(f'Set angle result: {result.stdout} {result.stderr}')

    # Also try via wrench
    cmd2 = [
        'gz', 'service',
        '-s', '/world/cartpole_world/model/cartpole/joint/pole_joint/cmd_pos',
        '--reqtype', 'gz.msgs.Double',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '1000',
        '--req', 'data: 0.17'
    ]
    result2 = subprocess.run(cmd2, capture_output=True, text=True)
    print(f'Service result: {result2.stdout} {result2.stderr}')

if __name__ == '__main__':
    main()
