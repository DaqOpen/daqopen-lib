"""
Module: helper
Description: various classes and functions as helper

Author: Michael Oberhofer
Created on: 2023-11-15
Last Updated: 2023-11-15

License: MIT

Notes: Only working within Unix

Version: 0.01
Github: https://github.com/DaqOpen/daqopen-lib/daqopen/
"""

import signal
import subprocess

def check_time_sync(sync_status):
    process = subprocess.Popen(['timedatectl'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    system_clock_sync = False
    rtc_time = False
    if not stderr:
        for item in stdout.decode().split('\n'):
            if item.strip().startswith("System clock synchronized:"):
                if 'yes' in item.strip().split(":")[1]:
                    system_clock_sync = True
            if item.strip().startswith("RTC time:"):
                if not 'n/a' in item.strip().split(":")[1]:
                    rtc_time = True
        sync_status[0] = system_clock_sync or rtc_time
    else:
        sync_status[0] = None

class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
    
    def exit_gracefully(self,signum, frame):
        self.kill_now = True
        print('Terminate App')