# daqopen/helper.py

"""Module for various helper classes and functions.

This module provides utility functions and classes that assist with system operations 
such as checking time synchronization and handling graceful termination of processes. 
The functionality is tailored to Unix-based systems.

## Usage

The primary components of this module are:
- `check_time_sync`: A function that checks whether the system's clock is synchronized.
- `GracefulKiller`: A class that allows for graceful termination of a running application 
  upon receiving interrupt signals (e.g., SIGINT, SIGTERM).

Examples:
    Using `GracefulKiller` to handle termination signals:

    >>> killer = GracefulKiller()
    >>> while not killer.kill_now:
    >>>     # Perform long-running task
    >>>     pass
    >>> print("Application terminated gracefully.")

Classes:
    GracefulKiller: Handles system signals for graceful application termination.

Functions:
    check_time_sync: Checks if the system clock and RTC are synchronized.

Notes:
    - This module is intended for use on Unix-based systems.

"""


import signal
import subprocess

def check_time_sync(sync_status: list):
    """Check if the system clock and RTC are synchronized.

    This function uses the `timedatectl` command to check whether the system clock is 
    synchronized and if the RTC (Real-Time Clock) is set. The results are stored in 
    the provided `sync_status` list.

    Parameters:
        sync_status: A list to store the synchronization status. The first element 
                     is set to `True` if either the system clock or RTC is synchronized, 
                    `False` if neither is synchronized, and `None` if an error occurred.

    Examples:
        >>> sync_status = [False]
        >>> check_time_sync(sync_status)
        >>> print(sync_status[0])
        True  # If the system is synchronized
    """
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
    """Handles system signals for graceful application termination.

    `GracefulKiller` listens for interrupt signals (e.g., SIGINT, SIGTERM) and sets a flag 
    when such a signal is received, allowing the application to terminate gracefully.

    Attributes:
        kill_now: A flag indicating whether a termination signal has been received.

    Methods:
        exit_gracefully(signum, frame): Signal handler that sets the `kill_now` flag to True.

    Examples:
        >>> killer = GracefulKiller()
        >>> while not killer.kill_now:
        >>>     # Perform long-running task
        >>>     pass
        >>> print("Application terminated gracefully.")
    """
    kill_now: bool = False
    def __init__(self):
        """Initialize the GracefulKiller instance and set up signal handlers.

            Registers the signal handlers for SIGINT and SIGTERM, which trigger the 
            `exit_gracefully` method to handle termination signals.
        """
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
    
    def exit_gracefully(self,signum: int, frame: any):
        """
        Handle termination signals and set the kill flag.

        This method is called when a registered signal (SIGINT or SIGTERM) is received. 
        It sets the `kill_now` attribute to `True`, indicating that the application 
        should terminate.

        Parameters:
            signum: The signal number received.
            frame: The current stack frame (unused in this method).

        Notes:
            This method is intended to be used as a signal handler and is not typically 
            called directly.
        """
        self.kill_now = True
        print('Terminate App')