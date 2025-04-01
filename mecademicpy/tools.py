"""
This file contains various tools and functions useful when using mecademicpy package.
"""
from __future__ import annotations

import logging
import os
import pathlib
import platform
import re
import socket
import subprocess
import sys
import time
import traceback
from datetime import datetime
from enum import IntEnum
from typing import Callable, Optional

try:
    from colorama import Fore, Style
    COLOR_RED = Fore.RED
    COLOR_YELLOW = Fore.YELLOW
    COLOR_RESET = Style.RESET_ALL
except ImportError:
    # Define empty strings if colorama is not installed
    COLOR_RED = ""
    COLOR_YELLOW = ""
    COLOR_RESET = ""

# pylint: disable=wildcard-import,unused-wildcard-import
from .mx_robot_def import *


class MxTraceFormatter(logging.Formatter):

    def format(self, record):
        try:
            timestamp = datetime.fromtimestamp(record.created).strftime('%Y-%m-%d %H:%M:%S')
        # pylint: disable=broad-exception-caught
        except Exception:
            # Note: This happen when quitting
            timestamp = ""
        prefix = f"{timestamp} [{record.levelname}]"
        if record.levelno >= logging.ERROR:
            prefix = f"{COLOR_RED}{prefix}"
        elif record.levelno >= logging.WARNING:
            prefix = f"{COLOR_YELLOW}{prefix}"
        return f"{prefix}: {record.msg}{COLOR_RESET}"


#pylint: disable=invalid-name
def SetDefaultLogger(console_level=logging.INFO, filename: str = "", file_level=logging.INFO):
    """Utility function that prepares a default console logger and optionally a file logger

    Parameters
    ----------
    console_level : optional
        Logging level to use on the console, by default logging.INFO
    filename : str, optional
        Log file name (path), by default ""
    file_level : optional
        Logging level to use in the file (if filename is not empty), by default logging.INFO
    """
    handlers: list[logging.StreamHandler | logging.FileHandler] = []
    formatter = MxTraceFormatter()
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    handlers.append(console_handler)

    if filename != "":
        file_handler = logging.FileHandler(filename=filename)
        file_handler.setFormatter(formatter)
        file_handler.setLevel(file_level)
        handlers.append(file_handler)

    logging.basicConfig(level=console_level, handlers=handlers)


def shorten_stack_trace_file_path(trace_line):
    """Replaces the file path in a call trace line with the filename, removing quotes.

  Args:
    trace_line: The call trace line string.

  Returns:
    The call trace line with the file path replaced by the filename, without quotes.
  """

    # Find the index of "File "
    file_index = trace_line.find("File ")
    if file_index == -1:
        return trace_line

    # Extract the path portion with quotes
    match = re.search(r'File "(.*?)",', trace_line)
    if not match:
        return trace_line

    path = match.group(1)  # Extract the captured path without quotes
    filename = os.path.basename(path)

    # Replace the path with the filename
    return trace_line.replace(match.group(0), f"File {filename},")


def format_stack(exception: Optional[Exception] = None,
                 from_str: Optional[str] = None,
                 included=False,
                 format_callback: Optional[Callable[[str], str]] = None) -> str:
    """Format the part of the call stack starting from the specified file name
    (i.e. avoid showing all parent functions in the call stack while we're interested in the problem within child code)

    Args:
        exception (Optional[Exception]): Exception to format partial call stack for.
                                         The current call stack is used if None.
        from_str (Optional[str]):   File path or string to search in the call stack.
                                    The stack will be formatted starting from this file/string.
                                    The whole stack is formatted if None.
        included (bool, optional): Include from_str in the trace (else start with next). Defaults to False.
        format_callback (Callable): Callback to customize the formatted call stack, called for each formatted line.

    Returns:
        str: A string that contains the call stack of the provided exception (or current call stack),
             starting from from_str (or just after) if necessary.
    """
    # Get the call stack from the exception
    if exception:
        # Get the exception call stack
        stack = traceback.format_tb(exception.__traceback__)
    else:
        # Get the current call stack
        stack = traceback.format_stack()

    trimmed_called_stack: list[str] = []
    found_file = False
    found_next_file = False
    skip_last = False
    if from_str:
        # Start formatting the call stack from the specified file
        from_str = pathlib.Path(from_str).stem
    else:
        # Start formatting the call stack from the main python file (avoid system stuff that precedes)
        from_str = pathlib.Path(sys.argv[0]).stem
        if exception is None:
            # Also skip the last call stack item (this function)
            skip_last = True

    for item in stack:
        if from_str in item:
            found_file = True
            found_next_file = False

        if found_next_file or (included and found_file):
            formatted_line = shorten_stack_trace_file_path(item)
            if format_callback is not None:
                formatted_line = format_callback(formatted_line)
            trimmed_called_stack.append(formatted_line)
        found_next_file |= found_file

    if skip_last:
        trimmed_called_stack = trimmed_called_stack[:-1]

    # Return the stack as a string
    return ''.join(trimmed_called_stack)


def string_to_numbers(input_string: str) -> list:
    """Convert comma-separated floats in string form to relevant type.

    Parameters
    ----------
    input_string : string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list of numbers
        Returns converted list of floats or integers, depending on nature of element in 'input_string'.
"""

    return [(float(x) if ('.' in x or 'nan' in x.lower()) else int(x)) for x in input_string.split(',')]


def args_to_string(arg_list: list) -> str:
    """Convert a list of arguments into a string, taking care of converting IntEnum arguments into their
       numeric value rather than their text value

    Parameters
    ----------
    arg_list : list
        List of arguments (int, float, string, IntEnum...)

    Returns
    -------
    str
        The arguments formatted, ready to be sent to the robot
    """
    str_arglist = []
    for arg in arg_list:
        if isinstance(arg, IntEnum):
            str_arglist.append(str(arg.value))
        else:
            str_arglist.append(str(arg))
    return ','.join([x for x in str_arglist])


def ping_robot(ip_address: str, timeout: int = 90, count: int = 1):
    """Ping the specified IP address, retrying until timeout

    Parameters
    ----------
    ip_address : str
        IP address to ping
    timeout : int, optional
        Maximum time to retry ping if no response, by default 90
    count : int, optional
        Number of successful ping to perform before returning

    Raises
    ------
    TimeoutError
        Error raised when no reply from specified IP address after specified timeout
    """
    logger = logging.getLogger(__name__)
    logger.info(f'Attempting to ping {ip_address} for {timeout} seconds')
    timeout_time = time.monotonic() + timeout
    ping_success = 0
    while True:
        if _ping(ip_address):
            ping_success += 1
            logger.info(f'Ping {ip_address} successfully')
            if ping_success >= count:
                break
        if time.monotonic() > timeout_time:
            error_msg = f'Timeout while waiting to ping robot: {ip_address}'
            logger.error(error_msg)
            raise TimeoutError(error_msg)

    logger.info(f'Successfully ping {ip_address}')


def _ping(ip_address: str) -> bool:
    """Ping the specified IP address (one attempt only)

    Parameters
    ----------
    ip_address : str
        IP address to ping

    Returns
    -------
    bool
        True if got successful ping reply, False otherwise
    """
    logger = logging.getLogger(__name__)
    if platform.system() == 'Windows':
        ping_command = ['ping', '-n', '1', ip_address]
    else:
        ping_command = ['ping', '-c', '1', ip_address]
    try:
        command_result = subprocess.run(ping_command, capture_output=True, shell=False, check=False)
        stdout = command_result.stdout.decode("utf-8")
    except subprocess.CalledProcessError as exc:
        logger.info(f"Error running command:'{ping_command}',  error: '{exc}'")
        return False
    if stdout:
        stdout = str(stdout).replace("\\n", "\n\t").replace("\\t", "\t")
        logger.debug(stdout)
        success_ping = f"Reply from {ip_address}: bytes="
        if stdout.find(success_ping) != -1:
            return True
    return False


def socket_connect_loop(address: str, port: int, timeout: float = 1.0) -> socket.socket:
    """Function that retry connecting a socket to remote IP/Port until it succeeds or until timeout

    Args:
        address (str): Remote address (IP or host name) to connect to
        port (int): Remote port to connect to
        timeout (float, optional): Maximum time retrying the connection. Defaults to 1.0.

    Raises:
        TimeoutError: Timeout trying to establish the connection
        ConnectionError: Failure to establish the connection for reason other than a timeout
                         (generally means that remote side is explicitly refusing the connection)

    Returns:
        socket.socket: The connected socket
    """
    start_time = time.monotonic()
    loop_timeout = 0.1
    success = False
    try:
        while True:
            # Create socket and attempt connection.
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.settimeout(loop_timeout)
            try:
                new_socket.connect((address, port))
                success = True
                return new_socket
            except (socket.timeout, TimeoutError) as exception:
                if (time.monotonic() - start_time) < timeout:
                    new_socket.close()
                    continue
                else:
                    raise TimeoutError(f'Unable to connect to {address}:{port} after {timeout}s, exception: {exception}'
                                       ) from exception
            except (ConnectionAbortedError, ConnectionRefusedError, ConnectionResetError) as exception:
                if (time.monotonic() - start_time) < timeout:
                    time.sleep(loop_timeout)
                else:
                    error_message = f'Unable to connect to {address}:{port}, exception: {exception}'
                    raise ConnectionError(error_message) from exception
    finally:
        if not success:
            new_socket.close()


def interruptable_sleep(timeout: float, condition_callback: Callable[[float], bool] = None):
    """Similar to time.sleep but can be interrupted

    Args:
        timeout (float): Timeout to wait for
        condition_callback (Callable[[float], bool], optional): Callback function that will receive the elapsed time
            and return True to interrupt this sleep. Defaults to None.
    """
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        if elapsed > timeout:
            break
        if condition_callback is not None:
            stop = condition_callback(elapsed)
            if stop:
                break
        time.sleep(0.1)  # Sleep in short intervals to allow interruption


def robot_operation_mode_to_string(robot_operation_mode: MxRobotOperationMode) -> str:
    """Returns a human-readable string that represents the specified robot operation mode"""
    if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_LOCKED:
        return "Locked"
    if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_AUTO:
        return "Automatic"
    if robot_operation_mode == MxRobotOperationMode.MX_ROBOT_OPERATION_MODE_MANUAL:
        return "Manual"
    return "Invalid"


def robot_model_is_meca500(robot_model: MxRobotModel):
    """Tells if the specified robot model is a Meca500 robot (any revision)"""
    return (robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R1 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R2
            or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R3 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R4)


def robot_model_is_mcs500(robot_model: MxRobotModel):
    """Tells if the specified robot model is a Mcs500 robot (any revision)"""
    return robot_model == MxRobotModel.MX_ROBOT_MODEL_MCS500_R1


def robot_model_is_mg2(robot_model: MxRobotModel):
    """Tells if the specified robot model is a Mecademic 2nd generation robot (Mcs500, ...)"""
    return (robot_model == MxRobotModel.MX_ROBOT_MODEL_MCA1000_R1
            or robot_model == MxRobotModel.MX_ROBOT_MODEL_MCA250_R1
            or robot_model == MxRobotModel.MX_ROBOT_MODEL_MCS500_R1)


def robot_model_support_eoat(robot_model: MxRobotModel):
    """Tells if the specified robot model support eoat (Meca500, ...)"""

    return (robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R1 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R2
            or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R3 or robot_model == MxRobotModel.MX_ROBOT_MODEL_M500_R4)


def robot_collision_group_robot_idx_to_string(robot_idx: MxCollisionGroupRobotIdx) -> str:
    """Returns a human-readable string that represents the collision index within group 'robot' """
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_BASE:
        return 'Base'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_1:
        return 'Link-1'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_2:
        return 'Link-2'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_3:
        return 'Link-3'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_4:
        return 'Link-4'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_5:
        return 'Link-5'
    if robot_idx == MxCollisionGroupRobotIdx.MX_COLLISION_GROUP_ROBOT_LINK_6:
        return 'Link-6'
    return 'Invalid'


def robot_collision_group_tool_idx_to_string(tool_idx: MxCollisionGroupToolIdx) -> str:
    """Returns a human-readable string that represents the collision index within group 'tool' """
    if tool_idx == MxCollisionGroupToolIdx.MX_COLLISION_GROUP_TOOL_SPHERE:
        return 'Tool Sphere'
    if tool_idx == MxCollisionGroupToolIdx.MX_COLLISION_GROUP_TOOL_MPM500:
        return 'MPM500'
    if tool_idx == MxCollisionGroupToolIdx.MX_COLLISION_GROUP_TOOL_MVK01:
        return 'MVK01'
    return 'Invalid'


def robot_collision_group_to_string(collision_group: MxCollisionGroup, index: Optional[int] = 0) -> str:
    """Returns a human-readable string that represents the collision group"""
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_ROBOT:
        if index is not None:
            return f'{robot_collision_group_robot_idx_to_string(index)}'
        else:
            return 'Robot'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_FCP:
        return 'Flange center point'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_TOOL:
        if index is not None:
            return f'{robot_collision_group_tool_idx_to_string(index)}'
        else:
            return 'Tool'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_ENV_OBJ:
        return f'Object #{0 if index is None else index}'
    if collision_group == MxCollisionGroup.MX_COLLISION_GROUP_WORK_ZONE:
        return 'Work Zone'
    return 'Invalid'
