"""
This file contains various tools and functions useful when using mecademicpy package.
"""
from __future__ import annotations

import inspect
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
from typing import Callable, Optional, Tuple, Type, Union

# pylint: disable=wildcard-import,unused-wildcard-import
from .mx_robot_def import *

try:
    # pylint: disable=unused-import
    import deprecation
except ImportError:
    # Define a no-op decorator with the same signature
    # pylint: disable=invalid-name
    class deprecation:

        @staticmethod
        # pylint: disable=unused-argument
        def deprecated(deprecated_in=None, removed_in=None, current_version=None, details=""):

            def decorator(func):
                return func

            return decorator


# importlib.metadata is new from v3.8
if sys.version_info < (3, 8):
    # From version lower than 3.8, use module
    from importlib_metadata import version
else:
    # Changed in version 3.10: importlib.metadata is no longer provisional.
    from importlib.metadata import version

try:
    __version__ = version('mecademicpy')
# pylint: disable=broad-exception-caught
except Exception:
    __version__ = '0.0'
try:
    from colorama import Fore, Style

    # pylint: disable=invalid-name
    COLOR_RESET = Style.RESET_ALL
    COLOR_RED = Fore.RED
    COLOR_YELLOW = Fore.YELLOW
except ImportError:
    # Define empty strings if colorama is not installed
    # pylint: disable=invalid-name
    COLOR_RESET = "\033[39m"
    COLOR_RED = ""
    COLOR_YELLOW = ""


def get_mecademicpy_version() -> str:
    """Return the current mecademicpy version as a string """
    return __version__


class MxTraceFormatter(logging.Formatter):
    """Enhanced and personalized default log formatter"""
    ANSI_ESCAPE_PATTERN = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')

    def __init__(self, keep_colors=True):
        super().__init__()
        self.keep_colors = keep_colors

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
        trace = f"{prefix}: {record.msg}{COLOR_RESET}"
        if not self.keep_colors:
            trace = self.ANSI_ESCAPE_PATTERN.sub('', trace)
        return trace


#pylint: disable=invalid-name
def SetDefaultLogger(console_level=logging.INFO, filename: str = "", file_level=logging.INFO):
    """Utility function that prepares a default console logger and optionally a file logger

    Parameters
    ----------
    console_level
        Logging level to use on the console, by default logging.INFO
    filename
        Log file name (path), by default ""
    file_level
        Logging level to use in the file (if filename is not empty), by default logging.INFO
    """
    handlers: list[logging.StreamHandler | logging.FileHandler] = []
    formatter = MxTraceFormatter(keep_colors=True)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    handlers.append(console_handler)

    if filename != "":
        log_file_formatter = MxTraceFormatter(keep_colors=False)
        file_handler = logging.FileHandler(filename=filename)
        file_handler.setFormatter(log_file_formatter)
        file_handler.setLevel(file_level)
        handlers.append(file_handler)

    logging.basicConfig(level=console_level, handlers=handlers)


def shorten_stack_trace_file_path(trace_line: str) -> Tuple[str, str]:
    """Replaces the file path in a call trace line with the filename, removing quotes.

    Parameters
    ----------
    trace_line
        The call trace line string.

    Returns
    -------
    Tuple
        - The call trace line with the file path replaced by the filename, without quotes.
        - The file path only
  """
    # Find the index of "File "
    file_index = trace_line.lower().find("file ")
    if file_index == -1:
        return trace_line, ''

    # Extract the path portion with quotes
    match = re.search(r'file "(.*?)",', trace_line, re.IGNORECASE)
    if not match:
        return trace_line, ''

    path = match.group(1)  # Extract the captured path without quotes
    filename = os.path.basename(path)

    # Replace the path with the filename
    shortened_trace = trace_line.replace(match.group(0), f"File {filename},")
    return (shortened_trace, path)


def format_simplified_tb(tb: traceback.TracebackType) -> list[str]:
    """Returns a simplified call stack as a list of strings.
    It's simplified by showing the file/line and the called function, for example:
    File my_program.mxpy, line 14 in move_my_robot(), calling WaitIdle()

    Parameters
    ----------
    tb
        The trace back to format (from exception.__traceback__ for example)

    Returns
    -------
    list[str]
        List with strings, each representing one call stack item
    """
    extracted = traceback.extract_tb(tb)
    lines = []
    for i, frame in enumerate(extracted):
        fname = frame.name
        next_name = extracted[i + 1].name if i + 1 < len(extracted) else None
        file_name = frame.filename
        if file_name == '<string>':
            file_name = 'Executed code'
        else:
            file_name = f'File "{file_name}"'
        if fname == '<module>':
            text = f'    -> {file_name}, line {frame.lineno}'
        else:
            text = f'    -> {file_name}, line {frame.lineno} in {fname}()'
        if next_name:
            text += f", calling {next_name}()"
        lines.append(text)
    return lines


def format_stack(exception: Optional[Exception] = None,
                 from_str: Optional[str] = None,
                 included=False,
                 format_callback: Optional[Callable[[str], str]] = None,
                 stack_paths: Optional[list[str]] = None) -> list[str]:
    """Format the part of the call stack starting from the specified file name
    (i.e. avoid showing all parent functions in the call stack while we're interested in the problem within child code)

    Parameters
    ----------
    exception
        Exception to format partial call stack for. The current call stack is used if None.
    from_str
        File path or string to search in the call stack. The stack will be formatted starting from this file/string.
        The whole stack is formatted if None.
    included
        Include from_str in the trace (else start with next).
    format_callback
        Callback to customize the formatted call stack, called for each formatted line.
    stack_paths
        List that this function will fill with the path of the file that corresponds to each item of the
        returned call stack list.

    Returns
    -------
    list[str]
        The formatted call stack of the provided exception (or current call stack), starting from
        from_str (or just after) and containing only user code (not mecademicpy or mecascript functions).
    """
    # Get the call stack from the exception
    if exception:
        # Get the exception call stack
        stack = format_simplified_tb(exception.__traceback__)
    else:
        # Get the current call stack
        stack = traceback.format_stack()

    trimmed_call_stack: list[str] = []
    found_file = False
    found_next_file = False
    if from_str:
        # Start formatting the call stack from the specified file
        from_str = pathlib.Path(from_str).stem

    # Start capturing the call stack from the provided "from_str" if appropriate
    for item in stack:
        append = False
        if from_str is None:
            append = True
        else:
            if from_str in item:
                found_file = True
                found_next_file = False

            if found_next_file or (included and found_file):
                append = True
            found_next_file |= found_file

        if append:
            formatted_line, path = shorten_stack_trace_file_path(item)
            if format_callback is not None:
                formatted_line = format_callback(formatted_line)
            trimmed_call_stack.append(formatted_line.rstrip())
            if stack_paths is not None:
                stack_paths.append(path)

    return trimmed_call_stack


def get_parent_fct_name() -> str:
    """Returns the name of the calling function

    Returns
    -------
    str
        The name of the calling function
    """
    # First
    frame = inspect.currentframe()
    try:
        # Step 1: skip ourself (get_parent_fct_name)
        caller = frame.f_back
        if not caller:
            return "<unknown>"

        # Step 2: Get caller function name
        parent_name = caller.f_code.co_name
        caller = caller.f_back

        # Step 3: skip caller, and caller's caller if same function name (e.g., base/derived call chain)
        while caller and caller.f_code.co_name == parent_name:
            caller = caller.f_back

        # Step 4: return parent name if found
        return caller.f_code.co_name if caller else parent_name
    finally:
        del frame


def string_to_numbers(input_string: str) -> list:
    """Convert comma-separated floats in string form to relevant type.

    Parameters
    ----------
    input_string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list
        Returns converted list of floats or int, depending on nature of element in 'input_string'.
"""

    return [(float(x) if ('.' in x or 'nan' in x.lower()) else int(x)) for x in input_string.split(',')]


def args_to_string(args: Union[str, list, tuple]) -> str:
    """Convert a list of arguments into a string, taking care of converting IntEnum arguments into their
       numeric value rather than their text value

    Parameters
    ----------
    args
        List of arguments (int, float, string, IntEnum...)

    Returns
    -------
    str
        The arguments formatted, ready to be sent to the robot
    """
    if isinstance(args, str):
        return args
    str_arglist = []
    for arg in args:
        if isinstance(arg, IntEnum):
            str_arglist.append(str(arg.value))
        else:
            str_arglist.append(str(arg))
    return ','.join([x for x in str_arglist])


def safe_enum_cast(enum_cls: Type[IntEnum], value: int) -> Union[IntEnum, int]:
    """Cast value to enum_cls if possible, otherwise return the original integer."""
    try:
        return enum_cls(value)
    except ValueError:
        return value


def ping_robot(ip_address: str, timeout: int = 90, count: int = 1):
    """Ping the specified IP address, retrying until timeout

    Parameters
    ----------
    ip_address
        IP address to ping
    timeout
        Maximum time to retry ping if no response, by default 90
    count
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
    ip_address
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

    Parameters
    ----------
    address
        Remote address (IP or host name) to connect to
    port
        Remote port to connect to
    timeout
        Maximum time retrying the connection.

    Raises
    ------
    TimeoutError
        Timeout trying to establish the connection
    ConnectionError
        Failure to establish the connection for reason other than a timeout (generally means that remote side
        is explicitly refusing the connection)

    Returns
    -------
    socket.socket
        The connected socket
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

    Parameters
    ----------
    timeout
        Timeout to wait for
    condition_callback
        Callback function that will receive the elapsed time and return True to interrupt this sleep.
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


def robot_model_support_vacuum(robot_model: MxRobotModel):
    """Tells if the specified robot model support the MVK01 vacuum module"""

    return robot_model_is_mg2(robot_model)


def robot_model_support_mecascript(robot_model: MxRobotModel):
    """Tells if the specified robot model support the Mecascript sidecar engine"""
    return robot_model_is_mg2(robot_model)


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
