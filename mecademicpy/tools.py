import logging
import platform
import subprocess
import time
from enum import IntEnum
from typing import Tuple, Union
from xmlrpc.client import Boolean


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


def ping_robot(ip_address: str, timeout: int = 90):
    """Ping the specified IP address, retrying until timeout

    Parameters
    ----------
    ip_address : str
        IP address to ping
    timeout : int, optional
        Maximum time to retry ping if no response, by default 90

    Raises
    ------
    TimeoutError
        Error raised when no reply from specified IP address after specified timeout
    """
    logger = logging.getLogger(__name__)
    logger.info(f"Attempting to ping {ip_address} for {timeout} seconds")
    timeout_time = time.monotonic() + timeout
    while True:
        if _ping(ip_address):
            break
        if time.monotonic() > timeout_time:
            error_msg = f"Timeout while waiting to ping robot: {ip_address}"
            logger.error(error_msg)
            raise TimeoutError(error_msg)

    logger.info(f"Successfully ping {ip_address}")


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
        command_result = subprocess.run(ping_command, capture_output=True, shell=False)
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
