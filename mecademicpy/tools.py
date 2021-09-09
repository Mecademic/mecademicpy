import logging
import platform
import subprocess
import time


def ping_robot(ip_address, timeout=90):
    """Attempt to ping robot supp

    :param ip_address: IP address to ping
    :param timeout: time allowed to ping robot in seconds
    :raise TimeoutError if method couldn't reach destination.
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


def _ping(ip_address):
    """Pings the IP address for response

    :param ip_address: IP address to Ping
    :return rep: True if replied, false if no reply
    """
    logger = logging.getLogger(__name__)
    if platform.system() == 'Windows':
        ping_command = ['ping', '-n', '1', ip_address]
    else:
        ping_command = ['ping', '-c', '1', ip_address]
    try:
        proc = subprocess.Popen(ping_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout = proc.stdout.read()
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
