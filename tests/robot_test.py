#!/usr/bin/env python3

import sys
import os
import socket
import threading
import time

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import mecademic.Robot as mdr

TEST_IP = '127.0.0.1'


def fake_server(address, port, data_list, server_up, delay):
    # Run a server to listen for a connection and then close it
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.settimeout(1)  # Allow up to 1 second to create the connection.
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((address, port))
    server_sock.listen()
    server_up.set()

    client, addr = server_sock.accept()

    for data in data_list:
        client.sendall(data.encode('ascii'))
        time.sleep(delay)


def run_fake_server(address, port, data_list, delay=0):
    server_up_event = threading.Event()  # Synchronization event for fake server.
    server_thread = threading.Thread(target=fake_server, args=(address, port, data_list, server_up_event, delay))
    server_thread.start()
    server_up_event.wait()
    return server_thread


def test_setup_invalid_input():
    with pytest.raises(TypeError):
        mdr.Robot(2)
    with pytest.raises(ValueError):
        mdr.Robot('1.1.1.1.1')


def test_connection_no_robot():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    assert not robot.Connect()


def test_successful_connection_split_response():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    # Set a longer delay between commands to avoid automatic concatenation by the socket.
    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[30', '00]\0'], delay=0.5)
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert robot.Connect()

    robot.Disconnect()
    assert robot._Robot__command_socket is None

    command_server_thread.join()
    monitor_server_thread.join()


def test_sequential_connections():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._Robot__command_rx_queue.put('[3001]')
    assert not robot.Connect(offline_mode=True)

    robot._Robot__command_rx_queue.put('[9999]')
    assert not robot.Connect(offline_mode=True)

    robot._Robot__command_rx_queue.put('[3000]')
    assert robot.Connect(offline_mode=True)
    robot.Disconnect()


def test_monitoring_connection():
    fake_array = [1, 2, 3, 4, 5, 6]

    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._Robot__command_rx_queue.put('[3000]')

    assert robot.Connect(offline_mode=True)

    robot._Robot__monitor_rx_queue.put('[2026]' + str(fake_array))

    # Wait until message is consumed, then check proper parsing.
    while not robot._Robot__monitor_rx_queue.empty():
        time.sleep(0.1)
    time.sleep(0.5)

    assert robot.GetJoints() == fake_array

    robot.Disconnect()


def test_checkpoint():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._Robot__command_rx_queue.put('[3000]')
    assert robot.Connect(offline_mode=True)

    robot.SetCheckpoint(2)
    assert not robot.WaitCheckpoint(2, timeout=0.5)
    robot._Robot__command_rx_queue.put('[3030][2]')
    assert robot.WaitCheckpoint(2)

    robot.SetCheckpoint(1)
    robot.SetCheckpoint(1)
    robot._Robot__command_rx_queue.put('[3030][1]')
    # Allow the first checkpoint to be invalidated.
    time.sleep(0.5)
    assert not robot.WaitCheckpoint(1, timeout=0.5)
    robot._Robot__command_rx_queue.put('[3030][1]')
    assert robot.WaitCheckpoint(1)

    assert robot.WaitCheckpoint(1)

    robot.SetCheckpoint(1)
    robot.SetCheckpoint(2)
    robot._Robot__command_rx_queue.put('[3030][1]')
    robot._Robot__command_rx_queue.put('[3030][2]')
    assert robot.WaitCheckpoint(2)
    assert robot.WaitCheckpoint(1)

    robot.Disconnect()