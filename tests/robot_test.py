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
    controller = mdr.Robot(TEST_IP)
    assert controller is not None

    assert not controller.Connect()


def test_successful_connection():
    controller = mdr.Robot(TEST_IP)
    assert controller is not None

    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[3000]\0'])
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert controller.Connect()

    controller.Disconnect()
    assert controller._Robot__command_socket is None

    command_server_thread.join()
    monitor_server_thread.join()


def test_successful_connection_split_response():
    controller = mdr.Robot(TEST_IP)
    assert controller is not None

    # Set a longer delay between commands to avoid automatic concatenation by the socket.
    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[30', '00]\0'], delay=0.5)
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert controller.Connect()

    controller.Disconnect()
    assert controller._Robot__command_socket is None

    command_server_thread.join()
    monitor_server_thread.join()


def test_connection_robot_busy():
    controller = mdr.Robot(TEST_IP)
    assert controller is not None

    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[3001]\0'])
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert not controller.Connect()

    # Test that socket is none if connection fails.
    assert controller._Robot__command_socket is None

    command_server_thread.join()
    monitor_server_thread.join()


def test_connection_unexpected_return_code():
    controller = mdr.Robot(TEST_IP)
    assert controller is not None

    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[9999]\0'])
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert not controller.Connect()

    # Test that socket is none if connection fails.
    assert controller._Robot__command_socket is None

    command_server_thread.join()
    monitor_server_thread.join()