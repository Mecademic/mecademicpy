#!/usr/bin/env python3

import sys
import os
import socket
import threading
import time

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import mecademic.Robot as mdr


def run_fake_server(data, server_up):
    # Run a server to listen for a connection and then close it
    server_sock = socket.socket()
    server_sock.settimeout(1)  # Allow up to 1 second to create the connection.
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('127.0.0.1', 10000))
    server_up.set()

    server_sock.listen()
    client, addr = server_sock.accept()
    client.sendall(data)


def test_setup_invalid_input():
    with pytest.raises(TypeError):
        mdr.Robot(2)
    with pytest.raises(ValueError):
        mdr.Robot("1.1.1.1.1")


def test_connection_no_robot():
    controller = mdr.Robot('127.0.0.1')
    assert controller is not None

    assert not controller.Connect()


def test_successful_connection():
    controller = mdr.Robot('127.0.0.1')
    assert controller is not None

    server_up_event = threading.Event()  # Synchronization event for fake server.
    server_thread = threading.Thread(target=run_fake_server, args=(
        b'[3000] \0',
        server_up_event,
    ))
    server_thread.start()
    server_up_event.wait()

    assert controller.Connect()

    controller.Disconnect()
    assert controller._Robot__socket is None

    server_thread.join()


def test_connection_robot_busy():
    controller = mdr.Robot('127.0.0.1')
    assert controller is not None

    server_up_event = threading.Event()  # Synchronization event for fake server.
    server_thread = threading.Thread(target=run_fake_server, args=(
        b'[3001] \0',
        server_up_event,
    ))
    server_thread.start()
    server_up_event.wait()

    assert not controller.Connect()

    # Test that socket is none if connection fails.
    assert controller._Robot__socket is None

    server_thread.join()


def test_connection_unexpected_return_code():
    controller = mdr.Robot('127.0.0.1')
    assert controller is not None

    server_up_event = threading.Event()  # Synchronization event for fake server.
    server_thread = threading.Thread(target=run_fake_server, args=(
        b'[9999] \0',
        server_up_event,
    ))
    server_thread.start()
    server_up_event.wait()

    assert not controller.Connect()

    # Test that socket is none if connection fails.
    assert controller._Robot__socket is None

    server_thread.join()
