#!/usr/bin/env python3

import sys
import os
import socket
import threading
import time
import queue

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import mecademic.Robot as mdr

TEST_IP = '127.0.0.1'


def fake_server(address, port, data_list, server_up):
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


def run_fake_server(address, port, data_list):
    server_up_event = threading.Event()  # Synchronization event for fake server.
    server_thread = threading.Thread(target=fake_server, args=(address, port, data_list, server_up_event))
    server_thread.start()
    server_up_event.wait()
    return server_thread


class FakeSocket():
    def __init__(self, input):
        self.queue = queue.Queue()
        for x in input:
            self.queue.put(x)

    def setblocking(self, _):
        pass

    def recv(self, _):
        return self.queue.get()


def test_setup_invalid_input():
    with pytest.raises(TypeError):
        mdr.Robot(2)
    with pytest.raises(ValueError):
        mdr.Robot('1.1.1.1.1')


def test_connection_no_robot():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    assert not robot.Connect()


def test_successful_connection_full_socket():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    # Set a longer delay between commands to avoid automatic concatenation by the socket.
    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[3000]\0'])
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert robot.Connect()

    robot.Disconnect()
    assert robot._command_socket is None
    assert robot._monitor_socket is None

    command_server_thread.join()
    monitor_server_thread.join()


def test_successful_connection_split_response():
    fake_socket = FakeSocket([b'[3', b'00', b'0][]\0', b''])
    rx_queue = queue.Queue()

    mdr.Robot._handle_socket_rx(fake_socket, rx_queue)

    assert rx_queue.qsize() == 1
    assert rx_queue.get().id == 3000


def test_sequential_connections():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3001, ''))
    assert not robot.Connect(offline_mode=True)

    robot._command_rx_queue.put(mdr.Message(99999, ''))
    assert not robot.Connect(offline_mode=True)

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)
    robot.Disconnect()


def test_monitoring_connection():
    fake_array = [1, 2, 3, 4, 5, 6]

    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))

    assert robot.Connect(offline_mode=True)

    # Send monitor messages.
    robot._monitor_rx_queue.put(mdr.Message(2026, ','.join(map(str, fake_array))))
    robot._monitor_rx_queue.put('Terminate process')
    # Wait until process ends to ensure the monitor messages are processed.
    robot._monitor_handler_process.join()
    robot._monitor_handler_process = None

    assert robot.GetJoints() == fake_array

    robot.Disconnect()


def test_internal_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    # Validate internal checkpoint waiting.
    checkpoint_1 = robot.SetCheckpoint(1)
    # Check that the command is sent to the robot.
    assert robot._command_tx_queue.get() == 'SetCheckpoint(1)'
    # Check that the id is correct.
    assert checkpoint_1.id == 1
    # Check that wait times out if response has not been sent.
    assert not checkpoint_1.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    # Check that wait succeeds if response is sent.
    assert checkpoint_1.wait()

    robot.Disconnect()


def test_external_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    # Validate external checkpoint waiting.
    checkpoint_1 = robot.ExpectExternalCheckpoint(1)
    # Check that the command is not sent to the robot.
    assert robot._command_tx_queue.qsize() == 0
    # Check that the id is correct.
    assert checkpoint_1.id == 1
    # Check that wait times out if response has not been sent.
    assert not checkpoint_1.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    # Check that wait succeeds if response is sent.
    assert checkpoint_1.wait()

    robot.Disconnect()


def test_multiple_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    # Validate multiple checkpoints, internal and external.
    checkpoint_1 = robot.SetCheckpoint(1)
    checkpoint_2 = robot.SetCheckpoint(2)
    checkpoint_3 = robot.ExpectExternalCheckpoint(3)
    # Check that wait times out if response has not been sent.
    assert not checkpoint_1.wait(timeout=0)
    assert not checkpoint_2.wait(timeout=0)
    assert not checkpoint_3.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    assert not checkpoint_2.wait(timeout=0)
    assert not checkpoint_3.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '2'))
    assert not checkpoint_3.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '3'))
    # Check that waits succeeds if response is sent.
    assert checkpoint_3.wait()
    assert checkpoint_2.wait()
    assert checkpoint_1.wait()

    robot.Disconnect()


def test_repeated_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    # Repeated checkpoints are discouraged, but supported.
    checkpoint_1_a = robot.SetCheckpoint(1)
    checkpoint_1_b = robot.SetCheckpoint(1)
    # Check that wait times out if response has not been sent.
    assert not checkpoint_1_a.wait(timeout=0)
    assert not checkpoint_1_b.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    # Only one checkpoint has been returned, the second should still block.
    assert not checkpoint_1_b.wait(timeout=0)
    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    # Check that waits succeeds if response is sent.
    assert checkpoint_1_b.wait()
    assert checkpoint_1_a.wait()

    robot.Disconnect()


def test_unaccounted_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    # Expect error for unaccounted checkpoints.
    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    robot._command_response_handler_process.join()
    with pytest.raises(AssertionError):
        robot.MoveJoints(0, 0, 0, 0, 0, 0)

    # Necessary so disconnect doesn't fail.
    robot._command_response_handler_process = None
    robot.Disconnect()