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

    with pytest.raises(Exception):
        robot.Connect()


def test_successful_connection_full_socket():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    command_server_thread = run_fake_server(TEST_IP, mdr.COMMAND_PORT, ['[3000]\0'])
    monitor_server_thread = run_fake_server(TEST_IP, mdr.MONITOR_PORT, [])

    assert not robot.WaitConnected(timeout=0)

    assert robot.Connect()
    assert robot.WaitConnected()

    robot.Disconnect()
    assert robot._command_socket is None
    assert robot._monitor_socket is None

    command_server_thread.join()
    monitor_server_thread.join()


def test_successful_connection_split_response():
    fake_socket = FakeSocket([b'[3', b'00', b'0][test]\0', b''])
    rx_queue = queue.Queue()

    mdr.Robot._handle_socket_rx(fake_socket, rx_queue)

    assert rx_queue.qsize() == 1
    message = rx_queue.get()
    assert message.id == 3000
    assert message.data == 'test'


def test_sequential_connections():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3001, ''))
    with pytest.raises(Exception):
        robot.Connect(offline_mode=True)

    robot._command_rx_queue.put(mdr.Message(99999, ''))
    with pytest.raises(Exception):
        robot.Connect(offline_mode=True)

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)
    robot.Disconnect()


def test_monitoring_connection():
    # Use this as the seed array, add the response code to each element to guarantee uniqueness.
    fake_array = [1, 2, 3, 4, 5, 6, 7]

    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))

    assert robot.Connect(offline_mode=True)

    # Send monitor messages.
    robot._monitor_rx_queue.put(mdr.Message(2026, ','.join([str(x + 2026) for x in fake_array[:-1]])))
    robot._monitor_rx_queue.put(mdr.Message(2027, ','.join([str(x + 2027) for x in fake_array[:-1]])))

    robot._monitor_rx_queue.put(mdr.Message(2200, ','.join([str(x + 2200) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2201, ','.join([str(x + 2201) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2202, ','.join([str(x + 2202) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2204, ','.join([str(x + 2204) for x in fake_array])))

    robot._monitor_rx_queue.put(mdr.Message(2208, ','.join([str(x + 2208) for x in fake_array[:4]])))
    robot._monitor_rx_queue.put(mdr.Message(2209, ','.join([str(x + 2209) for x in fake_array[:2]])))

    robot._monitor_rx_queue.put(mdr.Message(2210, ','.join([str(x + 2210) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2211, ','.join([str(x + 2211) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2212, ','.join([str(x + 2212) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2213, ','.join([str(x + 2213) for x in fake_array])))
    robot._monitor_rx_queue.put(mdr.Message(2214, ','.join([str(x + 2214) for x in fake_array])))

    robot._monitor_rx_queue.put(mdr.Message(2218, ','.join([str(x + 2218) for x in fake_array[:4]])))
    robot._monitor_rx_queue.put(mdr.Message(2219, ','.join([str(x + 2219) for x in fake_array[:2]])))

    robot._monitor_rx_queue.put(mdr.Message(2220, ','.join([str(x + 2220) for x in fake_array[:5]])))

    robot._monitor_rx_queue.put('Terminate process')
    # Wait until process ends to ensure the monitor messages are processed.
    robot._monitor_handler_process.join()
    robot._monitor_handler_process = None

    assert robot.GetJoints() == [x + 2026 for x in fake_array[:-1]]
    assert robot.GetEndEffectorPose() == [x + 2027 for x in fake_array[:-1]]

    # Temporarily test using direct members, switch to using proper getters once implemented.
    assert robot._robot_state.nc_joint_positions[:] == [x + 2200 for x in fake_array]
    assert robot._robot_state.nc_end_effector_pose[:] == [x + 2201 for x in fake_array]
    assert robot._robot_state.nc_joint_velocity[:] == [x + 2202 for x in fake_array]
    assert robot._robot_state.nc_end_effector_velocity[:] == [x + 2204 for x in fake_array]

    assert robot._robot_state.nc_joint_configurations[:] == [x + 2208 for x in fake_array[:4]]
    assert robot._robot_state.nc_multiturn[:] == [x + 2209 for x in fake_array[:2]]

    assert robot._robot_state.drive_joint_positions[:] == [x + 2210 for x in fake_array]
    assert robot._robot_state.drive_end_effector_pose[:] == [x + 2211 for x in fake_array]
    assert robot._robot_state.drive_joint_velocity[:] == [x + 2212 for x in fake_array]
    assert robot._robot_state.drive_joint_torque_ratio[:] == [x + 2213 for x in fake_array]
    assert robot._robot_state.drive_end_effector_velocity[:] == [x + 2214 for x in fake_array]

    assert robot._robot_state.drive_joint_configurations[:] == [x + 2218 for x in fake_array[:4]]
    assert robot._robot_state.drive_multiturn[:] == [x + 2219 for x in fake_array[:2]]

    assert robot._robot_state.accelerometer[:] == [x + 2220 for x in fake_array[:5]]

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


def test_special_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    checkpoint_1 = robot.SetCheckpoint(1)
    checkpoint_2 = robot.SetCheckpoint(2)

    assert not robot.WaitForAnyCheckpoint(timeout=0)

    robot._command_rx_queue.put(mdr.Message(3030, '1'))
    assert robot.WaitForAnyCheckpoint()

    robot.Disconnect()


def test_unaccounted_checkpoints():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    # Send unexpected checkpoint.
    robot._command_rx_queue.put(mdr.Message(3030, '1'))

    assert robot._check_monitor_processes()

    robot.Disconnect()


def test_events():
    robot = mdr.Robot(TEST_IP)
    assert robot is not None

    assert not robot.WaitActivated(timeout=0)
    assert robot.WaitDeactivated()
    assert not robot.WaitConnected(timeout=0)
    assert robot.WaitDisconnected()

    robot._command_rx_queue.put(mdr.Message(3000, ''))
    assert robot.Connect(offline_mode=True)

    assert robot.WaitConnected()
    assert not robot.WaitDisconnected(timeout=0)

    assert not robot.WaitActivated(timeout=0)
    assert robot.WaitDeactivated()

    robot.ActivateRobot()
    robot._monitor_rx_queue.put(mdr.Message(2007, '1,0,0,0,0,0,0'))

    assert robot.WaitActivated(timeout=1)
    assert not robot.WaitDeactivated(timeout=0)

    assert not robot.WaitHomed(timeout=0)
    robot.Home()
    robot._monitor_rx_queue.put(mdr.Message(2007, '1,1,0,0,0,0,0'))
    assert robot.WaitHomed(timeout=1)

    robot.PauseMotion()
    robot._monitor_rx_queue.put(mdr.Message(2007, '1,1,0,0,1,0,0'))
    # Wait until pause is successfully set.
    robot._events.OnRobotMotionPaused.wait()

    assert not robot.WaitMotionResumed(timeout=0)
    robot.ResumeMotion()
    robot._monitor_rx_queue.put(mdr.Message(2007, '1,1,0,0,0,0,0'))
    assert robot.WaitMotionResumed(timeout=1)

    assert not robot.WaitMotionCleared(timeout=0)
    robot.ClearMotion()
    robot._command_rx_queue.put(mdr.Message(2044, ''))
    assert robot.WaitMotionCleared(timeout=1)

    robot.DeactivateRobot()
    robot._monitor_rx_queue.put(mdr.Message(2007, '0,0,0,0,0,0,0'))

    # Note: the order of these waits is intentional.
    # The WaitActivated check may fail if message hasn't yet been processed.
    assert robot.WaitDeactivated(timeout=1)
    assert not robot.WaitActivated(timeout=0)

    assert not robot.WaitDisconnected(timeout=0)
    robot.Disconnect()
    assert robot.WaitDisconnected()
