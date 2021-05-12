![Mecademic](./docs/logo/mecademic_logo.jpg  "Mecademic")

# Mecademic Python API

A python module designed for Robot products from Mecademic. The module offers tools that give access to the features of the Mecademic Robots such as MoveLin and MoveJoints available through the TCP/IP text interface. The module can be started from a terminal or a python application and controls the Mecademic products. 

#### Supported Robots

 * Meca500 R1, R2, R3

## Prerequisites

Please read the [user programming manual](https://mecademic.com/resources/documentation) to understand concepts necessary for proper usage of the API.

To be able to use the module without unexpected errors, the user must have a copy of python installed on their machine and it is recommended to use python version 3.7 or higher. [Python](https://www.python.org/) can be installed from its main website (a reboot will be require after the installation to complete the setup).

The user can validate their python installation by running `python --version` in a terminal.

This library is compatible with Windows, Linux, and Mac.

## Downloading the package

To install the package through pip, Mecadmic Github repository url must be given. Pip will download and install the package on your machine and place it in the python local packages directory. This is done by running the following command:

```
pip install git+https://github.com/Mecademic/mecademic
``` 

## Quick Start

**Ensure the robot is properly connected to the computer, powered on, and in a nominal state.**

In a python shell or script, import the library. Then initialize an instance of the Robot class by passing the IP Address of the Robot as an argument. Finally, use connect() to establish a connection:

```python
import mecademic.Robot as mdr
robot = mdr.Robot()
assert robot.connect(address='192.168.0.100')
```

The connect function returns true if connection is successful.
This function is synchronous (awaits for success or timeout) even when using the `Robot` class in [asynchronous mode](#synchronous-vs.-asynchronous-mode). 

Before using the robot, it must be activated and homed. To do so, run the following functions:

```python
robot.ActivateRobot()
robot.Home()
```

The robot should move slightly to perform its homing routine. We can also use `robot.WaitHomed()` or [synchronous mode](#synchronous-vs.-asynchronous-mode) to block execution until homing is done. 

Once homing is complete, the robot is now ready to perform operations. [The user programming manual](https://mecademic.com/resources/documentation) or the documentation in the module is sufficient to be able to make the Robot perform actions and control the robot. 

Here is an example of a simple motion to perform:

```python
robot.MoveJoints(0, 0, 0, 0, 0, 0)
robot.MoveJoints(0, -60, 60, 0, 0, 0)
```

When done with the robot, the user should always deactivate and disconnect. Note that deactivating before the motion is complete will cause the motion to immediately stop. The user can wait for motions to complete using `WaitIdle()`.

Deactivating and disconnecting can be done with the following commands:

```python
robot.WaitIdle()
robot.DeactivateRobot()
robot.Disconnect()
```

If the robot encounters an error during operation, the robot will go into an error mode. In this mode, the module will block any command to the robot unless the error is reset. To properly reset errors on the robot, the following function must be run:

```python
robot.ResetError()
```

For complete and working examples, please refer to the `examples` folder.

## Features and Additional Information
### Synchronous vs. Asynchronous Mode

By default the API operates in 'asynchronous mode', which means sending a command to the robot does not block program execution. To illustrate, the following code will be able to successfully print out the changing joint values resulting from the `MoveJoints` command:

```python
import mecademic.Robot as mdr
robot = mdr.Robot()
assert robot.Connect(address='192.168.0.100', enable_synchronous_mode=False)
robot.ActivateAndHome()

robot.MoveJoints(0, 0, 0, 0, 0, 0)
robot.MoveJoints(0, -60, 60, 0, 0, 0)

for _ in range(100):
    print(robot.GetJoints())
    time.sleep(0.05)

robot.WaitIdle()
robot.DeactivateRobot()
robot.Disconnect()
```

However, sometimes it is desired for programs to wait until the previous command is completed before sending the next command. It is generally encouraged to use the [checkpoints](#checkpoints) system or the various `Wait()` functions, but for smaller or simpler programs, the user can set `enable_synchronous_mode=True` to have each command block until the robot has completed the command.

The code block below will only print out the final joint position, since `robot.GetJoints()` doesn't execute until the motion is complete.

```python
import mecademic.Robot as mdr
robot = mdr.Robot()
assert robot.Connect(address='192.168.0.100', enable_synchronous_mode=True)
robot.ActivateAndHome()

robot.MoveJoints(0, 0, 0, 0, 0, 0)
robot.MoveJoints(0, -60, 60, 0, 0, 0)

# The returned robot position will be (0, -60, 60, 0, 0, 0), because this line will only be executed once MoveJoints(0, -60, 60, 0, 0, 0) has completed.
print(robot.GetJoints())

robot.DeactivateRobot()
robot.Disconnect()
```

One disadvantage of using synchronous mode is that blending between motions is not possible, since the next motion is not sent to the robot until the previous motion is complete.

### Disconnect on Exception

By default, if any unrecoverable error occurs during usage of the Robot class, the class will automatically disconnect from the robot to avoid possible issues. Disconnection also causes the robot to pause its motion.

However, disconnecting on exeptions may be undesired when using an interactive terminal or Jupyter notebook, as an accidental mal-formed function call may cause disconnection. As such, this feature can be disabled by setting `disconnect_on_exception=False` when attempting the connection:

```python
robot.Connect(address='192.168.0.100', disconnect_on_exception=False)
```

### Checkpoints

The checkpoint system allows for creating event objects which will be triggered once the robot reaches a specified point in its execution. The `SetCheckpoint(n)` call registers a checkpoint with the robot (with `n` as the ID), and returns an event-type object that can be used to wait for the checkpoint. For example, the following code will wait until both `MoveJoints()` motions have completed, and then print "`The MoveJoints() motions are complete.`":

```python
robot.MoveJoints(0, -60, 60, 0, 0, 0)
robot.MoveJoints(0, 0, 0, 0, 0, 0)
checkpoint_1 = robot.SetCheckpoint(1)

checkpoint_1.wait(timeout=10) # A timeout of 10s is set to avoid infinite wait in case of error.
print('The MoveJoints() motions are complete.')
```

Note that creating multiple checkpoints with the same ID is possible but not recommended. The checkpoints will be triggered in the order they are set.

Checkpoints may also be set in an offline program, saved to robot memory. Use `ExpectExternalCheckpoint(n)` to receive these checkpoints while the robot is running the offline program. The call to `ExpectExternalCheckpoint(n)` should be made before the offline program is started, or otherwise must be guaranteed to occur before the robot can possibly send the checkpoint.

If the robot motion command queue is cleared (using `ClearMotion()` for example), or the robot is disconnected, all pending checkpoints will be aborted, and all active `wait()` calls will raise an `InterruptException`.

### Callbacks

The `Robot` class supports user-provided callback functions on a variety of events. These callbacks are entirely optional and are not required. The available events are listed in the `RobotCallbacks` class. Currently, they are:

- on_connected
- on_disconnected
- on_status_updated
- on_activated
- on_deactivated
- on_homed
- on_error
- on_error_reset
- on_p_stop
- on_p_stop_reset
- on_motion_paused
- on_motion_cleared
- on_motion_resumed
- on_checkpoint_reached
- on_activate_sim
- on_deactivate_sim
- on_command_message
- on_monitor_message
- on_offline_program_state

Note that `on_checkpoint_reached` passes the ID of the checkpoint, and `on_command_message` and `on_monitor_message` passes a `mecademic.Robot.Message` object. All other callbacks do not pass any arguments.

A simple usage example:

```python
import mecademic.Robot as mdr
robot = mdr.Robot()

def print_connected():
    print('Connected!')

callbacks = mdr.RobotCallbacks()
callbacks.on_connected = print_connected

robot.RegisterCallbacks(callbacks=callbacks, run_callbacks_in_separate_thread=True)
robot.Connect(address='192.168.0.100') # Will print 'Connected!' if successful.
```

If the user does not want to automatically run callbacks in a separate thread, set `run_callbacks_in_separate_thread=False` and call `RunCallbacks()` when ready to run all triggered callbacks.

Running any callback in a separate thread (either through the `Robot` class or otherwise) requires that the callback function is thread-safe and uses the proper locks when accessing shared state. Calling any public method of the `Robot` class is thread-safe.

Note that user-provided callback functions will be run in the **same process** as the rest of the `Robot` class. As such, callbacks which require non-trivial computation may interfere with the function of the API, especially when processing many monitoring messages at high frequency.

If non-trivial computation and high-frequency monitoring are both necessary, the user may offload computation into a separate python process using the built-in [multiprocessing](https://docs.python.org/3/library/multiprocessing.html) library.

### Handling Robot Errors

If the robot encounters an error during use, the robot will go into error mode. In this mode, the module will block any command to the robot unless the error is reset. If the robot is in an error state, `GetRobotState().error_status` will return `True`. To properly reset errors on the robot, the following function must be run:

```python
robot.ResetError()
```

It may also be necessary to call `ResumeMotion()`.

The `on_error` callback can also be used to manage robot errors.

Improper use of the class can also cause exceptions to be raised. For example, calling `MoveJoints()` without any arguments will raise an exception. 

If the user is waiting on an event or checkpoint that the `Robot` class later determines will never occur, the event will unblock and raise an exception. For example, if the user is waiting on a checkpoint, but calls `Disconnect()` or `ClearMotion()` before the checkpoint is received, the checkpoint will unblock and raise an exception. Events and checkpoints will also unblock with exception on robot error state.

The user should use python's built-in `try...except` blocks to handle exceptions.

### Preserved State on Disconnection

Once the robot is disconnected, not all state is immediately cleared. Therefore, it is possible to still get the last-known state of the robot. 

### Logging Data to File

It is possible to continuously log the robot state to a file using the API either using the `StartLogging` and `EndLogging` functions or using the `FileLogger` context. 

An example usage of `StartLogging` and `EngLogging`:

```python
robot.WaitIdle()
robot.StartLogging()
try:
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
except BaseException as e:
    print(f'Logging unsuccessful, exception encountered: {e}')
finally:
    robot.EndLogging()
```

Note that the user should wait for the robot to be idle before starting to log, and also wait for idle before ending the log. This is to ensure the log correctly captures the movements of interest.

The user can also use the `FileLogger` context:

```python
robot.WaitIdle()
with robot.FileLogger():
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
```

The `FileLogger` context will automatically end logging after either completing the `with` block or encountering an exception.

The user can select which fields to log using the `fields` parameter in `StartLogging` or `FileLogger`. By default, all available fields are logged. The available fields are currently:

- target_joint_positions 
- target_end_effector_pose 

- target_joint_velocity 
- target_end_effector_velocity 

- target_joint_configurations 
- target_last_joint_turn 

- drive_joint_positions 
- drive_end_effector_pose 

- drive_joint_velocity 
- drive_joint_torque_ratio 
- drive_end_effector_velocity 

- drive_joint_configurations 
- drive_last_joint_turn

The following example only logs the `target_joint_positions` and `target_end_effector_pose`.

```python
robot.WaitIdle()
with robot.FileLogger(fields=['target_joint_positions', 'target_end_effector_pose']):
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
```


## Getting Help

To get support, you can start an issue on the Mecademic/python_driver issues section or send an email to support@mecademic.com.

## License

All packages in this repository are licensed under the MIT license.

## Authors 

* **Mecademic** - *Continuous work*

