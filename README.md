![Mecademic](https://github.com/Mecademic/mecademicpy/blob/main/docs/logo/mecademic_logo.png?raw=true  "Mecademic")
# Mecademic Python API

A python module designed for robot products from Mecademic. The module offers tools that give access to the features of the Mecademic Robots such as MoveLin and MoveJoints available through the TCP/IP text interface. The module can be started from a terminal or a python application and controls the Mecademic products.

#### Supported Robots

 * Meca500, MCS500

#### Supported Firmware Versions

 * 8.3 and up

#### Supported Python versions
Mecademicpy 2.3 targets Python version 3.7 and above.
It has been tested with the following Python versions:
* Python 3.7.17
* Python 3.8.18
* Python 3.9.18
* Python 3.10.13
* Python 3.11.8
* Python 3.12.2
* Python 3.13.2

## Prerequisites

Please read the [user programming manual](https://www.mecademic.com/res/doc/programming-manual) to understand concepts necessary for proper usage of the API. This API implements a subset of the commands in the `Communicating over TCP/IP` section. For the exact list of available commands, use the `help()` command as explained in [API Reference](#api-reference).

To be able to use the module without unexpected errors, the user must have a copy of python installed on their machine and it is required to use python version 3.7 or higher. We recommend using Python 3.9 since this is the version on which this module is actively tested. [Python](https://www.python.org/) can be installed from its main website (a reboot will be require after the installation to complete the setup).

The user can validate their python installation by running `python --version` in a terminal.

This library is compatible with Windows, Linux, and Mac.

## Downloading the package

To download and install the package, the user can easily do so through pip. Pip will download and install the package on your machine and place it in the python local packages directory. This is done by running the following command:

```
pip install mecademicpy
```

## Quick Start

**Ensure the robot is properly connected to the computer, powered on, and in a nominal state.**

In a python shell or script, import the library. Then initialize an instance of the `Robot` class. Finally, use the `Connect` function by passing the IP Address of the robot as an argument to establish a connection:

```python
import mecademicpy.robot as mdr
robot = mdr.Robot()
robot.Connect(address='192.168.0.100')
```

The `Connect` function will raise if connection with robot fails.
This function is synchronous (awaits for success or timeout) even when using the `Robot` class in [asynchronous mode](#synchronous-vs.-asynchronous-mode).

Before using the robot, it must be activated and homed. To do so, run the following functions:

```python
robot.ActivateRobot()
robot.Home()
```

The robot should move slightly to perform its homing routine. We can also use `robot.WaitHomed()` or [synchronous mode](#synchronous-vs.-asynchronous-mode) to block execution until homing is done.

Once homing is complete, the robot is now ready to perform operations. [The user programming manual](https://www.mecademic.com/res/doc/programming-manual) or the documentation in the module is sufficient to be able to make the Robot perform actions and control the robot.

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

It is recommended to use `GetStatusRobot()` to learn about the current robot status before resuming operation.

For complete and working examples, please refer to the `examples` folder.

## Features and Additional Information

### Synchronous vs. Asynchronous Mode

By default the API operates in 'asynchronous mode', which means sending a command to the robot does not block program execution. To illustrate, the following code will be able to successfully print out the changing joint values resulting from the `MoveJoints` command:

```python
import mecademicpy.robot as mdr
import time

robot = mdr.Robot()
robot.Connect(address='192.168.0.100', enable_synchronous_mode=False)
robot.ActivateAndHome()
robot.WaitHomed()

robot.MoveJoints(0, 0, 0, 0, 0, 0)
robot.MoveJoints(0, -60, 60, 0, 0, 0)

# Print robot position while it's moving
for _ in range(100):
    print(robot.GetJoints())
    time.sleep(0.05)

robot.WaitIdle()
robot.DeactivateRobot()
robot.WaitDeactivated()
robot.Disconnect()
```

However, sometimes it is desired for programs to wait until the previous command is completed before sending the next command. It is generally encouraged to use the [checkpoints](#checkpoints) system or the various `Wait()` functions, but for smaller or simpler programs, the user can set `enable_synchronous_mode=True` to have each command block until the robot has completed the command.

The code block below will only print out the final joint position, since `robot.GetJoints()` doesn't execute until the motion is complete.

```python
import mecademicpy.robot as mdr
robot = mdr.Robot()
robot.Connect(address='192.168.0.100', enable_synchronous_mode=True)
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

However, disconnecting on exceptions may be undesired when using an interactive terminal or Jupyter notebook, as an accidental mal-formed function call may cause disconnection. As such, this feature can be disabled by setting `disconnect_on_exception=False` when attempting the connection:

```python
robot.Connect(address='192.168.0.100', disconnect_on_exception=False)
```

### Checkpoints

The checkpoint system allows for creating event objects which will be triggered once the robot reaches a specified point in its execution. The `SetCheckpoint(n)` call registers a checkpoint with the robot (with `n` as the ID), and returns an event-type object that can be used to wait for the checkpoint. This is true for both robot connection type (asynchronous and synchronous mode). For example, the following code will wait until both `MoveJoints()` motions have completed, and then print "`The MoveJoints() motions are complete.`":

```python
robot.MoveJoints(0, -60, 60, 0, 0, 0)
robot.MoveJoints(0, 0, 0, 0, 0, 0)
checkpoint_1 = robot.SetCheckpoint(1)

checkpoint_1.wait(timeout=10) # A timeout of 10s is set to avoid infinite wait in case of error.
print('The MoveJoints() motions are complete.')
```

Note that creating multiple checkpoints with the same ID is possible but not recommended. The checkpoints will be triggered in the order they are set.

Checkpoints may also be set in an offline program, saved to robot memory. Use `ExpectExternalCheckpoint(n)` to receive these checkpoints while the robot is running the offline program. The call to `ExpectExternalCheckpoint(n)` should be made before the offline program is started, or otherwise must be guaranteed to occur before the robot can possibly send the checkpoint. For example, the following code will start an offline program and expect to receive a checkpoint from the program, and then print "`Received expected external checkpoint`":

```python
checkpoint_event = robot.ExpectExternalCheckpoint(5)
robot.StartOfflineProgram(1)
checkpoint_event.wait(timeout=30)
print("Received expected checkpoint from running program")
```
where offline program 1 is
```
MoveJoints(100,0,0,0,0,0)
MoveJoints(-100,0,0,0,0,0)
SetCheckpoint(5)
MoveJoints(0,-60,60,0,0,0)
MoveJoints(0,0,0,0,0,0)
SetOfflineProgramLoop(1)
```

If the robot motion command queue is cleared (using `ClearMotion()` for example), or the robot is disconnected, all pending checkpoints will be aborted, and all active `wait()` calls will raise an `InterruptException`.

### Callbacks

The `Robot` class supports user-provided callback functions on a variety of events. These callbacks are entirely optional and are not required, but useful to implement asynchronous applications. The available events are listed in the `RobotCallbacks` class. Here are some of these callbacks:

- `on_connected`
- `on_disconnected`
- `on_activated`
- `on_deactivated`
- `on_homed`
- `on_error`
- `on_checkpoint_reached`
- etc... (refer to class `RobotCallbacks` for exhaustive list of callbacks)

Note that some callbacks pass arguments. For example `on_checkpoint_reached` passes the ID of the checkpoint, `on_command_message` and `on_monitor_message` passes a `mecademicpy.robot.Message` object. Refer to class documentation for details.

A simple usage example:

```python
import mecademicpy.robot as mdr
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

Note that, due to a Python limitation, all Python threads share the same CPU core and will not take advantage of parallelism and multiple CPU cores of a PC. Unfortunately, this means that an application performing heavy computations (in callback thread or in any other thread) may impact the performance of the `Robot` class (especially when processing many monitoring messages at high frequency).

If non-trivial computation and high-frequency monitoring are both necessary, the user may offload computation into a separate python process using the built-in [multiprocessing](https://docs.python.org/3/library/multiprocessing.html) library.

### Handling Robot Errors

If the robot encounters an error during use, the robot will go into error mode. In this mode, the module will refuse any command to the robot unless the error is reset. If the robot is in an error state, `GetStatusRobot().error_status` will return `True`. To properly reset errors on the robot, the following function must be run:

```python
robot.ResetError()
```

It is recommended to use `GetStatusRobot()` to learn about the current robot status and reset the relevant flags to an appropriate state before resuming operation. For example, an error may require to call `ResumeMotion()`. In this case, verify that `GetStatusRobot().pause_motion_status` is set to `True` before calling `ResumeMotion()`.

The `on_error` callback can also be used to manage robot errors.

Improper use of the class can also cause exceptions to be raised. For example, calling `MoveJoints()` without any arguments will raise an exception.

If the user is waiting on an event or checkpoint that the `Robot` class later determines will never occur, the event will unblock and raise an exception. For example, if the user is waiting on a checkpoint (`WaitCheckpoint`), but calls `Disconnect()` or `ClearMotion()` before the checkpoint is received, the checkpoint will unblock and raise an exception. Events and checkpoints will also unblock with exception on robot error state.

The user should use python's built-in `try...except` blocks to handle appropriate exceptions.

### Handling Robot safety stop conditions
(Note: This section applies to robots running firmware 10.1 and above)

The robot may encounter safety stop conditions based on external safety signals (EStop, PStop1, PStop2, operation mode changed, enabling device released) or other conditions (connection timeout).
It is recommended to use `GetSafetyStatus` to learn about the current safety stop signals status.
The `on_safety_stop`, `on_safety_stop_reset`, `on_safety_stop_resettable` and `on_safety_stop_state_change` callbacks can also be used to manage the safety stop signals status.

Some safety stop signals cause motor voltage to be removed (EStop, PStop1, operation mode change, etc.).
When these safety signals are present, the robot cannot be activated until the safety signals are reset.
For safety reasons, resetting these signals cannot be done programmatically from the Mecademicpy API. They require to press the Reset button on the power supply (or trigger the reset function from dedicated power supply input pins).

Other safety stop signals will cause the robot to pause motion (PStop2, enabling device released, connection dropped).
Once these safety conditions are resolved, the robot may be activated or, if already activated, motion can be resumed using `ResumeMotion()`.

For more information about safety signals, refer to the robot's programming manual.

### Preserved State on Disconnection

Once the robot is disconnected, not all states are immediately cleared. Therefore, it is possible to still get the last-known state of the robot.

### Restore a well-known robot configuration
File robot_initializer.py contains utilities that can be used to configure the robot to a well-known state.
It contains useful methods that can be called when reconnecting to a robot (which state is unknown) or activating a robot in order to restore a well-known state.
See documentation in file robot_initializer.py for more information.

`RobotWithTools`: This is a specialization of the `Robot` class that adds utilities required for using methods in robot_initializer.py.
An application that wants to use the methods from robot_initializer.py shall instantiate `RobotWithTools` instead of `Robot`.

`reset_robot_configuration`: This method restores default robot's static (permanent) configuration parameters, including joint limits, work zone limits, PStop2 configuration etc.

`reset_motion_queue`: This method configures the robot's motion queue with default values (or user-defined values).
It receives, as argument, an instance of class `MotionQueueParams` which lists all available motion queue parameters with their default values.
This method is is useful every time the robot is reactivated (remember that the robot resets its motion queue to default parameters every time it's activated).

`reset_vacuum_module`: This method is used to reset the vacuum module states (digital output states, vacuum parameters and states) to default values.
It can be used at any time since the vacuum module is functional even when the robot is not activated. Vacuum parameters are thus managed separately from other motion queue parameters.

Example
```python
# (use "with" block to ensure proper disconnection at end of block)
with initializer.RobotWithTools() as robot:
    robot.Connect(address='192.168.0.100')

    # Reset the robot's static (permanent) configuration (joint limits, work zone, etc.)
    initializer.reset_robot_configuration(robot)
    
    # Reset the vacuum module states (clear digital outputs, vacuum off)
    initializer.reset_vacuum_module(robot)

    # Activate robot and initialize its motion queue with desired parameters
    mq_params = initializer.MotionQueueParams()
    # Customize some motion queue parameters
    mq_params.joint_vel = 20
    mq_params.trf = [10,20,0,0,45,0]
    initializer.reset_motion_queue(robot, params=mq_params, activate_home=True)

    # Wait until robot is activated and homed
    robot.WaitHomed()
```

### Manage robot variables
Mecademic robots allow you to store persistent variables, which remain valid even after a reboot or power cycle.
These variables are useful in Python programs for a variety of tasks, such as utilizing target positions or reference
frames that have been pre-defined and fine-tuned on each of your robots.

You can create (`CreateVariables`), delete (`DeleteVariables`), read (`robot.vars.variable_name`), 
and modify (`robot.vars.variable_name = new_value`) these variables as needed.

For detailed information on how to use variables, refer to the `Robot` class documentation in the Mecademicpy API.
For further details on robot variables, consult the robot's programming manual.

#### Quick Example:
```python
# Create a variable on the robot
robot.CreateVariable("my_wrf", [0,0,0,0,0,0])
# Modify a variable on the robot
robot.vars.my_wrf = [10,-10,0,0,0,45]
# Use the variable value (cached in Robot class for performance)
print(f'my_wrf={robot.vars.my_wrf}')
robot.DeleteVariable("my_wrf")
```

### Logging Data to File

It is possible to continuously log the robot state to a file using the API either using the `StartLogging` and `EndLogging` functions or using the `FileLogger` context.

An example usage of `StartLogging` and `EngLogging`:

```python
robot.WaitIdle()
robot.StartLogging(0.001)
try:
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
except BaseException as e:
    print(f'Logging unsuccessful, exception encountered: {e}')
finally:
    robot.EndLogging()
```

Note that the user should wait for the robot to be idle (WaitIdle) before starting to log, and also wait for idle before ending the log. This is to ensure the log correctly captures the movements of interest.

It should also be noted that it is mandatory to give a monitoring interval, in seconds, to `StartLogging`, to specify at which rate data should be logged. In the example above, the monitoring interval is set at 0.001 seconds, or 1 ms. It is the minimum monitoring interval that can be set using `SetMonitoringInterval`, which is the robot command used by `StartLogging` to choose the monitoring interval.

The user can also use the `FileLogger` context:

```python
robot.WaitIdle()
with robot.FileLogger(0.001):
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
```

The `FileLogger` context will automatically end logging after either completing the `with` block or encountering an exception.

The user can select which fields to log using the `fields` parameter in `StartLogging` or `FileLogger`. By default, all available fields are logged. The available fields are currently:

- `"TargetJointPos"`
- `"TargetCartPos"`
- `"TargetJointVel"`
- `"TargetCartVel"`
- `"TargetConf"`
- `"TargetConfTurn"`

- `"JointPos"`
- `"CartPos"`
- `"JointVel"`
- `"JointTorq"`
- `"CartVel"`
- `"Conf"`
- `"ConfTurn"`
- `"Accel"`

- `"Wrf"`
- `"Trf"`
- `"Checkpoint"`

- `"ExtToolStatus"`
- `"ValveState"`
- `"GripperState"`
- `"GripperForce"`
- `"GripperPos"`

- `"IoModuleStatus"`
- `"IoModuleOutputState"`
- `"IoModuleInputState"`
- `"SigGenStatus"`
- `"SigGenOutputState"`
- `"SigGenInputState"`
- `"VacuumState"`
- `"VacuumPressure"`

These strings should be placed into the list given to the `fields` parameter.

The following example only logs the `"TargetJointPos"` and `"JointPos"`.

```python
robot.WaitIdle()
with robot.FileLogger(0.001, fields=['TargetJointPos', 'JointPos']):
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
```

Note that the `SetRealTimeMonitoring` command is used by in `StartLogging` or `FileLogger` to enable all the real-time monitoring events which are logged.

### Sending Custom Commands

It is possible to send an arbitrary command to the robot using the `SendCustomCommand()` call. The user can optionally provide expected response codes, which will cause `SendCustomCommand()` to return an event which can be used to wait for the response.

Example usage:
```python
import mecademicpy.robot as mdr
import mecademicpy.mx_robot_def as mdr_def
# Connect, activate, and home robot...

response_codes = [mdr_def.MX_ST_ERROR_RESET, mdr_def.MX_ST_NO_ERROR_RESET]
response = robot.SendCustomCommand('ResetError', expected_responses=response_codes, timeout=10)
```

Although raw numerical response codes can also be used, it is recommended to use the named aliases provided in `mx_robot_def.py` for clarity.

### Waiting for specific robot message
The robot API method `GetInterruptableEvent()` creates a "waitable event". This object contains a "wait" function that will block the Python application execution until the robot sends the expected message (or until timeout).
This event object can optionally get interrupted (throw InterruptException) in case the robot encounters an error or in case motion gets cleared.

Note that the robot API also contains other "wait" methods for various common conditions, like WaitActivated, WaitMotionPaused, WaitIdle, WaitHoldingPart, etc.
Method `GetInterruptableEvent()` is generally used for specific cases that are not covered by other "Wait" methods.

Example 1:
```python
# Create interruptable event that will wait until robot sends code MX_ST_RT_INPUT_STATE (which will happen if a digital input state changes)
input_state_changed_event = robot.GetInterruptableEvent([mdr.MxRobotStatusCode.MX_ST_RT_INPUT_STATE])

# (there can be code here that move the robot or whatever is expected to trigger digital input change)
# (...)

# Wait (block) until the event is received or until timeout (TimeoutException).
input_state_changed_event.wait(10)
```

Example 2:
```python
# Create interruptable event that will trigger when torque limit is exceeded, i.e. event id MX_ST_TORQUE_LIMIT_STATUS with data 1.
# Here we want this event to be interrupted (throw InterruptException) if the robot is in error or motion cleared.
torque_exceeded_event = robot.GetInterruptableEvent(
    codes=[mdr.Message(mdr.MxRobotStatusCode.MX_ST_TORQUE_LIMIT_STATUS, '1')],
    abort_on_error=True,
    abort_on_clear_motion=True)

# (there can be code here that move the robot or do other things, for example)
# (...)

# Check if the torque limit exceeded status event was received
if torque_exceeded_event.is_set():
    # Torque limit was exceeded between call to GetInterruptableEvent above and now
    pass

# Wait (block) until the torque limit exceeded event is received or until timeout (TimeoutException)
torque_exceeded_event.wait(10)

```

### API Reference

For a complete list of available methods and further documentation, use the help() function on any class in a python terminal (such as `ipython`).

```python
>>> import mecademicpy.robot as mdr
>>> help(mdr.Robot)
>>> help(mdr.Message)
>>> help(mdr.RobotCallbacks)
```

## Getting Help



To get support, you can start an issue on the Mecademic github page, issues section or send an email to support@mecademic.com.

## License

All packages in this repository are licensed under the MIT license.

## Authors

* **Mecademic** - *Continuous work*

