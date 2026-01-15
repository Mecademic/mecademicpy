![Mecademic](docs/logo/mecademic_logo.png "Mecademic")

# Mecademic Python API

## Introduction

The `mecademicpy` Python library is the official API provided by Mecademic for controlling its
high-precision robotic arms.
It enables seamless integration of Mecademic robots into Python applications by providing a Python interface
to the robot’s text-based TCP/IP command protocol.

This package can be installed using pip:

```bash
pip install mecademicpy
```

You can also download source code from
[MecademicPy GitHub page](https://github.com/Mecademic/mecademicpy).

## Key features

### Easy-to-use API

Although the `mecademicpy` API can unleash the full flexibility and power of the Python language to write robot
programs, writing a simple robot program remains easy, even for users not very familiar with the Python
language.

### Mirrors the robot's native text API

The `mecademicpy` library uses the same function names and arguments as the robot’s native text-based (TCP/IP) API,
for consistency and ease of use. Therefore, any user already familiar with Mecademic's TCP/IP text API will
quickly be able to write Python programs using the `mecademicpy` library.

### Control robot motion

Control robot motion using intuitive functions such as MoveLin, MoveJoints, GetStatusRobot, WaitIdle and more.

### Robot management

Initialize and control the robot’s operational state using functions such as
ActivateRobot, Home, WaitIdle, DeactivateRobot, UpdateRobot, and others.

The `mecademicpy` library also provides utilities to
[restore a well-known robot configuration](#restore-a-well-known-robot-configuration),
making it easy to ensure the robot is in the expected state before a program starts using it.

### Robot configuration

Set persistent configuration parameters with functions such as
SetJointLimits, SetWorkZoneLimits, SetPStop2Cfg, and more.

### Robot program management

Manage and execute on-robot programs using functions such as
ListFiles, SaveFile, LoadFile, DeleteFile, and StartProgram.

### Variable management

[Manage robot variables](#manage-robot-variables) that can be used across all robot APIs
(TCP/IP, cyclic protocols, or Python),
using functions such as GetVariable, SetVariable, CreateVariable, DeleteVariable, and ListVariables.
Variables are also automatically exposed via the robot.variables interface as easy-to-use, read/write Python attributes
(e.g., robot.variables.myVar = [...]).

### Synchronous and asynchronous modes

Send commands asynchronously to allow concurrent execution while the robot moves. It also allows blending
between consecutive move commands.

Alternatively, you can enable synchronous mode so that each command blocks until the robot completes the
requested operation.

For more details, refer to [Asynchronous mode considerations](#asynchronous-mode-considerations).

### Cross-platform compatibility

Supports Windows, Linux, and macOS environments through Python.

### Broad robot and firmware compatibility

- Supports all Mecademic robot models;
- Compatible with robot firmware version 8.3 and newer;
- Works with Python versions 3.8 through 3.13+.

## Installation

### Prerequisites

Please read the [programming manual](https://www.mecademic.com/res/doc/programming-manual) to understand
concepts necessary for proper usage of the API.

To use the `mecademicpy` library, you must have Python installed on your machine.
Python version 3.8 or newer is required.
We recommend using Python 3.13, as this is the version on which the module is actively tested.

[Python](https://www.python.org/) can be downloaded from the official website.

You can validate your Python installation by running the following command in a terminal:

```bash
python --version
```

Note that on some platforms (Linux, macOS), you may need to use `python3` instead of `python`:

```bash
python3 --version
```

The `mecademicpy` library is compatible with Windows, Linux, and macOS.

### Installing the mecademicpy package

You can easily install the package using pip.
Pip will download and install the package, along with its required dependencies, on your local machine.
To install, run the following command:

```
pip install mecademicpy
```

### Optional dependencies

By default, this package installs with only the **minimal dependencies** required to control a Mecademic robot.

Additional features require extra dependencies. These can be installed using pip with extras:

- `pip install mecademicpy[update_robot]`
  → Enables `robot.UpdateRobot` for updating the robot firmware.

- `pip install mecademicpy[trajectory_logger]`
  → Enables `robot.StartLogging` to capture trajectory (real-time) data.

- `pip install mecademicpy[full]`
  → Installs **all** optional dependencies to unlock all advanced features of `mecademicpy`.

See setup.py for full details on what each extra includes.

## Quick start

### Ensure the robot is powered and functional

Connect and power the robot, as explained in the robot's
[user manual](https://www.mecademic.com/res/doc/programming-manual).

If you can access the robot using its built-in web interface (MecaPortal), the `mecademicpy` library should
also be able to connect.

**Note:** Only one controlling user can be connected to the robot at a time, so your Python program cannot
connect in control mode if MecaPortal is already controlling the robot.

### Connecting to the robot using mecademicpy's robot class

In a Python shell or script, import the library, create a `Robot` instance, and call `Connect` with
the robot's IP address:

```python
import mecademicpy.robot as mdr
robot = mdr.Robot()
robot.Connect(address='192.168.0.100')
```

`Connect` raises an exception if the connection fails.
This function is synchronous (waits for success or timeout) even when the Robot class is used in asynchronous mode.

### Activating the robot

Before the robot can move, it must be activated and homed. To do so, run the following functions:

```python
robot.ActivateRobot()
robot.Home()
```

Some models may move slightly during homing.
You can use `robot.WaitHomed()` to block execution until homing is done.

### Moving the robot

After homing, the robot is ready to operate. For example:

```python
robot.MoveJoints(0, 0, 0, 0, 0, 0)
robot.MoveJoints(0, -60, 60, 0, 0, 0)
```

### Deactivating and disconnecting from the robot

Deactivating and disconnecting can be done with the following commands:
Deactivating before motion completes will stop the robot immediately.
Use `robot.WaitIdle()` to wait for motion completion.

```python
robot.WaitIdle()
robot.DeactivateRobot()
robot.Disconnect()
```

### Waiting for robot execution

In asynchronous mode, the `mecademicpy` API provides several functions to wait for different robot events,
like [checkpoints](#checkpoints) or [robot wait utilities](#robot-wait-utilities).

You can also use `WaitIdle()` which automatically posts a checkpoint and wait:

### More example programs

For complete examples, see the examples folder in the `mecademicpy` package.

## API commands

The `mecademicpy` library provides most of the TCP/IP API functions, additional Python-specific functions,
and several utility functions.
Please refer to the [programming manual](https://www.mecademic.com/res/doc/programming-manual) for detailed
information about functions that also exist in the TCP/IP API.
Documentation for Python-specific functions can be found in the `Robot` class alongside their definitions.

Below is a summary of the available functions, grouped by category.

### Commands specific to the mecademicpy library

Example of functions that exist only in the `mecademicpy` Python API (and not in the TCP/IP text API):

- `Connect` / `Disconnect`
- `IsConnected` / `IsControlling` / `IsSynchronousMode`
- `ConnectionWatchdog` / `AutoConnectionWatchdog`
- `RegisterCallback` / `UnregisterCallback`
- `SyncCmdQueue`
- Utility functions:
  - `reset_motion_queue`
  - `reset_robot_configuration`
  - `reset_vacuum_module`

### Robot wait utilities

- `WaitActivated` / `WaitHomed` / `WaitDeactivated`
- `WaitEndOfCycle`
- `WaitForError` / `WaitErrorReset`
- `WaitGripperMoveCompletion`
- `WaitHoldingPart` / `WaitReleasedPart` / `WaitPurgeDone`
- `WaitIdle`
- `WaitInputState` / `WaitOutputState`
- `WaitMotionCleared`
- `WaitMotionPaused`
- `WaitMotionResumed`

### Utility functions

Example of utility functions that exist only in the `mecademicpy` Python API (and not in the TCP/IP text API):

- `IsAllowedToMove`
- `GetInterruptableEvent`
- `UpdateRobot`

### Robot operation state controlling functions

Example of functions for controlling the robot's operational state (also available in the TCP/IP text API):

- `ActivateRobot` / `Home` / `DeactivateRobot`
- `RebootRobot`
- `ActivateSim` / `DeactivateSim`
- `SetMonitoringInterval` / `SetRealTimeMonitoring`
- `SetRecoveryMode`

### Motion commands

Example of motion commands that cause the robot to move:

- `MoveJoints` / `MoveJointsRel`
- `MoveJump` (Scara robots only)
- `MoveLin` / `MoveLinRelWrf` / `MoveLinRelTrf`
- `MovePose`

### Motion-related commands

Example of motion-related commands for controlling the motion queue status (also available in the TCP/IP text API):

- `PauseMotion` / `ResumeMotion` / `ClearMotion`
- `ResetError`
- `SetTimeScaling`

### Velocity mode commands

Example of commands that control the robot velocity in real-time (generally used for jogging the robot):

- `MoveJointsVel`
- `MoveLinVelWrf` / `MoveLinVelTrf`

### Motion queue parameter commands

Example of commands that control the robot's motion queue parameters:

- `SetVelTimeout`
- `SetConf` / `SetAutoConf` / `SetConfTurn` / `SetAutoConfTurn`
- `SetCheckpoint` / `GetCheckpoint`
- `SetBlending`
- `SetCartAcc` / `SetCartAngVel` / `SetCartLinVel`
- `SetJointAcc` / `SetJointVel` / `SetJointVelLimit`
- `SetMoveJumpApproachVel` / `SetMoveJumpHeight`
- `SetMoveMode` / `SetMoveDuration` / `SetMoveDurationCfg`
- `SetPayload`
- `SetTorqueLimits` / `SetTorqueLimitsCfg`
- `SetTrf` / `SetWrf`

### External tool motion queue commands

Example of commands that control the robot's external tools (gripper, valve box, IO module, ...):

- `GripperClose` / `GripperOpen` / `MoveGripper`
- `SetGripperForce` / `SetGripperRange` / `SetGripperVel`
- `SetOutputState`
- `SetValveState`
- `VacuumGrip` / `VacuumPurge` / `VacuumRelease` / `SetVacuumPurgeDuration`

### Variable management functions

Example of variable management functions:

- `CreateVariable`
- `DeleteVariable`
- `GetVariable`
- `SetVariable`

**Note:** In Python code, once created, variables are registered as robot class attributes and can be directly accessed
instead of using robot.GetVariable or robot.SetVariable(). For example

```python
robot.variables.my_pos = [10,0,10,0,0,45]
robot.MoveJoints(*robot.variables.my_pos)
```

### File (user program) management functions

Example of file management functions:

- `DeleteFile`
- `ListFiles`
- `LoadFile`
- `SaveFile`
- `StartProgram`

### MecaScript Python engine management functions

- `GetMecaScriptEngineStatus`
- `GetMecaScriptCfg`
- `SetMecaScriptCfg`
- `WaitMecaScriptEngineReady`

### Robot configuration management

Example of commands to manage robot's persistent configuration:

- `EnableEtherNetIp` / `EnableProfinet` / `SwitchToEtherCAT`
- `SetCalibrationCfg`
- `SetCollisionCfg`
- `SetJointLimits` / `SetJointLimitsCfg`
- `SetPStop2Cfg`
- `SetRobotName`
- `SetRtc`
- `SetSimModeCfg`
- `SetToolSphere`
- `SetWorkZoneCfg` / `SetWorkZoneLimits`

### Data logging functions

Example of data logging functions that capture robot real-time data in csv files:

- `StartLogging`
- `EndLogging`
- `GetCapturedTrajectory` (generally used as `robot.GetCapturedTrajectory().get_captured_data()`)

### Querying robot state, position, configuration, etc

Example of functions to query robot information:

- `GetRobotInfo`
- `GetRobotCalibrated`
- `GetRobotName`
- `GetRobotSerial`

Example of functions to query the robot's status:

- `GetCollisionStatus` / `GetWorkZoneStatus`
- `GetMonitoringInterval`
- `GetOperationMode`
- `GetRecoveryMode`
- `GetSafetyStatus`
- `GetStatusRobot`

Example of functions to query the robot's real-time position and data:

- `GetRobotRtData`
- `GetRtCartPos` / `GetRtJointPos` / `GetRtJointTorq` / ...
- `GetRtTargetCartPos` / `GetRtTargetJointPos` / ...

Example of functions to query robot's motion queue state:

- `GetAutoConf` / `GetAutoConfTurn` / `GetConf` / `GetConfTurn`
- `GetBlending`
- `GetCartAcc` / `GetCartAngVel` / `GetCartLinVel`
- `GetJointAcc` / `GetJointVel` / `GetJointVelLimit`
- `GetMoveJumpApproachVel` / `GetMoveJumpHeight`
- `GetMoveMode` / `GetMoveDuration` / `GetMoveDurationCfg`
- `GetTimeScaling`
- `GetTorqueLimits` / `GetTorqueLimitsCfg` / `GetTorqueLimitsStatus`
- `GetTrf` / `GetWrf`
- `GetVacuumPurgeDuration` / `GetVacuumThreshold`
- `GetVelTimeout`

Example of functions to manage the external tool or IO module:

- `GetGripperForce` / `GetGripperRange` / `GetGripperVel`
- `GetRtExtToolStatus`
- `GetRtGripperForce` / `GetRtGripperPos` / `GetRtGripperState`
- `GetRtIoStatus` / `GetRtInputState` / `GetRtOutputState`
- `GetRtVacuumPressure` / `GetRtVacuumState` / `GetRtValveState`

Example of functions to query robot's persistent configuration:

- `GetCalibrationCfg`
- `GetCollisionCfg` / `GetWorkZoneCfg` / `GetWorkZoneLimits` / `GetToolSphere`
- `GetEtherNetIpEnabled` / `GetProfinetEnabled`
- `GetJointLimits` / `GetJointLimitsCfg` / `GetModelJointLimits`
- `GetNetworkCfg`
- `GetPStop2Cfg`
- `GetSimModeCfg`
- `GetToolSphere`

## Library structure and main modules

The `mecademicpy` library is made up of several Python files. As a user of this library,
you will mostly refer to the following:

- robot.py: Defines the `Robot` class, the main class of the `mecademicpy` library. This class provides functions
  to connect to a Mecademic robot and control it using the [API commands](#api-commands).

- robot_classes.py: Defines [data classes](#data-classes) used throughout the `Robot` APIs to report robot state,
  configuration, and data. It also defines exception classes that may be raised by the `Robot` class functions.

- robot_initializer.py: Contains utility functions to [restore a well-known robot configuration](#restore-a-well-known-robot-configuration).

- mx_robot_def.py: Defines most of the [Enum classes](#enum-classes) used as arguments in various API calls.

Other files in the library are used for internal implementation.
You typically don’t need to refer to them to understand or use the `mecademicpy` library and the `Robot` class.

### Enum classes

The `mecademicpy` library defines multiple enum classes representing valid values for various API calls.
These enums are generated from the robot’s internal codebase, ensuring that the Python values match those
expected by the robot firmware.

Some examples include:

- `MxRobotModel`: Lists the supported robot models.
- `MxEventSeverity`: Used to define the severity of certain robot conditions, such as torque limits exceeded or motion
  outside the allowed workspace.
- `MxSafeStopCategory`: Lists the possible safety stop conditions that the robot may report.
- `MxRobotStatusCode`: Lists all status codes the robot can send to the application.
  Most of these codes are handled internally by the `mecademicpy` library and decoded into corresponding
  [data classes](#data-classes), but some API calls may require explicit use of them.
- etc. Please refer to `mx_robot_def` for a full list of enum classes and their documentation.

### Data classes

The `mecademicpy` library organizes robot status, configuration, and real-time data into data classes.
These are returned by various `Robot` class functions, or used as input parameters in some cases.

Some examples include:

- `InterruptableEvent`: A waitable object returned by some `Robot` functions like `SetCheckpoint()`.
- `RobotInfo`: Contains robot model, firmware version, serial number, etc. (returned by `GetRobotInfo()`).
- `TimestampedData`: Contains real-time robot data (e.g., joint positions) along with a timestamp.
- `RobotRtData`: Aggregates all available real-time data reported by the robot during execution.
- `RobotStatus`: Indicates the robot’s operational state (e.g., activation, motion pause, error).
- `RobotSafetyStatus`: Reports the state of safety signals such as EStop, PStop, and enabling device.
- etc. Please refer to `robot_classes` for a full list of classes and their documentation.

## Features and additional information

### Asynchronous mode considerations

By default, the API operates in _asynchronous mode_, which is the most flexible and powerful way to write
a Python program that controls the robot.

In _asynchronous mode_, `mecademicpy` API calls (like `MoveJoints`) send the command to the robot and return
immediately, allowing the Python code to continue executing while the robot moves, and enabling you to
post more commands or perform other tasks.

On the other hand, in _synchronous mode_, each `mecademicpy` function call blocks until the robot has finished
executing the command. This mode may be easier to use when writing simple applications, but it is much less flexible
and less efficient in terms of trajectory optimization and cycle time.

Things to keep in mind when using _asynchronous mode_:

- You can benefit from motion command blending on the robot;
- You can use [checkpoints](#checkpoints) to follow (or wait for) robot execution through the motion queue;
- You can use one of the `mecademicpy` API [robot wait utilities](#robot-wait-utilities);
- You can use [callbacks](#callbacks) to get informed of various robot events;
- You can [wait for specific robot message](#wait-for-specific-robot-message).

**Pitfalls**

- The robot may enter an error state while your application is busy doing something else, making it harder to identify
  which command caused the error. You may need to inspect the error message (`robot.GetStatusRobot().error_msg`)
  or check the robot logs.
- Some query functions return the current state of the motion queue (not the most recently posted command).
  For example, calling `SetJointVel()` followed immediately by `GetJointVel()` may not return the set value,
  as the command may still be queued.
- Real-time data query functions (such as `GetRobotRtData()`) return slightly outdated data, reflecting the
  most recent monitoring interval.
  You can force a synchronous update (e.g., `GetRobotRtData(synchronous_update=True)`) to request up-to-date
  data from the robot.

### Checkpoints

The checkpoint system allows you to create event objects that are triggered when the robot reaches a
specific point in its execution. Calling `SetCheckpoint(n)` registers a checkpoint with the robot
(using integer 'n' as the ID) and returns an event-like object that can be used to wait for the checkpoint.
You can also use `GetCheckpoint()` to check the most recently reached checkpoint id.

Example:

```python
# In async mode, the following commands are queued immediately without waiting for the robot to move

# Queue a move to the first position
robot.MoveJoints(0, 0, 0, 0, 0, 0)
first_checkpoint = robot.SetCheckpoint(1)

# Queue a move to a second position
robot.MoveJoints(20, 20, 0, 0, 35, 0)
second_checkpoint = robot.SetCheckpoint(2)

# Wait until the robot reaches the first position
first_checkpoint.wait()
logging.info('First position reached')

# Wait until the robot reaches the second position
second_checkpoint.wait()
logging.info('Second position reached')
```

If the robot’s motion command queue is cleared (using `ClearMotion()` or due to a robot error or deactivation),
all pending checkpoints are aborted, and calling `wait()` on checkpoint objects raise an `InterruptException`.

### External checkpoints

Checkpoints can also be set outside the Python application, such as in a program started using `StartProgram`.
To wait for such a checkpoint, the Python application must call `ExpectExternalCheckpoint(n)`
before starting the program. This returns a checkpoint event object that will remain blocked until the robot
reports reaching the corresponding checkpoint.

Example:

```python
checkpoint_event = robot.ExpectExternalCheckpoint(5)
robot.StartProgram("my_program")
checkpoint_event.wait(timeout=30)
print("Received expected checkpoint from running program")
```

where "my_program" is, for example

```
MoveJoints(100,0,0,0,0,0)
SetCheckpoint(5)
MoveJoints(0,0,0,0,0,0)
```

### Callbacks

The `Robot` class supports user-provided callback functions for various robot events.
These callbacks are entirely optional but can be useful for building asynchronous applications.

Callbacks are registered using the `RegisterCallback` or `RegisterCallbacks` methods.

There are two ways to handle callback execution:

- **Manually triggered:** Callbacks are executed only when the application explicitly calls `RunCallbacks()`.
- **Automatically triggered:** Callbacks are executed in a dedicated thread. This mode is enabled by passing
  `run_callbacks_in_separate_thread=True` when calling `RegisterCallbacks`.

**Warning:** If you use the threaded mode, make sure your callback functions are thread-safe. You may
use Python's `threading.Lock()` to protect shared resources if needed.

Some available callbacks include:

- `on_connected`
- `on_disconnected`
- `on_activated`
- `on_deactivated`
- `on_homed`
- `on_error`
- `on_checkpoint_reached`
- etc. (Refer to the `RobotCallbacks` class for a complete list.)

**Note:** Some callbacks receive arguments. For example:

- `on_checkpoint_reached` receives the checkpoint ID.
- `on_command_message` and `on_monitor_message` receive a `robot.Message` object.
  Refer to the `RobotCallbacks` class documentation for details.

Here is an example usage of callbacks:

```python
# Prepare a list to accumulate reached checkpoints
reached_checkpoints_list:list[int] = []

# Define and register a callback that adds the reached checkpoint to the list
def on_checkpoint_reached(checkpoint_id):
    reached_checkpoints_list.append(checkpoint_id)

robot.RegisterCallback('on_checkpoint_reached', on_checkpoint_reached)

# Run a program and wait for completion
robot.StartProgram("my_program")
robot.WaitIdle()

# Check result
print(f'Checkpoints reached during program: {reached_checkpoints_list}')
```

### Handling robot errors

If the robot encounters an error during use, it enters an error state.
In this mode, the robot will reject all commands until the error is reset.

When the robot is in an error state:

- Waitable events ([checkpoints](#checkpoints) and some [Robot wait utilities](#robot-wait-utilities)) will raise
  an `InterruptException`;
- `GetStatusRobot().error_status` will be True;
- `GetStatusRobot().error_code` will show the robot error code
  (see the [programming manual](https://www.mecademic.com/res/doc/programming-manual));
- `GetStatusRobot().error_msg` will contain a human-readable message explaining the error.

To properly reset the robot error state, call:

```python
robot.ResetError()
robot.ResumeMotion()
```

You can also use the `on_error` callback to handle robot errors asynchronously (see [callbacks](#callbacks))

### Handling Python errors

Incorrect usage of the Python API may raise exceptions.
For example, calling MoveJoints() without arguments will raise an error.

Use Python’s built-in "try...except" blocks to catch and handle such exceptions appropriately.

### Handling robot safety stop conditions

The robot may enter safety stop conditions due to external safety signals
(EStop, PStop1, PStop2, operation mode change, enabling device released) or other conditions
(such as connection timeout).

The robot safety status can be obtained using `GetSafetyStatus`.
The `on_safety_stop`, `on_safety_stop_reset`, `on_safety_stop_resettable` and `on_safety_stop_state_change`
[callbacks](#callbacks) can be used to monitor and manage safety stop signals.

Some safety stop signals remove motor voltage (EStop, PStop1, operation mode change, etc.).
While these signals are active, the robot cannot be activated.
For safety reasons, these signals cannot be reset programmatically via the `mecademicpy` API; they require
pressing the Reset button on the power supply or triggering the reset via dedicated power supply input pins.

Other safety stop signals will only pause or clear robot motion (PStop2, enabling device released, connection dropped).
Once these conditions are resolved, the robot can be activated, or if already activated,
motion can be resumed using `ResumeMotion()`.

For more information about safety signals, see the robot's [programming manual](https://www.mecademic.com/res/doc/programming-manual).

### Disconnect on exception

By default, if any unrecoverable error occurs while using the `Robot` class, the class will automatically
disconnect from the robot to avoid potential issues. This disconnection causes the robot to pause motion.

However, disconnecting on exceptions may be undesirable when using an interactive terminal or Jupyter notebook,
as an accidental or malformed function call could trigger a disconnection.
In these cases, this feature can be disabled by setting `disconnect_on_exception=False` when calling `Connect()`:

```python
robot.Connect(address='192.168.0.100', disconnect_on_exception=False)
```

### Restore a well-known robot configuration

When connecting to the robot from a Python application, the robot can be in any state:

- activated or deactivated
- in error or not
- in a safety stop or not
- motion paused or resumed
- the motion queue may contain commands or be empty
- the motion queue parameters may not match expected values (joint velocity, acceleration, blending, etc.)
- the external tool state may vary (gripper open/closed, valve states, digital outputs values, vacuum, etc.)
- in simulation mode or not
- in recovery mode or not
- the monitoring interval and optional enabled monitoring events may have been changed
- etc.

The `mecademicpy` library provides utility functions (in `robot_initializer`) to restore the robot
to a well-known state before your application starts using it.

`RobotWithTools` is a specialization of the `Robot` class that adds utilities needed by
the functions in `robot_initializer`. Applications that want to use these functions should
instantiate `RobotWithTools` instead of `Robot`.

- `reset_motion_queue()`:
  Configures the robot's motion queue with default or user-defined values.
  Receives an instance of `MotionQueueParams` which lists all available motion queue parameters with default values.
- `reset_robot_configuration()`
  Restores the robot's static (permanent) configuration parameters, including joint limits,
  work zone limits, PStop2 configuration, etc.
- `reset_vacuum_module()`
  Resets vacuum module states (digital outputs, vacuum parameters, and states) to default values.

Example

```python
# Use a "with" block to ensure proper disconnection at the end
with initializer.RobotWithTools() as robot:
    robot.Connect(address='192.168.0.100')

    # Reset the robot's persistent configuration (joint limits, work zone, etc.)
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

Mecademic robots support persistent variables that remain valid even after a reboot or power cycle.
These variables are useful in Python programs for a variety of tasks, such as storing target positions or reference
frames that have been pre-defined and fine-tuned on each of your robots.

You can:

- create a variable (`CreateVariable`)
- delete a variable (`DeleteVariable`)
- list variables (`ListVariables`)
- read a variable (`value = robot.variables.variable_name`)
- modify a variable (`robot.variables.variable_name = new_value`)

For further details on robot variables, consult the robot's
[programming manual](https://www.mecademic.com/res/doc/programming-manual).

#### Quick example:

```python
# Create a variable (stored permanently stored on the robot)
robot.CreateVariable("my_wrf", [0,0,0,0,0,0])
# Modify the variable's value (change is permanently saved on the robot)
robot.variables.my_wrf = [10,-10,0,0,0,45]
# Use the variable value (cached in the Robot class for performance, so this access is non-blocking)
print(f'my_wrf={robot.variables.my_wrf}')
# Delete the variable from the robot
robot.DeleteVariable("my_wrf")
```

### Capturing robot real-time data

You can capture the robot's real-time data (position, state, etc.) to a `.csv` file or to a Panda DataFrame.

#### Starting a real-time data capture

You can start a real-time data capture, either with the `StartLogging` / `EndLogging` functions
or with the `FileLogger` context manager, which automatically ends logging after the "with" block
completes or if an exception occurs.

```python
robot.WaitIdle()
with robot.FileLogger(monitoringInterval=0.001):
    robot.MoveJoints(0, -60, 60, 0, 0, 0)
    robot.MoveJoints(0, 0, 0, 0, 0, 0)
    robot.WaitIdle()
print(f'Done capturing in file {robot.GetCapturedTrajectoryPath()}')
```

**Note:** Wait for the robot to be idle (WaitIdle) before starting and ending logging to ensure the log
correctly captures the movements of interest.

For a more complete example, see trajectory_logger_example.py in the `mecademicpy` package.

#### Choosing fields to capture

To reduce the amount of captured data, you can increase the monitoring interval, or specify which real-time fields
to capture using the "fields" argument.
This argument can be a list of robot status codes (`MxRobotStatusCode`) or `RobotRtData` attribute names.

For example:

```python
fields_to_capture = [mdr.MxRobotStatusCode.MX_ST_RT_CART_POS, mdr.MxRobotStatusCode.MX_ST_RT_CART_VEL]
robot.StartLogging(monitoringInterval=0.001, fields=fields_to_capture)
```

or

```python
robot.StartLogging(monitoringInterval=0.001, fields=['rt_cart_pos', 'rt_cart_vel'])
```

#### Accessing the captured data

After capture, the path of the capture archive (`.zip`) is returned by `robot.GetCapturedTrajectoryPath()`.
This archive contains a `.json` file describing the robot and executed commands, as well as a `.csv` file
containing all the captured data at the requested monitoring interval.
You can also specify a custom file path using the "file_name" argument in `StartLogging`.

The captured data can be retrieved as a Panda DataFrame using `robot.GetCapturedTrajectory().get_captured_data()`:

```python
captured_df: pd.DataFrame = robot.GetCapturedTrajectory().get_captured_data()
```

The DataFrame will have columns for each captured real-time field. Some fields may span multiple columns
(e.g., joint positions have one column per joint, Cartesian positions as x, y, z, etc.).
To get the column name prefix for a specific field, use the `rt_data_field_by_status_code` map.

Please refer to the official [Pandas Dataframe](https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.html)
class documentation for more information.

Example: extract all columns representing Cartesian position:

```python
# Get the column names prefix for MX_ST_RT_CART_POS
cart_pos_col_prefix = mdr.rt_data_field_by_status_code[mdr.MxRobotStatusCode.MX_ST_RT_CART_POS].col_prefix
# Keep the names of the matching columns
cart_pos_col_names = [col for col in captured_df.columns if col.startswith(cart_pos_col_prefix)]
# Extract a DataFrame with only the desired columns
cart_pos_df = captured_df[cart_pos_col_names]
```

### Sending custom commands

You can send an arbitrary TCP/IP text command to the robot using the `SendCustomCommand()` method.
Optionally, you can provide a list of expected response codes. If provided, `SendCustomCommand()` will return
an event object that can be used to wait for the matching response.

Example usage:

```python
import mecademicpy.robot as mdr
import mecademicpy.mx_robot_def as mdr_def
# Connect, activate, and home robot...

response_codes = [mdr_def.MX_ST_ERROR_RESET, mdr_def.MX_ST_NO_ERROR_RESET]
response = robot.SendCustomCommand('ResetError', expected_responses=response_codes, timeout=10)
if response.id == mdr_def.MX_ST_ERROR_RESET:
    print('The error was in error, the error is now reset')

```

### Wait for specific robot message

The `GetInterruptableEvent()` method creates a waitable event object.
This object provides a `wait()` method that blocks the Python application until the robot sends the
expected message (or until timeout).
The event can optionally be configured to raise an `InterruptException` if the robot encounters an
error or if motion is cleared.

Note that the API already provides convenient wait methods for many common conditions, such as:
`WaitActivated`, `WaitMotionPaused`, `WaitIdle`, `WaitHoldingPart`, etc.
`GetInterruptableEvent()` is typically used for specific cases that are not covered by those
built-in wait methods.

Example 1 (note that here you could use `WaitInputState` instead):

```python
# Create an interruptable event that will wait until the robot sends MX_ST_RT_INPUT_STATE
# which happens when a digital input state changes.
input_state_changed_event = robot.GetInterruptableEvent([mdr.MxRobotStatusCode.MX_ST_RT_INPUT_STATE])

# (code here could trigger a digital input change)
# ...

# Wait (block) until the event is received or until timeout (raises TimeoutException)
input_state_changed_event.wait(10)
```

Example 2:

```python
# Create an interruptable event that will trigger when the torque limit is exceeded
# which happens when MX_ST_TORQUE_LIMIT_STATUS with data '1' is received.
# This event is configured to raise InterruptException if a robot error occurs or if motion is cleared.
torque_exceeded_event = robot.GetInterruptableEvent(
    codes=[mdr.Message(mdr.MxRobotStatusCode.MX_ST_TORQUE_LIMIT_STATUS, '1')],
    abort_on_error=True,
    abort_on_clear_motion=True)

# (code here could move the robot or perform other actions)
# ...

# Example: Check if the torque-exceeded event was already received
if torque_exceeded_event.is_set():
    # Torque limit was exceeded between the call to GetInterruptableEvent and now
    pass

# Wait (block) until the event is received or until timeout (raises TimeoutException)
torque_exceeded_event.wait(10)

```

### Managing network timeouts

When the robot loses connection with the controlling application, it will automatically pause motion.
However, in some network failure scenarios (e.g., connection cannot be closed explicitly), detecting a
disconnection can take a long time.

To ensure the robot stops promptly when the application cannot communicate, it is strongly recommended to use
the connection watchdog feature of the `mecademicpy` library.
This provides a deterministic maximum time before the robot considers the connection lost.

There are two ways to use the connection watchdog:

1. **Automatic watchdog**: Call `AutoConnectionWatchdog` to let `mecademicpy` automatically enable and refresh
   the watchdog at appropriate intervals. This is the simplest option for most applications.
2. **Manual watchdog**: Call `ConnectionWatchdog` periodically to manually refresh the connection timer.
   This approach gives you precise control and ensures that if your application becomes blocked, deadlocked,
   or encounters an exception, the robot will quickly detect the issue and stop moving.

Using either method ensures safer operation and faster response to network (or application) issues.

### Performance considerations

Because Python is relatively slow in terms of execution speed, some older PCs may struggle to receive all
data from the robot when the monitoring interval is very short (e.g., 1 ms) and all optional real-time data
has been enabled (`SetRealTimeMonitoring(all)`).
This issue is especially noticeable when running your application in a debugger.
To improve performance, you can reduce the amount of data received from the robot by increasing the monitoring interval.

## Getting help

If you need support, please open an issue on the
[MecademicPy GitHub page](https://github.com/Mecademic/mecademicpy/issues) or contact us by email at
[support@mecademic.com](mailto:support@mecademic.com).

## License

All packages in this repository are licensed under the MIT license.

## Authors

- **Mecademic** - _Continuous work_
