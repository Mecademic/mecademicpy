# Release Notes
---
The information contained herein is the property of Mecademic Inc. and shall not be reproduced in whole or in part without prior written approval of Mecademic Inc. The information herein is subject to change without notice and should not be construed as a commitment by Mecademic Inc. This manual will be periodically reviewed and revised.

If you have any technical questions, please visit the support section of our web site (https://www.mecademic.com/res/support).

Mecademic Inc.~assumes no responsibility for any errors or omissions in this document.

Copyright &copy; 2025 by Mecademic Inc.

---
## Version 2.3.0 (March 2025)
**Features**
- Support for time-based movements (`SetMoveMode`, `SetMoveDurationCfg`, `SetMoveDuration`)
- Support for fast simulation mode (`MxRobotSimulationMode.MX_SIM_MODE_FAST` with `ActivateSim`)
- Support for relative-torque limit mode (`MxTorqueLimitsMode.MX_TORQUE_LIMITS_MODE_DELTA_WITH_EXPECTED` with `SetTorqueLimitsCfg`)
- Support for robot variables (`CreateVariable`, `robot.vars.my_var=my_val`, etc.)

**Fixes**
- ROBOT-3186: RobotRtData does not populate data correctly when in Monitor mode

**Improvements**
- Improved performance in some situations
- Better detection of closed connection with the robot and awakening awaiting threads

**Migration Guide**
Some methods have been renamed. Although the legacy name remains supported, we recommend updating your
code to use the new function names:
- `ListPrograms` -> `ListFiles`
- `LoadProgram` -> `LoadFile`
- `SaveProgram` -> `SaveFile`
- `DeleteProgram` -> `DeleteFile`
- `SaveProgram` -> `SaveFile`
- `GetNetworkConfig` -> `GetNetworkCfg`

The `offline_mode` parameter of `robot.Connect` has been removed (was not used).

## Version 2.2.0 (May 2024)
**Features**
- Updated API to report Mcs500 power supply input states (GetPowerSupplyInputs -> RobotPowerSupplyInputs)
- Updated API to support new safety signal "minor error"
- Updated API to support discarded checkpoint notifications from robot (firmware version 10.2.1 and above)
- Added possibility to register/unregister callback methods without disconnecting from the robot
- New method GetInterruptableEvent that can be used to wait for specific robot events (by id, or id and data)

**Improvements**
- Fixed coding style and errors reported by pylint 3.1.0
- Improved Python type hints for RobotCallbacks
- Improved documentation for RobotRtData to clarify which values are reported by the robot by default

## Version 2.1.0 (February 2024)
**Features**
- Updated API to support new safety signals APIs:
  - For robot firmware 10.1 and above:
    - SetPStop2Cfg (for robot firmware 10.1 and above)
    - ConnectionWatchdog
  - Changes also available on older robot firmware versions:
    - GetSafetyStatus, returning new status class RobotSafetyStatus
    - And other appropriate APIs and callbacks in class Robot for managing safety signals
- Updated API to support self-collision and work zone configuration and status (robot firmware 10.1 and above)
  - GetCollisionStatus, returning new class CollisionStatus (reporting collision status and work zone status)
  - SetWorkZoneCfg, SetWorkZoneLimits, SetCollisionCfg
- Support for SetTimeScaling (robot firmware 10.0 and above)

**Migration Guide**
Some methods related to safety signals have been deprecated. It is recommended to update Python's application code to
use the new safety signal APIs instead:
- Deprecated callbacks: on_estop, on_estop_reset, on_estop_resettable, on_pstop2, on_pstop2_reset, on_pstop2_resettable
- Deprecated methods: WaitPStop2Reset, WaitPStop2Resettable, WaitEStopReset, WaitEStopResettable and ResetPStop2
- New callbacks: on_safety_stop, on_safety_stop_reset, on_safety_stop_resettable, on_safety_stop_state_change
- New methods: WaitSafetyStopReset, WaitSafetyStopResettable and WaitSafetyStopStateChange, GetSafetyStatus

Previous support for beta "workspace" feature has been removed, replaced by the final API for managing collision
and work zone configuration and status.
- Removed methods: SetWorkspaceLimitsCfg, SetWorkspaceLimits
- New methods: SetWorkZoneCfg, SetWorkZoneLimits, SetCollisionCfg, GetCollisionStatus

## Version 2.0.0 (November 2023)

**Features**
- Update API to support Mcs500 and its Mvk01 vacuum and IO module
- Update API to support new features from robot firmware 10.0
  - MoveJump API (MoveJump and related jump configuration methods)
- Better usage of user-defined socket timeout (from "Connect" method)

**Migration Guide**
The method "UpdateRobot" has been modified.
- The "ip_address" parameter has been removed.
- It is now mandatory to call method "Connect" before calling "UpdateRobot"

## Version 1.4.0 (July 2023)

**Features**
- Update API to support Mcs500 (Beta)
- Update API to support new features from robot firmware 9.3 and 9.4-alpha

## Version 1.3.0 (November 2022)

**Features**
- Update API to support Meca500 R4
- Update API to support new features in 9.2 robot firmware

**Migration Guide**
Some defines were refactored in the mx_robot_def.py. Therefore, user should do one the following:
- Change 'import mecademicpy.mx_robot_def' to 'import mecademicpy.mx_robot_def_legacy'
or
- Modify code to use the new defines

## Version 1.2.1 (February 2022)

**Fixes**
- Fix possible infinite timeout when sending command in synchronous mode

## Version 1.2.0 (February 2022)

**Features**
- Update API to support new features in 9.1 robot firmware

**Fixes**
- Fix connection in monitoring mode for robot running version 8.3
- Fix UpdateRobot that didn't always timeout
- Streamlining robot method cases
- Fix GetRtExtToolStatus data that was not always included on the connect
- Add 'deprecation' module requirement

## Version 1.1.0 (November 2021)

**Features**
- Update API to support new features from 8.4.4 robot firmware
- Update API to support new features from 9.0 robot firmware

**Fixes**
- Rename update_robot to UpdateRobot for API name uniformity
- Fix UpdateRobot to support 8.4.4 and up
- Fix annotation issues for Python 3.7. Added annotations for return values
- Prevent Disconnect() errors when already disconnected
- Make sure initial status is received before completing connection
- Add missing 'requests' module requirement
- Fix issue when using Connect() command on an already connected robot

## Version 1.0.0 (September 2021)

**Features**
- Compatible with robot firmware version 8.3 and up
- Synchronous and Asynchronous Mode compatible
- Checkpoints support
- Robot Event Callbacks
- Preserved State on Disconnection
- Robot command and state logging
- Robot state updates up to 1kHz frequency
- Multithreading/Multiprocessing compatible