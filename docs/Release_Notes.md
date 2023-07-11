# Release Notes
---
The information contained herein is the property of Mecademic Inc. and shall not be reproduced in whole or in part without prior written approval of Mecademic Inc. The information herein is subject to change without notice and should not be construed as a commitment by Mecademic Inc. This manual will be periodically reviewed and revised.

If you have any technical questions, please use the technical support form on our web site (http://mecademic.com/contact-technical-support).

Mecademic Inc.~assumes no responsibility for any errors or omissions in this document.

Copyright &copy; 2023 by Mecademic Inc.

---

## Version 1.4.0 (July 2023)

**Features**
- Update API to support Mcs500
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
- Rename udpate_robot to UpdateRobot for API name uniformity
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