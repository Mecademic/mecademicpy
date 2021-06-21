#!/usr/bin/env python3
import threading
import queue

_CHECKPOINT_ID_MAX_PRIVATE = 8191  # Max allowable checkpoint id, inclusive

_TERMINATE = '--terminate--'


class _MecademicException(Exception):
    """Base exception class for Mecademic-related exceptions.

    """
    pass


class _InvalidStateError(_MecademicException):
    """The internal state of the instance is invalid.

    """
    pass


class _CommunicationError(_MecademicException):
    """There is a communication issue with the robot.

    """
    pass


class _DisconnectError(_MecademicException):
    """A non-nominal disconnection has occurred.

    """
    pass


class _InterruptException(_MecademicException):
    """An event has encountered an error. Perhaps it will never be set.

    """
    pass


class _InterruptableEvent:
    """Extend default event class to also be able to unblock and raise an exception.

    Attributes
    ----------
    id : int or None
        Id for event.
    _event : event object
        A standard event-type object.
    _lock : lock object
        Used to ensure atomic operations.
    _interrupted : boolean
        If true, event is in an error state.

    """

    def __init__(self, id=None, data=None):
        self._id = id
        self._data = data
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._interrupted = False

    def wait(self, timeout=None):
        """Block until event is set or should raise an exception.

        Attributes
        ----------
        timeout : float
            Maximum duration to wait in seconds.

        Return
        ------
        success : boolean
            False if event timed out, true otherwise.

        """
        success = self._event.wait(timeout=timeout)
        if self._interrupted:
            raise _InterruptException('Event received exception, possibly because event will never be triggered.')
        return success

    def wait_for_data(self, timeout=None):
        """Block until event is set or should raise an exception.

        Attributes
        ----------
        timeout : float
            Maximum duration to wait in seconds.

        Return
        ------
        data : object
            Return the data object.

        """
        success = self._event.wait(timeout=timeout)
        if self._interrupted:
            raise _InterruptException('Event received exception, possibly because event will never be triggered.')
        elif not success:
            raise _InterruptException('Event timed out.')
        else:
            return self._data

    def set(self, data=None):
        """Set the event and unblock all waits. Optionally modify data before setting.

        """
        with self._lock:
            self._data = data
            self._event.set()

    def abort(self):
        """Unblock any waits and raise an exception.

        """
        with self._lock:
            if not self._event.is_set():
                self._interrupted = True
                self._event.set()

    def clear(self):
        """Reset the event to its initial state.

        """
        with self._lock:
            self._interrupted = False
            self._event.clear()

    def is_set(self):
        """Checks if the event is set.

        Return
        ------
        boolean
            False if event is not set or instance should '_interrupted'. True otherwise.

        """
        with self._lock:
            if self._interrupted:
                return False
            else:
                return self._event.is_set()

    def clear_abort(self):
        """Clears the abort.

        """
        with self._lock:
            if self._interrupted:
                self._interrupted = False
                self._event.clear()

    @property
    def id(self):
        """Make id a read-only property since it should not be changed after instantiation.

        """
        return self._id

    @property
    def data(self):
        """Make data a read-only property and enforce that it is only assignable at construction or using set().

        """
        return self._data


class _Message:
    """Class for storing a response message from a Mecademic robot.

    Attributes
    ----------
    id : integer
        The id of the message, representing the type of message.
    data : string
        The raw payload of the message.

    """

    def __init__(self, id, data):
        self.id = id
        self.data = data

    def __repr__(self):
        return "Message with id={}, data={}".format(self.id, self.data)

    @classmethod
    def from_string(cls, input):
        """Construct message object from raw string input.

        Parameters
        ----------
        input : string
            Input string to convert to message.

        """
        id_start = input.find('[') + 1
        id_end = input.find(']', id_start)
        id = int(input[id_start:id_end])
        # Find next square brackets (contains data).
        data_start = input.find('[', id_end) + 1
        data_end = input.find(']', data_start)

        data = ''
        if data_start != -1 and data_end != -1:
            data = input[data_start:data_end]

        return cls(id, data)


class _RobotEvents:
    """Class for storing possible status events for the generic Mecademic robot.

    Attributes
    ----------
    on_connected : event
        Set if robot is connected.
    on_disconnected : event
        Set if robot is disconnected.
    on_status_updated : event
        Set if robot status is updated.
    on_activated : event
        Set if robot is activated.
    on_deactivated : event
        Set if robot is deactivated.
    on_homed : event
        Set if robot is homed.
    on_error : event
        Set if robot is in error.
    on_error_reset : event
        Set if robot error has been reset.
    on_p_stop : event
        Set if robot receives p_stop.
    on_p_stop_reset : event
        Set if p_stop is reset.
    on_motion_paused : event
        Set if robot motion is paused.
    on_motion_resumed : event
        Set if robot motion is not paused.
    on_motion_cleared : event
        Set if there are no pending ClearMotion commands.
    on_activate_sim : event
        Set if robot is in sim mode.
    on_deactivate_sim : event
        Set if robot is not in sim mode.
    on_conf_updated : event
        Set if robot configuration has been updated.
    on_conf_turn_updated : event
        Set if last joint turn number has been updated.
    on_joints_updated : event
        Set if joint angles has been updated.
    on_pose_updated : event
        Set if robot pose has been updated.
    on_brakes_activated : event
        Set if brakes are activated.
    on_brakes_deactivated : event
        Set if brakes are deactivated.
    on_offline_program_started : event
        Set if there has been a change in the offline program state.
    on_end_of_block : event
        Set if end of block has been reached.
    on_end_of_cycle: event
        Set if end of cycle has been reached

    """

    def __init__(self):
        self.on_connected = _InterruptableEvent()
        self.on_disconnected = _InterruptableEvent()

        self.on_status_updated = _InterruptableEvent()

        self.on_activated = _InterruptableEvent()
        self.on_deactivated = _InterruptableEvent()

        self.on_homed = _InterruptableEvent()

        self.on_error = _InterruptableEvent()
        self.on_error_reset = _InterruptableEvent()
        self.on_p_stop = _InterruptableEvent()
        self.on_p_stop_reset = _InterruptableEvent()

        self.on_motion_paused = _InterruptableEvent()
        self.on_motion_resumed = _InterruptableEvent()
        self.on_motion_cleared = _InterruptableEvent()

        self.on_activate_sim = _InterruptableEvent()
        self.on_deactivate_sim = _InterruptableEvent()

        self.on_conf_updated = _InterruptableEvent()
        self.on_conf_turn_updated = _InterruptableEvent()
        self.on_joints_updated = _InterruptableEvent()
        self.on_pose_updated = _InterruptableEvent()

        self.on_brakes_activated = _InterruptableEvent()
        self.on_brakes_deactivated = _InterruptableEvent()

        self.on_offline_program_started = _InterruptableEvent()

        self.on_end_of_block = _InterruptableEvent()
        self.on_end_of_cycle = _InterruptableEvent()

        self.on_disconnected.set()
        self.on_deactivated.set()
        self.on_error_reset.set()
        self.on_p_stop_reset.set()
        self.on_motion_resumed.set()
        self.on_deactivate_sim.set()

        self.on_status_updated.set()
        self.on_conf_updated.set()
        self.on_conf_turn_updated.set()
        self.on_joints_updated.set()
        self.on_pose_updated.set()
        self.on_brakes_activated.set()

    def clear_all(self):
        """Clear all events.

        """
        for attr in self.__dict__:
            self.__dict__[attr].clear()

    def abort_all_except_on_connected(self):
        """Abort all events, except for on_connected.

        """
        for attr in self.__dict__:
            if attr != 'on_connected':
                self.__dict__[attr].abort()

    def clear_abort_all(self):
        """Clear aborts for all events.

        """
        for attr in self.__dict__:
            self.__dict__[attr].clear_abort()


class _CallbackQueue():
    """Queue class for storing triggered callbacks. Only registered callbacks are added to the queue.

    Attributes
    ----------
    _queue : queue
        Queue to use to store callback names and associated data.
    _registered_callbacks : set
        Set of names of registered callbacks.

    """

    def __init__(self, robot_callbacks):
        self._queue = queue.Queue()
        self._registered_callbacks = set()

        for attr in robot_callbacks.__dict__:
            if robot_callbacks.__dict__[attr] is not None:
                self._registered_callbacks.add(attr)

    def qsize(self):
        """Returns the queue size.

        """
        return self._queue.qsize()

    def put(self, callback_name, data=None):
        """Put the callback name and associated data into the queue if is registered.

        Parameters
        ----------
        callback_name : str
            Name of callback.
        data : any object type
            Associated data.

        """
        if callback_name in self._registered_callbacks or callback_name == _TERMINATE:
            self._queue.put((callback_name, data))

    def get(self, block=False, timeout=None):
        """Get the next callback in the queue.

        Parameters
        ----------
        block : bool
            Block on next available callback if true.
        timeout : float
            Maximum time to wait on a callback.

        Returns
        -------
        tuple of callback name and data

        """
        return self._queue.get(block=block, timeout=timeout)


def _string_to_numbers(input_string):
    """Convert comma-separated floats in string form to relevant type.

    Parameters
    ----------
    input_string : string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list of numbers
        Returns converted list of floats or integers, depending on nature of element in 'input_string'.

    """

    return [(float(x) if ('.' in x) else int(x)) for x in input_string.split(',')]
