#!/usr/bin/env python3
import threading
import queue
import re


class MecademicException(Exception):
    """Base exception class for Mecademic-related exceptions.

    """
    pass


class InvalidStateError(MecademicException):
    """The internal state of the instance is invalid.

    """
    pass


class CommunicationError(MecademicException):
    """There is a communication issue with the robot.

    """
    pass


class DisconnectError(MecademicException):
    """A non-nominal disconnection has occurred.

    """
    pass


class InterruptException(MecademicException):
    """An event has encountered an error. Perhaps it will never be set.

    """
    pass


class InterruptableEvent:
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
    def __init__(self, id=None):
        self.id = id
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
            raise InterruptException('Event received exception, possibly because event will never be triggered.')
        return success

    def set(self):
        """Set the event and unblock all waits.

        """
        with self._lock:
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


class TimestampedData:
    """ Class for storing timestamped data.

    Attributes
    ----------
    timestamp : number-like
        Timestamp associated with data.
    data : object
        Data to be stored.

    """
    def __init__(self, timestamp, data):
        self.timestamp = timestamp
        self.data = data

    def update_from_csv(self, input_string):
        """Update from comma-separated string, only if timestamp is newer.

        Parameters
        ----------
        input_string : string
            Comma-separated string. First value is timestamp, rest is data.

        """
        floats = string_to_floats(input_string)

        if (len(floats) - 1) != len(self.data):
            raise ValueError('Cannot update TimestampedData with incompatible data.')

        if floats[0] > self.timestamp:
            self.timestamp = floats[0]
            self.data = floats[1:]

    def update_from_data(self, timestamp, data):
        """Update with data if timestamp is newer.

        Parameters
        ----------
        timestamp : number-like
            Timestamp associated with data.
        data : object
            Data to be stored if timestamp is newer.

        """
        if timestamp > self.timestamp:
            self.timestamp = timestamp
            self.data = data

    @classmethod
    def zeros(cls, length):
        """ Construct empty TimestampedData object of specified length.

        Parameters
        ----------
        length : int
            Length of data to construct.

        Return
        ------
        TimestampedData object

        """
        return cls(0, [0.] * length)

    def __eq__(self, other):
        """ Return true if other object has identical timestamp and data.

        Parameters
        ----------
        other : object
            Object to compare against.

        Return
        ------
        bool
            True if objects have same timestamp and data.

        """
        return other.timestamp == self.timestamp and other.data == self.data

    def __ne__(self, other):
        """ Return true if other object has different timestamp or data.

        Parameters
        ----------
        other : object
            Object to compare against.

        Return
        ------
        bool
            True if objects have different timestamp or data.

        """
        return not self == other


class Message:
    """Class for storing the internal state of a generic Mecademic robot.

    Attributes
    ----------
    id : integer
        The id of the message, representing the type of message.
    data : string
        The raw payoad of the message.

    """
    def __init__(self, id, data):
        self.id = id
        self.data = data

    def __repr__(self):
        return "Message with id={}, data={}".format(self.id, self.data)


def string_to_floats(input_string):
    """Convert comma-separated floats in string form to list of floats.

    Parameters
    ----------
    input_string : string
        Comma-separated floats values encoded as a string.

    Returns
    -------
    list of floats
        Returns converted list of floats.

    """
    return [float(x) for x in input_string.split(',')]
