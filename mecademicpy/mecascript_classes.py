"""
This file define classes used for managing Mecascript programs and registered commands.
"""
from __future__ import annotations

import copy
import enum
import inspect
from collections.abc import MutableMapping, MutableSequence
from typing import Any, Callable, Optional, Union

#pylint: disable=wildcard-import,unused-wildcard-import
from mecademicpy.mx_robot_def import *
from mecademicpy.robot_classes import NotFoundException


class MecaScriptType(enum.IntEnum):
    MXPY = 0  # Basic Python MecaScript program (.mxpy)
    PY = 1  # Advanced Python Mecascript module (.py)


class RegisteredArg:
    """ Class that describes one argument for RegisteredCommand """

    def __init__(
        self,
        name: str,
        description: str = "",
        #pylint: disable=redefined-builtin
        type: MxArgType = MxArgType.MX_ARG_TYPE_ANY,
        length: Optional[int] = None,
        units: Optional[MxArgUnit] = None,
        default: Optional[Any] = None,
        min: Optional[Any] = None,
        max: Optional[Any] = None,
    ):
        """ Mandatory constructor for this object

        Parameters
        ----------
        name
            Name of this argument/variable
        description
            Description of this argument for the users.
        type
            Type of this argument.
        length
            Length of this array (0 if single argument, not array).
        units
            Units of this argument (for documentation only).
        default
            Default value for this argument (for documentation).
        min
            Minimum value of this argument (for documentation only).
        max
            Maximum value of this argument (for documentation only).
        """
        self.name = name
        self.description = description
        self.type = type
        self.length = length
        self.units = units
        self.default = default
        self.min = min
        self.max = max

    def copy(self) -> RegisteredArg:
        """Return a shallow copy of this argument."""
        return RegisteredArg(
            name=self.name,
            description=self.description,
            type=self.type,
            length=self.length,
            units=self.units,
            default=self.default,
            min=self.min,
            max=self.max,
        )

    def __str__(self) -> str:
        """ Returns string representation """
        return self.name

    def __repr__(self) -> str:
        return str(self)

    def to_dict(self) -> dict[str, Any]:
        """ Returns a dictionary string representation of this object, ready to be serialized as JSON for the robot"""
        return {
            MX_JSON_KEY_VAR_NAME: self.name,
            MX_JSON_KEY_VAR_DESC: self.description,
            MX_JSON_KEY_VAR_TYPE: int(self.type),
            MX_JSON_KEY_VAR_ARR_LEN: self.length,
            MX_JSON_KEY_VAR_UNIT: int(self.units) if self.units else int(MxArgUnit.MX_UNIT_NONE),
            MX_JSON_KEY_VAR_DEFVAL: self.default,
            MX_JSON_KEY_VAR_MIN: self.min,
            MX_JSON_KEY_VAR_MAX: self.max,
        }

    @classmethod
    def from_dict(cls, arg_def: dict) -> RegisteredArg:
        """ Fill this class members from a dictionary (generally parsed from a JSON received from the robot) """
        return RegisteredArg(
            name=str(arg_def.get(MX_JSON_KEY_VAR_NAME, "")),
            description=str(arg_def.get(MX_JSON_KEY_VAR_DESC, "")),
            type=RegisteredArg._parse_type(arg_def.get(MX_JSON_KEY_VAR_TYPE, int(MxArgType.MX_ARG_TYPE_ANY))),
            length=arg_def.get(MX_JSON_KEY_VAR_ARR_LEN, None),
            units=arg_def.get(MX_JSON_KEY_VAR_UNIT, None),
            default=arg_def.get(MX_JSON_KEY_VAR_DEFVAL, None),
            min=arg_def.get(MX_JSON_KEY_VAR_MIN, None),
            max=arg_def.get(MX_JSON_KEY_VAR_MAX, None),
        )

    @classmethod
    #pylint: disable=redefined-builtin
    def _parse_type(cls, type: Union[str, int]) -> MxArgType:
        """ Parse argument type, which can either be enum value (int) or string (from robot command dict) """
        if isinstance(type, int):
            return MxArgType(type)
        elif type == "int32" or type == "uint32":
            return MxArgType.MX_ARG_TYPE_INTEGER
        elif type == "double":
            return MxArgType.MX_ARG_TYPE_FLOAT
        elif type == "string":
            return MxArgType.MX_ARG_TYPE_STRING
        else:
            return MxArgType.MX_ARG_TYPE_ANY


class RegisteredCommand:
    """ Class to store information of a registered command from a user MecaScript """

    def __init__(self,
                 function: Callable,
                 name: Optional[str] = None,
                 description: str = "",
                 args: Optional[list[RegisteredArg]] = None,
                 validate_args: bool = True,
                 tags: Optional[list[str]] = None,
                 cyclic_id: Optional[int] = None,
                 command_type: Optional[MxRegisteredCmdType] = MxRegisteredCmdType.MX_REGISTERED_CMD_TYPE_INLINE,
                 private: bool = False):
        """Constructor for this class

        Parameters
        ----------
        function
            Function to register
        name
            Name of the command to use in the namespaces (and on the robot's command dictionary).
            If None, then the command name is deduced from the provided function pointer.
        description
            Short description that will show up in MecaPortal code editor
        args
            Description of arguments for this command.
            This is used for user help/documentation in the MecaPortal code editor.
            It can also be used for arguments validation (if validate_args is True) to avoid the command to be
            called with unexpected/incorrect arguments (the robot will refuse the command, it won't be called).
            Note that if validate_args is False, the command will receive any passed arguments, whether they
            correspond or not to args provided here.
        validate_args
            Validate that the argument match 'args' above before calling this command.
            The command will not be called if the arguments don't match the description.
        tags
            Tags that identify which type of robot command this is.
            The tags are used to classify the command in MecaPortal's code editor context menus.
        cyclic_id
            Optional Id that can be used in cyclic protocols to call this
            command. This is a value between 30000 an 39999 (or None).
            Note that, when called from cyclic protocols, the command can normally only receive float arguments,
            and up to 6 of them.
        private
            Indicate that this registered command should not be included in robot's commands
            dictionary. It will thus not be known by the MecaPortal's code editor and not advertize to users
            (no auto-complete suggestion, not visible in context menus).
        """
        self.function = function
        self.file_path: str = ""
        self.name: str = function.__name__ if name is None or name == '' else name
        self.description = description
        self.tags = tags
        self.args = args
        self.validate_args = validate_args
        self.cyclic_id = cyclic_id
        self.mecascript_type = MecaScriptType.PY
        self.command_type: Optional[MxRegisteredCmdType] = command_type
        self.private = private
        # Other members
        self.locally_created = True  # Tells if this command is locally created, else received from the robot
        self.is_override = False  # Tells if this local command is an override of an existing robot command

    def __str__(self) -> str:
        """ Returns string representation """
        return self.name

    def __repr__(self) -> str:
        return str(self)

    def to_dict(self) -> dict[str, Any]:
        """ Returns a dictionary string representation of this object, ready to be serialized as JSON for the robot"""
        return {
            MX_JSON_KEY_SIDECAR_CMD_NAME: self.name,
            MX_JSON_KEY_SIDECAR_CMD_FILE_PATH: self.file_path,
            MX_JSON_KEY_SIDECAR_CMD_DESC: self.description,
            MX_JSON_KEY_SIDECAR_CMD_ARGS: [arg.to_dict() for arg in self.args] if self.args else [],
            MX_JSON_KEY_SIDECAR_CMD_VALIDATE_ARGS: self.validate_args,
            MX_JSON_KEY_SIDECAR_CMD_TAGS: self.tags,
            MX_JSON_KEY_SIDECAR_CMD_CYCLIC_ID: self.cyclic_id,
            MX_JSON_KEY_SIDECAR_CMD_PRIVATE: self.private,
            MX_JSON_KEY_SIDECAR_CMD_OVERRIDE: self.is_override,
            MX_JSON_KEY_SIDECAR_CMD_TYPE: int(self.command_type)
        }

    @classmethod
    def from_robot_cmd_def(cls, function: Callable, name: str, cmd_def: dict) -> RegisteredCommand:
        """Build a RegisteredCommand object from a command definition received from the robot
        as a JSON string and parsed as a dictionary passed as argument to this function.
        This function is called, for example, when receiving a notification from the robot indicating that a sidecar
        module has registered a new command.

        Parameters
        ----------
        function
            The function to call if this command is called.
            Typically, this function will send a message to the robot telling to call this
            registered command with the provided arguments)
        name
            Name of the command (may include prefixes, like my_module.my_command)
        cmd_def
            Definition of the command (description, tags, arguments, etc.)

        Returns:
            RegisteredCommand
                The RegisteredCommand object rebuilt from the provided cmd_def dictionary
        """
        registered_command = RegisteredCommand(
            function=function,
            name=name,
            description=cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_DESC, ""),
            args=[RegisteredArg.from_dict(arg_dict) for arg_dict in cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_ARGS, [])],
            validate_args=bool(cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_VALIDATE_ARGS, True)),
            tags=[str(tag) for tag in cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_TAGS, [])],
            cyclic_id=cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_CYCLIC_ID, None),
            command_type=cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_TYPE, None),
            private=bool(cmd_def.get(MX_JSON_KEY_SIDECAR_CMD_PRIVATE, False)))

        # Remember that this command was not locally registered (it comes from the robot's command dictionary)
        registered_command.locally_created = False
        return registered_command


class RegisteredVariable(RegisteredArg):
    """ Class to store information of a robot variable registered in the local namespace """

    def __init__(
            self,
            name: str,
            description: str = "",
            #pylint: disable=redefined-builtin
            type: MxArgType = MxArgType.MX_ARG_TYPE_ANY,
            length: Optional[int] = None,
            units: Optional[MxArgUnit] = None,
            volatile: Optional[bool] = False,
            cyclic_id: Optional[int] = None,
            default: Optional[Any] = None,
            min: Optional[Any] = None,
            max: Optional[Any] = None):
        """ Constructor for this class
            See RegisteredArg for arguments description.
        """
        super().__init__(name, description, type, length, units, default, min, max)
        self.volatile = volatile
        self.cyclic_id = cyclic_id
        self._value = default

    def copy(self) -> RegisteredVariable:
        """Return a fully detached snapshot of this variable."""

        new_var = RegisteredVariable(
            name=self.name,
            description=self.description,
            type=self.type,
            length=self.length,
            units=self.units,
            volatile=self.volatile,
            cyclic_id=self.cyclic_id,
            default=None,
            min=self.min,
            max=self.max,
        )

        # Detach value completely
        new_var._value = copy.deepcopy(_unwrap_mutable(self._value))  # pylint: disable=protected-access

        return new_var

    def set_value(self, value: Any) -> Any:
        """Set the variable's value. This function will attempt to "cast" the value into the expected type.

        Parameters
        ----------
        value
            Value to set

        Raises
        ------
            ValueError
                Cannot set value (array of incorrect length for example)

        Returns
        -------
        Any
            The previous value of the variable before this function call
        """
        old_value = self._value
        self._value = value
        # Return unwrapped value to avoid the callback to be attached to the old value
        return _unwrap_mutable(old_value)

    def get_value(self) -> Any:
        """ Get the current variable value """
        return self._value

    #pylint: disable=invalid-name
    def cast_value(self, value: Any):
        """ Cast a value into the expected type for this variable """
        if self.type == MxArgType.MX_ARG_TYPE_INTEGER:
            return int(value)
        elif self.type == MxArgType.MX_ARG_TYPE_FLOAT:
            return float(value)
        elif self.type == MxArgType.MX_ARG_TYPE_STRING:
            return str(value)
        else:
            return value

    def __str__(self) -> str:
        """ Returns string representation """
        if self.length is None:
            return f'{self._value}'
        else:
            if isinstance(self._value, (list, _ListProxy)):
                return f'[{", ".join([str(x) for x in self._value])}]'
            else:
                return f'[{self._value}]'

    def __repr__(self) -> str:
        return str(self)


def _wrap_mutable(value: Any, on_change: Callable) -> Any:
    """ This function is used to determine which robot variable values need to be wrapped into proxy classes
    so that changes to values are detected and trigger an update on the robot too"""
    if isinstance(value, list):
        return _ListProxy(value, on_change)
    if isinstance(value, dict):
        return _DictProxy(value, on_change)
    return value


def _unwrap_mutable(value: Any) -> Any:
    """ This function is used to unwrap proxy classes used for robot variable values back into
        standard Python types (list, dict, etc.) before sending them to the robot"""
    if isinstance(value, _ListProxy):
        return value._data  # pylint: disable=protected-access
    if isinstance(value, _DictProxy):
        return value._data  # pylint: disable=protected-access
    return value


class _ListProxy(MutableSequence):
    """ This class is used to wrap list values of robot variables so that any change to the list items calls
        the on_change callback to update the robot variable value too."""

    def __init__(self, data: Any, on_change: Callable):
        self._data = data
        self._on_change = on_change

    def __len__(self):
        return len(self._data)

    def __getitem__(self, index: int):
        value = self._data[index]
        return _wrap_mutable(value, on_change=lambda _: self._on_change(self._data))

    def __setitem__(self, index: int, value: Any):
        self._data[index] = value
        self._on_change(self._data)

    def __delitem__(self, index: int):
        del self._data[index]
        self._on_change(self._data)

    def copy(self):
        """Return a detached shallow copy as a plain list so changes to that copy don't affect the robot."""
        # Note: Forcing deepcopy as it would not be correct if caller is modifying a nested element, causing a
        # modification in the robot class, but no on_change callback causing the value in the robot class to be
        # different than on the actual robot, which is inconsistent and what now we want.
        return copy.deepcopy(self._data)

    def __copy__(self):
        """Return a detached shallow copy as a plain list so changes to that copy don't affect the robot."""
        # Note: Forcing deepcopy as it would not be correct if caller is modifying a nested element, causing a
        # modification in the robot class, but no on_change callback causing the value in the robot class to be
        # different than on the actual robot, which is inconsistent and what now we want.
        return copy.deepcopy(self._data)

    def __deepcopy__(self, memo):
        """Return a detached shallow copy as a plain list so changes to that copy don't affect the robot."""
        return copy.deepcopy(self._data, memo)

    def insert(self, index: int, value: Any):
        self._data.insert(index, value)
        self._on_change(self._data)

    def append(self, value: Any):
        self._data.append(value)
        self._on_change(self._data)

    def extend(self, values: list[Any]):
        self._data.extend(values)
        self._on_change(self._data)

    def pop(self, index=-1):
        value = self._data.pop(index)
        self._on_change(self._data)
        return value

    def remove(self, value: Any):
        self._data.remove(value)
        self._on_change(self._data)

    def clear(self):
        self._data.clear()
        self._on_change(self._data)

    def __iter__(self):
        for item in self._data:
            yield _wrap_mutable(item, self._on_change)

    def __repr__(self):
        return repr(self._data)

    def __eq__(self, other):
        if isinstance(other, _ListProxy):
            return self._data == other._data
        if isinstance(other, list):
            return self._data == other
        return NotImplemented

    def __ne__(self, other):
        eq = self.__eq__(other)
        if eq is NotImplemented:
            return NotImplemented
        return not eq


class _DictProxy(MutableMapping):
    """ This class is used to wrap dict values of robot variables so that any change to the dict items calls
        the on_change callback to update the robot variable value too."""

    def __init__(self, data: Any, on_change: Callable):
        self._data = data
        self._on_change = on_change

    def __len__(self):
        return len(self._data)

    def __getitem__(self, key):
        value = self._data[key]
        return _wrap_mutable(value, on_change=lambda _: self._on_change(self._data))

    def __setitem__(self, key, value: Any):
        self._data[key] = value
        self._on_change(self._data)

    def __delitem__(self, key):
        del self._data[key]
        self._on_change(self._data)

    def copy(self):
        """Return a detached shallow copy as a plain dict so changes to that copy don't affect the robot."""
        # Note: Forcing deepcopy as it would not be correct if caller is modifying a nested element, causing a
        # modification in the robot class, but no on_change callback causing the value in the robot class to be
        # different than on the actual robot, which is inconsistent and what now we want.
        return copy.deepcopy(self._data)

    def __copy__(self):
        """Return a detached shallow copy as a plain dict so changes to that copy don't affect the robot."""
        # Note: Forcing deepcopy as it would not be correct if caller is modifying a nested element, causing a
        # modification in the robot class, but no on_change callback causing the value in the robot class to be
        # different than on the actual robot, which is inconsistent and what now we want.
        return copy.deepcopy(self._data)

    def __deepcopy__(self, memo):
        """Return a detached shallow copy as a plain dict so changes to that copy don't affect the robot."""
        return copy.deepcopy(self._data, memo)

    def __iter__(self):
        return iter(self._data)

    # pylint: disable=arguments-differ
    def update(self, other=(), **kwds):
        self._data.update(other, **kwds)
        self._on_change(self._data)

    def pop(self, key, default=None):
        value = self._data.pop(key, default)
        self._on_change(self._data)
        return value

    def clear(self):
        self._data.clear()
        self._on_change(self._data)

    def __repr__(self):
        return repr(self._data)

    def __eq__(self, other):
        if isinstance(other, _DictProxy):
            return self._data == other._data
        if isinstance(other, dict):
            return self._data == other
        return NotImplemented


class AttributeName:
    """ This class is a simple container that is used by AttributeContainer to store the full name (with prefixes)
        of a registered attribute """

    def __init__(self, name: str):
        self.name = name


class AttributeContainer:
    """ This class is used to recursively store attributes (variables or commands) by path into object and sub-object,
    allowing code to later access attributes with code like:

        Calling a registered command:
            robot.my_group.my_sub_group.my_command(my_args)
        Read or write a registered variable
            robot.my_group.my_subgroup.my_variable += 2
            print(f'robot.my_group.my_subgroup.my_variable')

    This class supports saving the attributes in a local dictionary, or else using callback functions at run-time
    to get/set registered attribute values dynamically (without actually storing the values in this class instance).

    This class also supports a "callable_mode" where operator() (__call__) is overloaded to call the function,
    which allows to register leaf objects as AttributeContainer, which can be called.
    This allows a folder to have the same name as a function, for example ("test/..." and "test.mxprog").
    """

    def __init__(self, callable_mode: bool):
        """Constructor

        Parameters
        ----------
        callable_mode
            When "callable" mode is used, leaf attributes are created as AttributeContainer
            objects with _callable_attribute set. See explanation above.
        """
        #pylint: disable=invalid-name
        # Make sure that __setattr__ and __getattr__ skip searching self._attributes during initialization
        self.__dict__['_initialized'] = False
        self._get_attribute_callback: Callable = None
        self._set_attribute_callback: Callable = None
        self._attributes: dict[str, Union[str, AttributeContainer]] = {}
        self._callable_mode = callable_mode
        self._callable_attribute: Optional[Union[AttributeName, Callable]] = None  # Add this line

        # Mark initialization as complete. From now-on, __setattr__ and __getattr__ will search in self._attributes
        # for any attribute not found in self.__dict__
        self._initialized = True

    def attach(self, get_attribute_callback: Callable[[str]], set_attribute_callback: Callable[[str, Any]]):
        """Attach the callback functions that will be used to actually get and set the attributes when this
            class is accessed

        Parameters
        ----------
        get_attribute_callback
            Function to call (with attribute full name as argument) to retrieve an
            attribute that was registered in this container.
        set_attribute_callback
            Function to call (with attribute full name as argument and value) to
            change the value of an attribute that was registered in this container.
        """
        self._get_attribute_callback = get_attribute_callback
        self._set_attribute_callback = set_attribute_callback

    def detach(self):
        """ Detach previously attached get/set callbacks """
        self._get_attribute_callback = None
        self._set_attribute_callback = None

    def register_attribute(self, attribute_name: str, attribute_full_name: str, attribute: Optional[Any] = None):
        """Recursively registers an attribute, creating nested containers as needed.

        This method builds a hierarchical structure. For example, registering an
        `attribute_name` of 'folder.program' will create a nested `AttributeContainer`
        named 'folder' that holds the 'program' attribute.

        If self._callable_mode is True, this the leaf attribute will be an AttributeContainer object with
        _callable_mode, allowing overriding the () operator.
        Otherwise the leaf object is not a container (is the provided object directly or an AttributeName if
        "attach" has been used to retrieve attribute by name)

        Parameters
        ----------
        attribute_name
            The name of the attribute to register. May contain dots ('.') to indicate nesting.
        attribute_full_name
            The complete, unique name for the attribute, which is passed to the attached get/set callbacks.
        attribute
            The value for the attribute. It can be None if "attach" has been called on this
            object, in which case the attribute value is fetched at run time by name with attached callbacks.
        """
        if '.' in attribute_name:
            # Nested attribute. Recurse into a sub-object.
            prefix, sub_name = attribute_name.split('.', 1)

            # Ensure the prefix points to a container, creating it if necessary.
            if prefix not in self._attributes:
                self._attributes[prefix] = self.__class__(self._callable_mode)
                self._attributes[prefix].attach(self._get_attribute_callback, self._set_attribute_callback)
            elif not isinstance(self._attributes[prefix], AttributeContainer):
                # if this happens, it means we have registered leaf objects that are not callable AttributeContainer
                # objects, in which case it's not permitted to have a leaf object with the same "path" as another
                # sub-nested attribute (ex: Cannot have 'attr.sub' and 'attr' both registered)
                raise TypeError(f"Attribute '{prefix}' is already registered as a non-container.")

            self._attributes[prefix].register_attribute(sub_name, attribute_full_name, attribute)
        else:
            leaf_name = attribute_name
            final_attribute = None
            if attribute is None:
                # We'll retrieve attribute with self._get_attribute_callback when necessary
                final_attribute = AttributeName(attribute_full_name)
            else:
                final_attribute = attribute
            if self._callable_mode:
                # In "callable" mode, the leaf object is an AttributeContainer into which we save the
                # attribute as _callable_attribute. This allows creating sub-objects at the same time as
                # being able to call this leaf. For example, having
                #   attr()
                #   and at the same time
                #   attr.sub_attr_1()
                #   attr.sub_attr_2()
                if leaf_name not in self._attributes:
                    # Create new attribute container node
                    self._attributes[leaf_name] = self.__class__(self._callable_mode)
                    self._attributes[leaf_name].attach(self._get_attribute_callback, self._set_attribute_callback)

                # Set the callable name on the container.
                self._attributes[leaf_name]._callable_attribute = final_attribute  #pylint: disable=protected-access
            else:
                self._attributes[attribute_name] = final_attribute

    def unregister_attribute(self, attribute_name: str):
        """Unregister a previously registered attribute

        If the attribute represents both a callable command and a container
        for sub-attributes, this method will first remove its callable status.
        The container itself is only deleted if it no longer holds any children.

        Parameters
        ----------
        attribute_name
            The name of the attribute to unregister from this container
        """
        if '.' in attribute_name:
            # Nested attribute: recurse into the sub-container.
            prefix, sub_name = attribute_name.split('.', 1)
            if prefix in self._attributes and isinstance(self._attributes[prefix], AttributeContainer):
                sub_container = self._attributes[prefix]
                sub_container.unregister_attribute(sub_name)

                # After recursing, prune the sub-container if it's now fully empty.
                if sub_container._callable_attribute is None and sub_container.empty():  #pylint: disable=protected-access
                    del self._attributes[prefix]
        else:
            # Leaf attribute, which we're trying to unregister.
            if attribute_name in self._attributes:
                attribute = self._attributes[attribute_name]

                if isinstance(attribute, AttributeContainer):
                    # The attribute is a container (folder/callable program).
                    # First, remove its callable status.
                    attribute._callable_attribute = None  #pylint: disable=protected-access

                    # Only delete the container itself if it has no children.
                    if attribute.empty():
                        del self._attributes[attribute_name]
                else:
                    # The attribute is a simple value (not a container).
                    # It can be safely deleted directly.
                    del self._attributes[attribute_name]

    def list(self) -> list[str]:
        """ Get the list of attributes registered to this container (some may be sub-containers) """
        return list(self._attributes.keys())

    def get(self, attribute_name: str) -> Optional[Any]:
        """Get the specified attribute by name, or return None if not found.
           This is similar to accessing the container attribute directly (ex: container.my_attr) except that:
           - here we use a recursive string to identify the attribute (attr_name.sub_name...)
           - Here we return None if the attribute is not found instead of raising an exception.

        Parameters
        ----------
        attribute_name
            Name of the attribute to get (including prefixes if appropriate)

        Returns
        -------
        Any
            The found attribute value or None
        """
        if '.' in attribute_name:
            # Nested attribute. Let's recurse into a sub-object
            prefix, sub_name = attribute_name.split('.', 1)
            if prefix not in self._attributes:
                return None
            return self._attributes[prefix].get(sub_name)
        else:
            # Not nested, get the attribute locally
            if attribute_name not in self._attributes:
                return None
            return getattr(self, attribute_name)

    def set(self, attribute_name: str, value: Any) -> Optional[Any]:
        """Set the specified attribute by name, or raise NotFoundException if not found.
           This is similar to accessing the container attribute directly (ex: container.my_attr=...) except that
           here we use a recursive string to identify the attribute (attr_name.sub_name...)

           This is a blocking function that awaits for robot response to confirm success or failure.

        Parameters
        ----------
        attribute_name
            Name of the attribute to get (including prefixes if appropriate)
        value
            The new value to set for this attribute

        Raises
        ------
            NotFoundException
                The variable with specified name does not exist

        Returns
        -------
        Any
            The previous variable value.
            Note that if the function returns None, it means that the variable existed and had a value None
            (if the variable does not exist, this function raises NotFoundException)

        """
        if '.' in attribute_name:
            # Nested attribute. Let's recurse into a sub-object
            prefix, sub_name = attribute_name.split('.', 1)
            if prefix not in self._attributes:
                raise NotFoundException(f'Cannot set {attribute_name}, does not exist')
            return self._attributes[prefix].set(sub_name, value)
        else:
            # Not nested, set the attribute locally
            if attribute_name not in self._attributes:
                raise NotFoundException(f'Cannot set {attribute_name}, does not exist')
            prev_val = copy.deepcopy(getattr(self, attribute_name))
            setattr(self, attribute_name, value)
            return prev_val

    def empty(self) -> bool:
        """ Tells if there are attributes or sub-groups in this instance """
        return len(self._attributes) == 0 and self._callable_attribute is None

    def __call__(self, *args, **kwargs):
        """ Overloading the Python's __call__ method allows to intercept the case where the user is treating the
            registered attribute as a function, like in: robot.programs.folder.my_func().
            This would end-up calling the __call__ method of the leaf AttributeContainer object ("my_func").
            Here we retrieve the actual function to call using the attached _get_attribute_callback method.
            """
        if self._callable_attribute is None:
            raise TypeError(f"'{self.__class__.__name__}' object is not registered as a callable, it cannot be called")

        # The get_attribute_callback is expected to return the function that sends the robot command.
        if isinstance(self._callable_attribute, AttributeName):
            if self._get_attribute_callback is None:
                raise ModuleNotFoundError(
                    "Attribute container has not been attached to a callback to execute commands.")
            program_func = self._get_attribute_callback(self._callable_attribute)
        elif isinstance(self._callable_attribute, Callable):
            # Assume it's a Callable
            program_func = self._callable_attribute
        else:
            raise TypeError(f"'{self.__class__.__name__}' registered object is not a callable or attribute name")
        return program_func(*args, **kwargs)

    def __getattr__(self, attr_name: str) -> Any:
        """We override this to provide access to registered attributes as class attributes

        Parameters
        ----------
        attr_name
            The name of the attribute to get (no prefix, attribute name only)

        Raises
        ------
            AttributeError
                No attribute is registered with this name
            ModuleNotFoundError
                No callback function attached, cannot retrieve the attribute value from parent module

        Returns
        -------
        Any
            The retrieved attribute value
        """
        # Other attribute (not explicit members of this class): Check if exists in our registered attributes dictionary
        if attr_name not in self._attributes:
            raise AttributeError(f'Trying to get non-registered attribute {attr_name}')

        if isinstance(self._attributes[attr_name], AttributeContainer):
            # Nested attribute, return the nested container
            return self._attributes[attr_name]

        # Check if we have saved the attribute itself, or just its name and must use the "get" callback
        if isinstance(self._attributes[attr_name], AttributeName):
            # Return the attribute value (by using our "get" callback)
            if self._get_attribute_callback is None:
                raise ModuleNotFoundError(f'Attributes container has not been attached to get/set callbacks')
            attribute_full_name: str = self._attributes[attr_name].name
            return self._get_attribute_callback(attribute_full_name)
        else:
            # Return the attribute value directly
            return self._attributes[attr_name]

    def __setattr__(self, attr_name: str, value: Any):
        """We override this to provide access to registered attributes as class attributes

        Parameters
        ----------
        attr_name
            The name of the attribute to set (no prefix, attribute name only)
        value
            Value to set.

        Raises
        ------
            ValueError
                Trying to assign a value into an already registered sub-group
            AttributeError
                No attribute with this name was registered
            ModuleNotFoundError
                No callback function attached, cannot set the attribute value into parent module
        """
        # Make sure we can still access member variables of this class
        if not self.__dict__.get('_initialized') or attr_name in self.__dict__:
            super().__setattr__(attr_name, value)
            return

        # Make sure that this attribute is registered
        if attr_name not in self._attributes:
            raise AttributeError(f'Trying to set non-registered attribute {attr_name}')

        if isinstance(self._attributes[attr_name], AttributeContainer):
            # Nested attribute, should not happen until someone is trying to assign a value name that conflicts with
            # a sub-container name that was previously registered
            # For example g_attributes.my_group = 2
            # while we already have sub-attributes nested under sub-container "my_group"
            raise ValueError(f'Trying to assign a value to attribute group {attr_name}')

        # Set the attribute value (by using our "set" callback)
        if isinstance(self._attributes[attr_name], AttributeName):
            if self._set_attribute_callback is None:
                raise ModuleNotFoundError(f'Attributes container has not been attached to get/set callbacks')
            attribute_full_name: str = self._attributes[attr_name].name
            return self._set_attribute_callback(attribute_full_name, value)
        else:
            self._attributes[attr_name] = value
            return value


class VariablesContainer(AttributeContainer):
    """ This class is a specialization of AttributeContainer used to store variable values """

    def __init__(self, callable_mode=False):
        super().__init__(callable_mode=callable_mode)

        # Note: Since we derive from AttributeContainer which overrides __setattr__, we need to clear _initialized
        # while we're creating new class members
        self._initialized = False
        self._getting_reg_var = False
        self._initialized = True

    def get(self, attribute_name: str) -> Optional[RegisteredVariable]:
        """ Returns the RegisteredVariable object by name """
        # This  will call __getattr__ which we want to return the RegisteredVariable object, not it's value
        self._getting_reg_var = True
        var = super().get(attribute_name)
        self._getting_reg_var = False
        return var

    def __getattr__(self, attr_name: str) -> Any:
        """ See base class doc. Here we just return the value from the registered variable.
        This allows code that do robot.variables.myvar  to get the value directly, while robot.variables.get("myvar")
        returns a RegisteredVariable object
        """
        reg_var = super().__getattr__(attr_name)
        if isinstance(reg_var, RegisteredVariable) and not self._getting_reg_var:
            return reg_var.get_value()
        return reg_var


class ScriptUtils:
    """ This class contains utility functions for basic .mxpy programs """

    @classmethod
    def get_arg(cls,
                args: list[Any],
                kwargs: dict[str, Any],
                index: Optional[int] = None,
                name: Optional[str] = None,
                default: Optional[Any] = None) -> Any:
        """
        This is a utility function for user programs, in particular .mxpy programs.
        They have access to args and kwargs, but using 'get_arg' by index and/or keyword is convenient.
        Gets an argument from args or kwargs.

        This function will first attempt to retrieve the argument by name from kwargs.
        If not found, will try to retrieve the positional argument (by index) from args.

        Parameters
        ----------
        *args
            Positional arguments.
        **kwargs
            Keyword arguments.
        index
            Index of the argument to retrieve from args.
        name
            Name of the argument to retrieve from kwargs.
        default
            Default value to return if the argument is not found.

        Returns
        -------
        Any
            The retrieved argument or the default value.
        """
        # Try to get by argument name
        if name is not None and name in kwargs:
            return kwargs.get(name, default)

        # Try to get by argument index
        if index is not None and index < len(args):
            return args[index] if index < len(args) else default

        # Return default
        return default

    @classmethod
    def max_nb_positional_args(cls, func):
        """ Returns the maximum number of positional arguments that a function can take"""
        sig = inspect.signature(func)
        max_args = 0
        for parameter in sig.parameters.values():
            if parameter.kind in (parameter.POSITIONAL_ONLY, parameter.POSITIONAL_OR_KEYWORD):
                max_args += 1
            elif parameter.kind == parameter.VAR_POSITIONAL:  # *args
                return float('inf')  # unlimited
        return max_args


def _register_nested_function(function_full_name: str, get_parent_attr: Callable[[str]],
                              set_parent_attr: Callable[[str, Any]], function: Callable):
    """This is a helper method to register a nested function into a namespace.
    This function will take care to recursively create intermediate containers in case the function name contains
    prefixes (like my_group.my_sub_group.my_function)

    Parameters
    ----------
    function_full_name
        Name of the function to register (which may have a prefix)
    get_parent_attr
        Callback function to get by name from the desired namespace
    set_parent_attr
        Callback function to set by name into the desired namespace
    function
        The function to register
    """
    if '.' in function_full_name:
        prefix, sub_name = function_full_name.split('.', 1)
        # Must register inside a nested attribute. Let's first check if this sub-container already exists
        attr_container: AttributeContainer = get_parent_attr(prefix)
        if attr_container is None:
            # Create a new attribute container
            attr_container = AttributeContainer(callable_mode=True)
            # Register this attribute container in the parent namespace
            set_parent_attr(prefix, attr_container)

        # Recursive registering inside the attribute container
        attr_container.register_attribute(sub_name, function_full_name, function)

    else:
        # Not nested, directly register the function in the parent namespace
        set_parent_attr(function_full_name, function)


def _unregister_nested_function(function_full_name: str, get_parent_attr: Callable[[str]],
                                del_parent_attr: Callable[[str]]):
    """This is a helper method to recursively unregister a function from a namespace.

    Parameters
    ----------
    function_full_name
        Name of the attribute to unregister (which may have a prefix)
    get_parent_attr
        Callback function to get by name (or prefix) from the desired namespace
    del_parent_attr
        Callback function to delete by name (or prefix) from the desired namespace
    """
    if '.' in function_full_name:
        # Nested attribute. Let's delete from nested container object
        prefix, sub_name = function_full_name.split('.', 1)

        # Retrieve the attribute container that we previously registered in the parent namespace
        attr_container: AttributeContainer = get_parent_attr(prefix)
        if attr_container is not None:
            if isinstance(attr_container, AttributeContainer):
                # Recursively unregister the sub-function name from the attribute container
                attr_container.unregister_attribute(sub_name)
                # Note: Do not delete the prefix object from the root parent container because programs will keep
                #       references to that root object. And later if we recreate a new root object, the programs will
                #       continue (until they are reloaded) to point to the old (empty) root object and would not longer
                #       be able to call re-registered functions.
                #
                #       So we keep ('leak') this root object forever in case functions are re-registered in it later.
                #
                #       Note that it's correct to delete sub-attributes because programs will go through root object's
                #       __getattr__ overload when accessing child attributes so it's correct to delete/recreate child
                #       attributes within this root object or sub-objects. We just can't delete the root object.
                # if attr_container.empty():
                #     del_parent_attr(prefix)
            else:
                # Oops... seems like the function name prefix was never registered as an attribute container.
                # This is most likely a mismatch between the function names passed in register vs unregister calls.
                raise ValueError(f'Found attribute {prefix} in parent namespace but it is not an AttributeContainer')
    else:
        # Not nested, directly delete the attribute within the parent namespace
        del_parent_attr(function_full_name)


class SidecarInternalStatus:
    """ This class contains the internal sidecar status reported to robot """

    def __init__(self):
        """ Constructor for internal sidecar status """
        self.loaded_programs_status: dict[str, dict[str, MxProgramStatus]] = {}

    def to_dict(self) -> dict[str, Any]:
        """ Returns a dictionary string representation of this object, ready to be serialized as JSON for the robot"""

        return {MX_JSON_KEY_SIDECAR_STATUS_PROGRAMS_STATUS: self.loaded_programs_status}
