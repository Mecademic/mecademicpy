"""
This file define classes used for managing sidecar modules, scripts and registered functions.
"""
from __future__ import annotations

import copy
from typing import Callable, Optional, Union

#pylint: disable=wildcard-import,unused-wildcard-import
from mecademicpy.mx_robot_def import *
from mecademicpy.robot_classes import NotFoundException


class SidecarScriptType(IntEnum):
    MXPY = 0  # Simplified .mxpy script
    PY = 1  # Complete python file (.py)


class RegisteredFunctionType(IntEnum):
    NON_BLOCKING = 0  # Non-blocking script that quickly pushes robot instructions and terminates
    BLOCKING_NON_EXCLUSIVE = 1  # Blocking script, non-exclusive (API commands are postponed while it runs)
    BLOCKING_EXCLUSIVE = 1  # Blocking script, exclusive (API commands are refused while it runs)


class RegisteredArg:
    """ Class that describes one argument for RegisteredFunction """

    def __init__(
        self,
        name: str,
        description: str = "",
        #pylint: disable=redefined-builtin
        type: MxArgType = MxArgType.MX_ARG_TYPE_ANY,
        length: Optional[int] = None,
        units: Optional[MxArgUnit] = None,
        default: Optional[any] = None,
        min: Optional[any] = None,
        max: Optional[any] = None,
    ):
        """ Mandatory constructor for this object

        Args:
            name (str): Name of this argument/variable
            description (str, optional): Description of this argument for the users. Defaults to "".
            type (MxArgType, optional): Type of this argument. Defaults to MxArgType.MX_ARG_TYPE_ANY.
            length (Optional[int], optional): Length of this array (0 if single argument, not array). Defaults to None.
            units (Optional[MxArgUnit], optional): Units of this argument (for documentation only). Defaults to None.
            default (Optional[any], optional): Default value for this argument (for documentation). Defaults to None.
            min (Optional[any], optional): Minimum value of this argument (for documentation only). Defaults to None.
            max (Optional[any], optional): Maximum value of this argument (for documentation only). Defaults to None.
        """
        self.name = name
        self.description = description
        self.type = type
        self.length = length
        self.units = units
        self.default = default
        self.min = min
        self.max = max

    def __str__(self) -> str:
        """ Returns string representation """
        return self.name

    def __repr__(self) -> str:
        return str(self)

    def to_dict(self) -> str:
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


class RegisteredFunction:
    """ Class to store information of a registered function from a user sidecar module """

    def __init__(self,
                 function: callable,
                 name: str,
                 description: str = "",
                 args: Optional[list[RegisteredArg]] = None,
                 validate_args: bool = True,
                 tags: Optional[list[str]] = None,
                 cyclic_id: Optional[int] = None,
                 script_type=SidecarScriptType.PY,
                 function_type=RegisteredFunctionType.NON_BLOCKING,
                 private: bool = False):
        """Constructor for this class

        Args:
            function (callable): Function to register
            name (str): Name of the function to use in the namespaces (and on the robot's command dictionary)
            description (str): Short description that will show up in MecaPortal code editor
            args (Optional[list[any]], optional): Description of arguments for this function.
                This is used for user help/documentation in the MecaPortal code editor.
                It can also be used for arguments validation (if validate_args is True) to avoid the function to be
                called with unexpected/incorrect arguments (the robot will refuse the command, it won't be called).
                Note that if validate_args is False, the function will receive any passed arguments, whether they
                correspond or not to args provided here.
                Defaults to None.
            validate_args (bool, optional): Validate that the argument match 'args' above before calling this function.
                The function will not be called if the arguments don't match the description.
                Defaults to True.
            tags (Optional[list[str]], optional): Tags that identify which type of robot command this function
                implements. The tags are used to classify the function in MecaPortal's code editor context menus.
                Defaults to None.
            cyclic_id (Optional[int], optional): Optional Id that can be used in cyclic protocols to call this
                function.
                Note that, when called from cyclic protocols, the function can normally only receive float arguments,
                and up to 6 of them.
                Defaults to None.
            private (bool, optional): Indicate that this registered function should not be included in robot's commands
                dictionary. It will thus not be known by the MecaPortal's code editor and not advertize to users
                (no auto-complete suggestion, not visible in context menus).
                Defaults to False.
        """
        # Parameters
        self.function: str = function
        self.module_name: str = ""
        self.name: str = name
        self.description = description
        self.tags = tags
        self.args = args
        self.validate_args = validate_args
        self.cyclic_id = cyclic_id
        self.script_type = script_type
        self.function_type = function_type
        self.private = private
        # Other members
        self.locally_created = True  # Tells if this function is locally created, else received from the robot
        self.is_override = False  # Tells if this local function is an override of an existing robot function

    def __str__(self) -> str:
        """ Returns string representation """
        return self.name

    def __repr__(self) -> str:
        return str(self)

    def get_full_name(self) -> str:
        """ Get the full name (i.e.'function:module.name')"""
        return f'function:{self.module_name}.{self.name}'

    def to_dict(self) -> str:
        """ Returns a dictionary string representation of this object, ready to be serialized as JSON for the robot"""
        return {
            MX_JSON_KEY_SIDECAR_FCT_NAME: self.name,
            MX_JSON_KEY_SIDECAR_FCT_DESC: self.description,
            MX_JSON_KEY_SIDECAR_FCT_ARGS: [arg.to_dict() for arg in self.args] if self.args else [],
            MX_JSON_KEY_SIDECAR_FCT_VALIDATE_ARGS: self.validate_args,
            MX_JSON_KEY_SIDECAR_FCT_TAGS: self.tags,
            MX_JSON_KEY_SIDECAR_FCT_CYCLIC_ID: self.cyclic_id,
            MX_JSON_KEY_SIDECAR_FCT_PRIVATE: self.private,
            MX_JSON_KEY_SIDECAR_FCT_OVERRIDE: self.is_override
        }

    @classmethod
    def from_robot_cmd_def(cls, function: callable, name: str, cmd_def: dict) -> RegisteredFunction:
        """Build a RegisteredFunction object from a command definition received from the robot
        as a JSON string and parsed as a dictionary passed as argument to this function.
        This function is called, for example, when receiving a notification from the robot indicating that a sidecar
        module has registered a new function.

        Args:
            function (callable): The function to call if this command is called.
                                 Typically, this function will send a message to the robot telling to call this
                                 registered function with the provided arguments)
            name (str): Name of the function (may include prefixes, like my_module.my_function)
            cmd_def (dict): Definition of the function (description, tags, arguments, etc.)

        Returns:
            RegisteredFunction: The RegisteredFunction object rebuilt from the provided cmd_def dictionary
        """
        function = RegisteredFunction(
            function=function,
            name=name,
            description=cmd_def.get(MX_JSON_KEY_SIDECAR_FCT_DESC, ""),
            args=[RegisteredArg.from_dict(arg_dict) for arg_dict in cmd_def.get(MX_JSON_KEY_SIDECAR_FCT_ARGS, [])],
            validate_args=bool(cmd_def.get(MX_JSON_KEY_SIDECAR_FCT_VALIDATE_ARGS, True)),
            tags=[str(tag) for tag in cmd_def.get(MX_JSON_KEY_SIDECAR_FCT_TAGS, [])],
            cyclic_id=cmd_def.get(MX_JSON_KEY_SIDECAR_FCT_CYCLIC_ID, None),
            private=bool(cmd_def.get(MX_JSON_KEY_SIDECAR_FCT_PRIVATE, False)))

        # Remember that this function was not locally registered (it comes from the robot's command dictionary)
        function.locally_created = False
        return function


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
            cyclic_id: Optional[int] = None,
            default: Optional[any] = None,
            min: Optional[any] = None,
            max: Optional[any] = None):
        """ Constructor for this class

        Args:
            See @ref RegisteredArg for arguments description.
            This class adds the following arguments to base class:
            module_name (str): Name of the module that has registered this variable
        """
        super().__init__(name, description, type, length, units, default, min, max)
        self.cyclic_id = cyclic_id
        self._value = default

    def set_value(self, value: any) -> any:
        """Set the variable's value. This function will attempt to "cast" the value into the expected type.

        Args:
            value (any): Value to set

        Raises:
            ValueError: Cannot set value (array of incorrect length for example)

        Returns:
            any: The previous value of the variable before this function call
        """
        old_value = self._value
        self._value = value
        return old_value

    def get_value(self) -> any:
        """ Get the current variable value """
        return self._value

    #pylint: disable=invalid-name
    def cast_value(self, value: any):
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
            if isinstance(self._value, list):
                return f'[{", ".join([str(x) for x in self._value])}]'
            else:
                return f'[{self._value}]'

    def __repr__(self) -> str:
        return str(self)


class AttributeName:
    """ This class is a simple container that is used by AttributeContainer to store the full name (with prefixes)
        of a registered attribute """

    def __init__(self, name: str):
        self.name = name


class AttributeContainer:
    """ This class is used to recursively store attributes (variables or functions) by path into object and sub-object,
    allowing code to later access attributes with code like:
        Calling a registered function:
            robot.my_module.my_sub_module.my_function(my_args)
        Read or write a registered variable
            robot.my_group.my_subgroup.my_variable += 2
            print(f'robot.my_group.my_subgroup.my_variable')

    This class supports saving the attributes in a local dictionary, or else using callback functions at run-time
    to get/set registered attribute values dynamically (without actually storing the values in this class instance).
    """

    def __init__(self):
        #pylint: disable=invalid-name
        self._get_attribute_callback: callable = None
        self._set_attribute_callback: callable = None
        self._attributes: dict[str, Union[str, AttributeContainer]] = {}

    def attach(self, get_attribute_callback: Callable[[str]], set_attribute_callback: Callable[[str, any]]):
        """Attach the callback functions that will be used to actually get and set the attributes when this
            class is accessed

        Args:
            get_attribute_callback (callable): Function to call (with attribute full name as argument) to retrieve an
                                               attribute that was registered in this container.
            set_attribute_callback (callable): Function to call (with attribute full name as argument and value) to
                                               change the value of an attribute that was registered in this container.
        """
        self._get_attribute_callback = get_attribute_callback
        self._set_attribute_callback = set_attribute_callback

    def detach(self):
        """ Detach previously attached get/set callbacks """
        self._get_attribute_callback = None
        self._set_attribute_callback = None

    def register_attribute(self, attribute_name: str, attribute_full_name: str, attribute: Optional[any] = None):
        """Register one attribute into this attribute container (recursively)

        Args:
            attribute_name (str): Name of the attribute to register (partial for nested instances of this class)
            attribute_full_name (str): Full attribute name (no prefix removal even when we're a nested instance)
            attribute(any): Optional attribute value to register. If None, then self._get_attribute_callback will
                            be used to retrieve the attribute
        """
        if '.' in attribute_name:
            # Nested attribute. Let's save into a sub-object
            prefix, sub_name = attribute_name.split('.', 1)
            if prefix not in self._attributes:
                self._attributes[prefix] = self.__class__()  # This should create AttributeContainer, or derived class
                self._attributes[prefix].attach(self._get_attribute_callback, self._set_attribute_callback)
            self._attributes[prefix].register_attribute(sub_name, attribute_full_name, attribute)
        else:
            # Not nested, save the attribute locally
            if attribute is None:
                # We'll retrieve attribute with self._get_attribute_callback when necessary
                self._attributes[attribute_name] = AttributeName(attribute_full_name)
            else:
                self._attributes[attribute_name] = attribute

    def unregister_attribute(self, attribute_name: str):
        """Unregister a previously registered attribute

        Args:
            attribute_name (str): The name of the attribute to unregister from this container
        """
        if '.' in attribute_name:
            # Nested attribute. Let's delete from sub-object
            prefix, sub_name = attribute_name.split('.', 1)
            if prefix in self._attributes:
                if isinstance(self._attributes[prefix], AttributeContainer):
                    self._attributes[prefix].unregister_attribute(sub_name)
                    if self._attributes[prefix].empty():
                        del self._attributes[prefix]
                else:
                    del self._attributes[prefix]
        else:
            # Not nested, delete the attribute locally
            if attribute_name in self._attributes:
                del self._attributes[attribute_name]

    def get(self, attribute_name: str) -> Optional[any]:
        """Get the specified attribute by name, or return None if not found.
           This is similar to accessing the container attribute directly (ex: container.my_attr) except that:
           - here we use a recursive string to identify the attribute (attr_name.sub_name...)
           - Here we return None if the attribute is not found instead of raising an exception.

        Args:
            attribute_name (str): Name of the attribute to get (including prefixes if appropriate)

        Returns:
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

    def set(self, attribute_name: str, value: any) -> Optional[any]:
        """Set the specified attribute by name, or raise NotFoundException if not found.
           This is similar to accessing the container attribute directly (ex: container.my_attr=...) except that
           here we use a recursive string to identify the attribute (attr_name.sub_name...)

        Args:
            attribute_name (str):   Name of the attribute to get (including prefixes if appropriate)
            value (any):            The new value to set for this attribute

        Raises:
            NotFoundException: The variable with specified name does not exist

        Returns:
            Optional[any]: The previous variable value.
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
        return len(self._attributes) == 0

    def __getattr__(self, attr_name: str) -> any:
        """We override this to provide access to registered attributes as class attributes

        Args:
            attr_name (str): The name of the attribute to get (no prefix, attribute name only)

        Raises:
            AttributeError: No attribute is registered with this name
            ModuleNotFoundError: No callback function attached, cannot retrieve the attribute value from parent module

        Returns:
            any: The retrieved attribute value
        """
        # Make sure we can still access member variables of this class
        if attr_name == "_get_attribute_callback":
            return self._get_attribute_callback
        if attr_name == "_set_attribute_callback":
            return self._set_attribute_callback
        if attr_name == "_attributes":
            return self._attributes

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

    def __setattr__(self, attr_name: str, value=any):
        """We override this to provide access to registered attributes as class attributes

        Args:
            attr_name (str): The name of the attribute to set (no prefix, attribute name only)
            value (_type_, optional): Value to set. Defaults to any.

        Raises:
            ValueError: Trying to assign a value into an already registered sub-group
            AttributeError: No attribute with this name was registered
            ModuleNotFoundError: No callback function attached, cannot set the attribute value into parent module
        """
        # Make sure we can still access member variables of this class
        if (attr_name == "_get_attribute_callback" or attr_name == "_set_attribute_callback"
                or attr_name == "_attributes" or attr_name == "_getting_reg_var"):
            return super().__setattr__(attr_name, value)

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

    def __init__(self):
        super().__init__()
        self._getting_reg_var = False

    def get(self, attribute_name: str) -> Optional[RegisteredVariable]:
        """ Returns the RegisteredVariable object by name """
        # This  will call __getattr__ which we want to return the RegisteredVariable object, not it's value
        self._getting_reg_var = True
        var = super().get(attribute_name)
        self._getting_reg_var = False
        return var

    def __getattr__(self, attr_name: str) -> any:
        """ See base class doc. Here we just return the value from the registered variable.
        This allows code that do robot.vars.myvar  to get the value directly, while robot.vars.get("myvar") returns
        a RegisteredVariable object
        """
        reg_var = super().__getattr__(attr_name)
        if isinstance(reg_var, RegisteredVariable) and not self._getting_reg_var:
            return reg_var.get_value()
        return reg_var


class ScriptUtils:
    """ This class contains utility functions for simplified .mxpy scripts """

    @classmethod
    def get_arg(cls,
                args: list[any],
                kwargs: dict[str, any],
                index: Optional[int] = None,
                name: Optional[str] = None,
                default: Optional[any] = None) -> any:
        """
        This is a utility function for user programs, in particular .mxpy programs.
        They have access to *args and *kwargs, but using 'get_arg' by index and/or keyword is convenient.
        Gets an argument from *args or **kwargs.

        This function will first attempt to retrieve the argument by name from kwargs.
        If not found, will try to retrieve the positional argument (by index) from args.

        Args:
            *args: Positional arguments.
            **kwargs: Keyword arguments.
            index: Index of the argument to retrieve from *args.
            name: Name of the argument to retrieve from **kwargs.
            default: Default value to return if the argument is not found.

        Returns:
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


def register_nested_function(function_full_name: str, get_parent_attr: Callable[[str]],
                             set_parent_attr: Callable[[str, any]], function: callable):
    """This is a helper method to register a nested function into a namespace.
    This function will take care to recursively create intermediate containers in case the function name contains
    prefixes (like my_group.my_sub_group.my_function)

    Args:
        function_full_name (str): Name of the function to register (which may have a prefix)
        get_parent_attr (Callable[[str]]): Callback function to get by name from the desired namespace
        set_parent_attr (Callable[[str, any]]): Callback function to set by name into the desired namespace
        function (callable): The function to register
    """
    if '.' in function_full_name:
        prefix, sub_name = function_full_name.split('.', 1)
        # Must register inside a nested attribute. Let's first check if this sub-container already exists
        attr_container: AttributeContainer = get_parent_attr(prefix)
        if attr_container is None:
            # Create a new attribute container
            attr_container = AttributeContainer()
            # Register this attribute container in the parent namespace
            set_parent_attr(prefix, attr_container)

        # Recursive registering inside the attribute container
        attr_container.register_attribute(sub_name, function_full_name, function)

    else:
        # Not nested, directly register the function in the parent namespace
        set_parent_attr(function_full_name, function)


def unregister_nested_function(function_full_name: str, get_parent_attr: Callable[[str]],
                               del_parent_attr: Callable[[str]]):
    """This is a helper method to recursively unregister a function from a namespace.

    Args:
        function_full_name (str): Name of the attribute to unregister (which may have a prefix)
        get_parent_attr (Callable[[str]]): Callback function to get by name (or prefix) from the desired namespace
        del_parent_attr (Callable[[str]]): Callback function to delete by name (or prefix) from the desired namespace
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
                # Note: Do not delete the prefix object from the root parent container because scripts will keep
                #       references to that root object. And later if we recreate a new root object, the scripts will
                #       continue (until they are reloaded) to point to the old (empty) root object and would not longer
                #       be able to call re-registered functions.
                #
                #       So we keep ('leak') this root object forever in case functions are re-registered in it later.
                #
                #       Note that it's correct to delete sub-attributes because scripts will go through root object's
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
