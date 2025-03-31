"""
This file is a namespace that contains sidecar registered functions as global functions that scripts can call directly,
for example a script import * from this file, then simply call MoveCirc() or my_module.my_function(my_args).
"""
from __future__ import annotations

from mecademicpy.robot_sidecar_classes import register_nested_function, unregister_nested_function


def _get_global(attr_name: str) -> any:
    """ This helper function is used to get an attribute from the global namespace of the calling function """
    return globals()[attr_name] if attr_name in globals() else None


def _set_global(attr_name: str, attr_val: any):
    """ This helper function is used to set an attribute from the global namespace of the calling function """
    globals()[attr_name] = attr_val


def _del_global(attr_name: str):
    """ This helper function is used to delete an attribute from the global namespace of the calling function """
    del globals()[attr_name]


def attach_global_function(function: callable, function_name: str):
    """Attach a function to the global namespace defined in this file, and which script files will import in their
       own namespace in order to be able to call these functions directly.

    Args:
        function (callable): The function to register
        function_name (str): Name of the function to export in the namespace
    """

    # Add the function name to the global namespace (scope this file or any file importing * from this file)
    # nested under prefix if appropriate.
    # We do that by calling helper register_nested_function and passing the appropriate callbacks (_get_global and
    # _set_global) so it sets the attribute in our namespace
    # This helper supports nested functions (with one or multiple prefixes)
    register_nested_function(function_full_name=function_name,
                             get_parent_attr=_get_global,
                             set_parent_attr=_set_global,
                             function=function)


def detach_global_function(function_name: str):
    """ Detach a function previously registered with attach_global_function"""
    # Use the helper unregister_nested_function to which we pass appropriate callbacks to get/delete from our
    # namespace (_get_global and _del_global).
    # This helper supports nested functions (with one or multiple prefixes)
    unregister_nested_function(function_full_name=function_name,
                               get_parent_attr=_get_global,
                               del_parent_attr=_del_global)
