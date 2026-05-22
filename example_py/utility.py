#!/usr/bin/env python

"""utility.py

Utility methods.

Import this module as `import utility` and use functions through the module namespace,
for example `utility.quat2eulerZYX(...)` and `utility.primitive_state_true_for_groups(...)`.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import math

# pip install scipy
from scipy.spatial.transform import Rotation as R


def quat2eulerZYX(quat, degree=False):
    """
    Convert quaternion to Euler angles with ZYX axis rotations.

    Parameters
    ----------
    quat : float list
        Quaternion input in [w,x,y,z] order.
    degree : bool
        Return values in degrees, otherwise in radians.

    Returns
    ----------
    float list
        Euler angles in [x,y,z] order, radian by default unless specified otherwise.
        Returned angles are normalized by scipy to a principal range. In degree mode,
        this is typically within [-180, 180].
    """

    # Convert target quaternion to Euler ZYX using scipy package's 'xyz' extrinsic rotation
    # NOTE: scipy uses [x,y,z,w] order to represent quaternion. The returned Euler values
    # are a normalized representation of the same physical orientation.
    eulerZYX = (
        R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        .as_euler("xyz", degrees=degree)
        .tolist()
    )

    return eulerZYX


def primitive_state_true_for_groups(primitive_states, state_name):
    """
    Check whether primitive states are true for all groups included in primitive_states.

    Parameters
    ----------
    primitive_states : dict
        Primitive states keyed by joint group.
    state_name : str or dict
        Primitive state name to check for all groups, or per-group state names keyed by joint
        group, e.g. "reachedTarget" or {group: "terminated"}.

    Returns
    ----------
    bool
        True if all included groups contain the requested primitive state and its integer value is
        non-zero.
    """

    def primitive_state_true(primitive_state, requested_state_name):
        if requested_state_name not in primitive_state.names_and_values:
            return False

        value = primitive_state.names_and_values[requested_state_name]
        return isinstance(value, int) and value != 0

    if isinstance(state_name, dict):
        return all(
            group in state_name
            and primitive_state_true(primitive_state, state_name[group])
            for group, primitive_state in primitive_states.items()
        )

    return all(
        primitive_state_true(primitive_state, state_name)
        for primitive_state in primitive_states.values()
    )


def list2str(ls):
    """
    Convert a list to a string.

    Parameters
    ----------
    ls : list
        Source list of any size.

    Returns
    ----------
    str
        A string with format "ls[0] ls[1] ... ls[n] ", i.e. each value
        followed by a space, including the last one.
    """

    ret_str = ""
    for i in ls:
        ret_str += str(i) + " "
    return ret_str
