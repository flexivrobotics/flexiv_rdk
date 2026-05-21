#!/usr/bin/env python

"""utility.py

Utility methods.

Import this module as `import utility` and use functions through the module namespace,
for example `utility.quat2eulerZYX(...)`.
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


def primitive_state_true_for_groups(primitive_states, groups, state_name=None):
    """
    Check whether one primitive state is true for all specified groups.

    Parameters
    ----------
    primitive_states : dict
        Primitive states keyed by joint group.
    groups : iterable or dict
        If state_name is provided, this is an iterable of joint groups. If state_name is None,
        this must be a dict mapping each joint group to its transition key.
    state_name : str or None
        Primitive state name to check for all specified groups. Leave as None to use a per-group
        transition-key dict from groups.

    Returns
    ----------
    bool
        True if all specified groups contain the requested primitive state and its value is true.
    """

    if state_name is None:
        return all(
            group in primitive_states
            and transition_key in primitive_states[group].names_and_values
            and bool(primitive_states[group].names_and_values[transition_key])
            for group, transition_key in groups.items()
        )

    return all(
        group in primitive_states
        and state_name in primitive_states[group].names_and_values
        and bool(primitive_states[group].names_and_values[state_name])
        for group in groups
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
