#!/usr/bin/env python

"""utility.py

Utility methods.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
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
    """

    # Convert target quaternion to Euler ZYX using scipy package's 'xyz' extrinsic rotation
    # NOTE: scipy uses [x,y,z,w] order to represent quaternion
    eulerZYX = (
        R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        .as_euler("xyz", degrees=degree)
        .tolist()
    )

    return eulerZYX


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
