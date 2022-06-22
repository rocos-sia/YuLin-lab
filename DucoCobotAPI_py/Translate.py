import numpy as np
from scipy.spatial.transform import Rotation as Ro


def rot2T(pose):
    r = Ro.from_rotvec([pose[3], pose[4], pose[5]])
    rotation = r.as_matrix()
    translation = np.array([pose[0], pose[1], pose[2]])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2rot(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rv = r.as_rotvec()
    rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
    return rv_end


def T2rot_2(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rot = r.as_rotvec()
    return end_translation, rot


def euler2T(pose):
    r = Ro.from_euler('XYZ', [pose[3], pose[4], pose[5]])
    rotation = r.as_matrix()
    translation = np.array([pose[0], pose[1], pose[2]])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2euler(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rv = r.as_euler('XYZ')
    rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
    return rv_end


def T2euler_2(matrix):
    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_euler('XYZ')
    return end_translation, end_rotation


def quat2T(a, b, c, d, e, f, g):
    r = Ro.from_quat([d, e, f, g])
    rotation = r.as_matrix()
    translation = np.array([a, b, c])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2quat(matrix):
    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_quat()
    qvec_nvm = np.array(end_rotation)
    return end_translation, qvec_nvm


def quat2euler(data):
    a = Ro.from_quat(data)
    rot = a.as_euler('XYZ')
    return rot


def euler2quat(data):
    a = Ro.from_euler('XYZ', data)
    quart = a.as_quat()
    return quart


def euler2rot(rotation):
    a = Ro.from_euler('XYZ', rotation)
    rot = a.as_rotvec()
    return rot


def send_matrix(tr_matrix, ro_matrix):
    str_my = ""
    for x in tr_matrix:
        str_my += str('%.4f' % x) + " "
    for y in ro_matrix:
        str_my += str('%.4f' % y) + " "
    # str_my += "end"
    str_my.encode('utf-8')
    return str_my


def rpy2T(pose):
    r = Ro.from_euler('xyz', [pose[3], pose[4], pose[5]])
    rotation = r.as_matrix()
    translation = np.array([pose[0], pose[1], pose[2]])
    one = np.array([0, 0, 0, 1])
    t = np.concatenate([rotation.reshape(3, 3), translation.reshape(3, 1)], axis=1)
    T = np.concatenate([t.reshape(3, 4), one.reshape(1, 4)], axis=0)
    return T


def T2rpy(T):
    end_rotation = T[0:3, 0:3]
    end_translation = T[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    rv = r.as_euler('xyz')
    rv_end = [end_translation[0], end_translation[1], end_translation[2], rv[0], rv[1], rv[2]]
    return rv_end


def T2rpy_2(matrix):
    end_rotation = matrix[0:3, 0:3]
    end_translation = matrix[0:3, 3]
    r = Ro.from_matrix(end_rotation)
    end_rotation = r.as_euler('xyz')
    return end_translation, end_rotation
