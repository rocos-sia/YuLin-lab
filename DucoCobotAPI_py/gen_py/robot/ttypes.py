#
# Autogenerated by Thrift Compiler (0.13.0)
#
# DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
#
#  options string: py
#

from thrift.Thrift import TType, TMessageType, TFrozenDict, TException, TApplicationException
from thrift.protocol.TProtocol import TProtocolException
from thrift.TRecursive import fix_spec

import sys

from thrift.transport import TTransport
all_structs = []


class StateRobot(object):
    SR_Start = 0
    SR_Initialize = 1
    SR_Logout = 2
    SR_Login = 3
    SR_PowerOff = 4
    SR_Disable = 5
    SR_Enable = 6

    _VALUES_TO_NAMES = {
        0: "SR_Start",
        1: "SR_Initialize",
        2: "SR_Logout",
        3: "SR_Login",
        4: "SR_PowerOff",
        5: "SR_Disable",
        6: "SR_Enable",
    }

    _NAMES_TO_VALUES = {
        "SR_Start": 0,
        "SR_Initialize": 1,
        "SR_Logout": 2,
        "SR_Login": 3,
        "SR_PowerOff": 4,
        "SR_Disable": 5,
        "SR_Enable": 6,
    }


class StateProgram(object):
    SP_Stopped = 0
    SP_Stopping = 1
    SP_Running = 2
    SP_Paused = 3
    SP_Pausing = 4

    _VALUES_TO_NAMES = {
        0: "SP_Stopped",
        1: "SP_Stopping",
        2: "SP_Running",
        3: "SP_Paused",
        4: "SP_Pausing",
    }

    _NAMES_TO_VALUES = {
        "SP_Stopped": 0,
        "SP_Stopping": 1,
        "SP_Running": 2,
        "SP_Paused": 3,
        "SP_Pausing": 4,
    }


class OperationMode(object):
    kManual = 0
    kAuto = 1
    kRemote = 2

    _VALUES_TO_NAMES = {
        0: "kManual",
        1: "kAuto",
        2: "kRemote",
    }

    _NAMES_TO_VALUES = {
        "kManual": 0,
        "kAuto": 1,
        "kRemote": 2,
    }


class TaskState(object):
    ST_Idle = 0
    ST_Running = 1
    ST_Paused = 2
    ST_Stopped = 3
    ST_Finished = 4
    ST_Interrupt = 5
    ST_Error = 6
    ST_Illegal = 7
    ST_ParameterMismatch = 8

    _VALUES_TO_NAMES = {
        0: "ST_Idle",
        1: "ST_Running",
        2: "ST_Paused",
        3: "ST_Stopped",
        4: "ST_Finished",
        5: "ST_Interrupt",
        6: "ST_Error",
        7: "ST_Illegal",
        8: "ST_ParameterMismatch",
    }

    _NAMES_TO_VALUES = {
        "ST_Idle": 0,
        "ST_Running": 1,
        "ST_Paused": 2,
        "ST_Stopped": 3,
        "ST_Finished": 4,
        "ST_Interrupt": 5,
        "ST_Error": 6,
        "ST_Illegal": 7,
        "ST_ParameterMismatch": 8,
    }


class SafetyState(object):
    SS_INIT = 0
    SS_WAIT = 2
    SS_CONFIG = 3
    SS_POWER_OFF = 4
    SS_RUN = 5
    SS_RECOVERY = 6
    SS_STOP2 = 7
    SS_STOP1 = 8
    SS_STOP0 = 9
    SS_MODEL = 10
    SS_REDUCE = 12
    SS_BOOT = 13
    SS_FAIL = 14
    SS_UPDATE = 99

    _VALUES_TO_NAMES = {
        0: "SS_INIT",
        2: "SS_WAIT",
        3: "SS_CONFIG",
        4: "SS_POWER_OFF",
        5: "SS_RUN",
        6: "SS_RECOVERY",
        7: "SS_STOP2",
        8: "SS_STOP1",
        9: "SS_STOP0",
        10: "SS_MODEL",
        12: "SS_REDUCE",
        13: "SS_BOOT",
        14: "SS_FAIL",
        99: "SS_UPDATE",
    }

    _NAMES_TO_VALUES = {
        "SS_INIT": 0,
        "SS_WAIT": 2,
        "SS_CONFIG": 3,
        "SS_POWER_OFF": 4,
        "SS_RUN": 5,
        "SS_RECOVERY": 6,
        "SS_STOP2": 7,
        "SS_STOP1": 8,
        "SS_STOP0": 9,
        "SS_MODEL": 10,
        "SS_REDUCE": 12,
        "SS_BOOT": 13,
        "SS_FAIL": 14,
        "SS_UPDATE": 99,
    }


class Op(object):
    """
    Attributes:
     - time_or_dist_1
     - trig_io_1
     - trig_value_1
     - trig_time_1
     - trig_dist_1
     - time_or_dist_2
     - trig_io_2
     - trig_value_2
     - trig_time_2
     - trig_dist_2

    """


    def __init__(self, time_or_dist_1=None, trig_io_1=None, trig_value_1=None, trig_time_1=None, trig_dist_1=None, time_or_dist_2=None, trig_io_2=None, trig_value_2=None, trig_time_2=None, trig_dist_2=None,):
        self.time_or_dist_1 = time_or_dist_1
        self.trig_io_1 = trig_io_1
        self.trig_value_1 = trig_value_1
        self.trig_time_1 = trig_time_1
        self.trig_dist_1 = trig_dist_1
        self.time_or_dist_2 = time_or_dist_2
        self.trig_io_2 = trig_io_2
        self.trig_value_2 = trig_value_2
        self.trig_time_2 = trig_time_2
        self.trig_dist_2 = trig_dist_2

    def read(self, iprot):
        if iprot._fast_decode is not None and isinstance(iprot.trans, TTransport.CReadableTransport) and self.thrift_spec is not None:
            iprot._fast_decode(self, iprot, [self.__class__, self.thrift_spec])
            return
        iprot.readStructBegin()
        while True:
            (fname, ftype, fid) = iprot.readFieldBegin()
            if ftype == TType.STOP:
                break
            if fid == 1:
                if ftype == TType.BYTE:
                    self.time_or_dist_1 = iprot.readByte()
                else:
                    iprot.skip(ftype)
            elif fid == 2:
                if ftype == TType.BYTE:
                    self.trig_io_1 = iprot.readByte()
                else:
                    iprot.skip(ftype)
            elif fid == 3:
                if ftype == TType.BOOL:
                    self.trig_value_1 = iprot.readBool()
                else:
                    iprot.skip(ftype)
            elif fid == 4:
                if ftype == TType.DOUBLE:
                    self.trig_time_1 = iprot.readDouble()
                else:
                    iprot.skip(ftype)
            elif fid == 5:
                if ftype == TType.DOUBLE:
                    self.trig_dist_1 = iprot.readDouble()
                else:
                    iprot.skip(ftype)
            elif fid == 6:
                if ftype == TType.BYTE:
                    self.time_or_dist_2 = iprot.readByte()
                else:
                    iprot.skip(ftype)
            elif fid == 7:
                if ftype == TType.BYTE:
                    self.trig_io_2 = iprot.readByte()
                else:
                    iprot.skip(ftype)
            elif fid == 8:
                if ftype == TType.BOOL:
                    self.trig_value_2 = iprot.readBool()
                else:
                    iprot.skip(ftype)
            elif fid == 9:
                if ftype == TType.DOUBLE:
                    self.trig_time_2 = iprot.readDouble()
                else:
                    iprot.skip(ftype)
            elif fid == 10:
                if ftype == TType.DOUBLE:
                    self.trig_dist_2 = iprot.readDouble()
                else:
                    iprot.skip(ftype)
            else:
                iprot.skip(ftype)
            iprot.readFieldEnd()
        iprot.readStructEnd()

    def write(self, oprot):
        if oprot._fast_encode is not None and self.thrift_spec is not None:
            oprot.trans.write(oprot._fast_encode(self, [self.__class__, self.thrift_spec]))
            return
        oprot.writeStructBegin('Op')
        if self.time_or_dist_1 is not None:
            oprot.writeFieldBegin('time_or_dist_1', TType.BYTE, 1)
            oprot.writeByte(self.time_or_dist_1)
            oprot.writeFieldEnd()
        if self.trig_io_1 is not None:
            oprot.writeFieldBegin('trig_io_1', TType.BYTE, 2)
            oprot.writeByte(self.trig_io_1)
            oprot.writeFieldEnd()
        if self.trig_value_1 is not None:
            oprot.writeFieldBegin('trig_value_1', TType.BOOL, 3)
            oprot.writeBool(self.trig_value_1)
            oprot.writeFieldEnd()
        if self.trig_time_1 is not None:
            oprot.writeFieldBegin('trig_time_1', TType.DOUBLE, 4)
            oprot.writeDouble(self.trig_time_1)
            oprot.writeFieldEnd()
        if self.trig_dist_1 is not None:
            oprot.writeFieldBegin('trig_dist_1', TType.DOUBLE, 5)
            oprot.writeDouble(self.trig_dist_1)
            oprot.writeFieldEnd()
        if self.time_or_dist_2 is not None:
            oprot.writeFieldBegin('time_or_dist_2', TType.BYTE, 6)
            oprot.writeByte(self.time_or_dist_2)
            oprot.writeFieldEnd()
        if self.trig_io_2 is not None:
            oprot.writeFieldBegin('trig_io_2', TType.BYTE, 7)
            oprot.writeByte(self.trig_io_2)
            oprot.writeFieldEnd()
        if self.trig_value_2 is not None:
            oprot.writeFieldBegin('trig_value_2', TType.BOOL, 8)
            oprot.writeBool(self.trig_value_2)
            oprot.writeFieldEnd()
        if self.trig_time_2 is not None:
            oprot.writeFieldBegin('trig_time_2', TType.DOUBLE, 9)
            oprot.writeDouble(self.trig_time_2)
            oprot.writeFieldEnd()
        if self.trig_dist_2 is not None:
            oprot.writeFieldBegin('trig_dist_2', TType.DOUBLE, 10)
            oprot.writeDouble(self.trig_dist_2)
            oprot.writeFieldEnd()
        oprot.writeFieldStop()
        oprot.writeStructEnd()

    def validate(self):
        if self.time_or_dist_1 is None:
            raise TProtocolException(message='Required field time_or_dist_1 is unset!')
        if self.trig_io_1 is None:
            raise TProtocolException(message='Required field trig_io_1 is unset!')
        if self.trig_value_1 is None:
            raise TProtocolException(message='Required field trig_value_1 is unset!')
        if self.trig_time_1 is None:
            raise TProtocolException(message='Required field trig_time_1 is unset!')
        if self.trig_dist_1 is None:
            raise TProtocolException(message='Required field trig_dist_1 is unset!')
        if self.time_or_dist_2 is None:
            raise TProtocolException(message='Required field time_or_dist_2 is unset!')
        if self.trig_io_2 is None:
            raise TProtocolException(message='Required field trig_io_2 is unset!')
        if self.trig_value_2 is None:
            raise TProtocolException(message='Required field trig_value_2 is unset!')
        if self.trig_time_2 is None:
            raise TProtocolException(message='Required field trig_time_2 is unset!')
        if self.trig_dist_2 is None:
            raise TProtocolException(message='Required field trig_dist_2 is unset!')
        return

    def __repr__(self):
        L = ['%s=%r' % (key, value)
             for key, value in self.__dict__.items()]
        return '%s(%s)' % (self.__class__.__name__, ', '.join(L))

    def __eq__(self, other):
        return isinstance(other, self.__class__) and self.__dict__ == other.__dict__

    def __ne__(self, other):
        return not (self == other)
all_structs.append(Op)
Op.thrift_spec = (
    None,  # 0
    (1, TType.BYTE, 'time_or_dist_1', None, None, ),  # 1
    (2, TType.BYTE, 'trig_io_1', None, None, ),  # 2
    (3, TType.BOOL, 'trig_value_1', None, None, ),  # 3
    (4, TType.DOUBLE, 'trig_time_1', None, None, ),  # 4
    (5, TType.DOUBLE, 'trig_dist_1', None, None, ),  # 5
    (6, TType.BYTE, 'time_or_dist_2', None, None, ),  # 6
    (7, TType.BYTE, 'trig_io_2', None, None, ),  # 7
    (8, TType.BOOL, 'trig_value_2', None, None, ),  # 8
    (9, TType.DOUBLE, 'trig_time_2', None, None, ),  # 9
    (10, TType.DOUBLE, 'trig_dist_2', None, None, ),  # 10
)
fix_spec(all_structs)
del all_structs
