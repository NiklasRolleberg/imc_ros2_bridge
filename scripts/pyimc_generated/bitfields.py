'''
IMC global bitfields definitions.
'''

import enum as _enum

#Enumerations:

class CLoopsMask(_enum.IntFlag):
    '''Full name: Control Loops Mask
    Prefix: CL'''

    EMPTY = 0
    '''No active flags'''

    NONE = 0
    '''Name: None'''

    PATH = 1
    '''Name: Path Control'''

    TELEOPERATION = 2
    '''Name: Teleoperation Control'''

    ALTITUDE = 4
    '''Name: Altitude Control'''

    DEPTH = 8
    '''Name: Depth Control'''

    ROLL = 16
    '''Name: Roll Control'''

    PITCH = 32
    '''Name: Pitch Control'''

    YAW = 64
    '''Name: Yaw Control'''

    SPEED = 128
    '''Name: Speed Control'''

    YAW_RATE = 256
    '''Name: Yaw Rate Control'''

    VERTICAL_RATE = 512
    '''Name: Vertical Rate Control'''

    TORQUE = 1024
    '''Name: Torque Control'''

    FORCE = 2048
    '''Name: Force Control'''

    VELOCITY = 4096
    '''Name: Velocity Control'''

    THROTTLE = 8192
    '''Name: Throttle Control'''

    EXTERNAL = 1073741824
    '''Name: Unspecified External Control'''

    NO_OVERRIDE = 2147483648
    '''Name: Non-overridable control'''

    ALL = 4294967295
    '''Name: All'''


class OpLimitsMask(_enum.IntFlag):
    '''Full name: Operational Limits Mask
    Prefix: OPL'''

    EMPTY = 0
    '''No active flags'''

    MAX_DEPTH = 1
    '''Name: Maximum Depth'''

    MIN_ALT = 2
    '''Name: Minimum Altitude'''

    MAX_ALT = 4
    '''Name: Maximum Altitude'''

    MIN_SPEED = 8
    '''Name: Minimum Speed'''

    MAX_SPEED = 16
    '''Name: Maximum Speed'''

    MAX_VRATE = 32
    '''Name: Maximum Vertical Rate'''

    AREA = 64
    '''Name: Operation Area'''

