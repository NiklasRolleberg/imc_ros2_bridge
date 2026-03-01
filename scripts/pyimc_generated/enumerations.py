'''
IMC global enumerations definitions.
'''

import enum as _enum

#Enumerations:

class Boolean(_enum.IntEnum):
    '''Full name: Boolean Value
    Prefix: BOOL'''

    FALSE = 0
    '''Name: False'''

    TRUE = 1
    '''Name: True'''


class ControlledMode(_enum.IntEnum):
    '''Full name: Controlled Mode
    Prefix: CTLMD'''

    RELINQUISH_HANDOFF_CTL = 0
    '''Name: Relinquish / Handoff Control'''

    REQUEST_CTL = 1
    '''Name: Request Control'''

    OVERRIDE_CTL = 2
    '''Name: Override Control'''


class SpeedUnits(_enum.IntEnum):
    '''Full name: Speed Units
    Prefix: SUNITS'''

    METERS_PS = 0
    '''Name: Meters per second'''

    RPM = 1
    '''Name: RPM'''

    PERCENTAGE = 2
    '''Name: Percentage'''


class SystemType(_enum.IntEnum):
    '''Full name: System Type
    Prefix: SYSTEMTYPE'''

    CCU = 0
    '''Name: CCU'''

    HUMANSENSOR = 1
    '''Name: Human-portable Sensor'''

    UUV = 2
    '''Name: UUV'''

    USV = 3
    '''Name: USV'''

    UAV = 4
    '''Name: UAV'''

    UGV = 5
    '''Name: UGV'''

    STATICSENSOR = 6
    '''Name: Static sensor'''

    MOBILESENSOR = 7
    '''Name: Mobile sensor'''

    WSN = 8
    '''Name: Wireless Sensor Network'''


class ZUnits(_enum.IntEnum):
    '''Full name: Z Units
    Prefix: Z'''

    NONE = 0
    '''Name: None'''

    DEPTH = 1
    '''Name: Depth'''

    ALTITUDE = 2
    '''Name: Altitude'''

    HEIGHT = 3
    '''Name: Height'''


class RSSIUnits(_enum.IntEnum):
    '''Full name: RSSI Units
    Prefix: RSSIUNITS'''

    dB = 0
    '''Name: Decibel'''

    PERCENTAGE = 1
    '''Name: Percentage'''


class UAVType(_enum.IntEnum):
    '''Full name: UAV Type
    Prefix: UAVTYPE'''

    FIXEDWING = 0
    '''Name: Fixed-Wing'''

    COPTER = 1
    '''Name: Copter'''

    VTOL = 2
    '''Name: Vtol'''

