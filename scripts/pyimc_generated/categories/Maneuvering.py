'''
IMC Maneuvering messages.
'''

from .. import _base
import enum as _enum

class Goto(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            roll : fp64_t, unit: rad

            pitch : fp64_t, unit: rad

            yaw : fp64_t, unit: rad

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_roll', '_pitch', '_yaw', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 450, stable = None, description = "A \"Goto\" is a maneuver specifying a movement of the vehicle to a target waypoint. The waypoint is described by the WGS-84 waypoint coordinate and target Z reference. Mandatory parameters defined for a \"Goto\" are timeout, speed and speed units. Optional parameters may be defined for the target Euler Angles (roll, pitch and yaw) though these parameters may not be considered by all maneuver controllers in charge of the execution of this type of maneuver.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'roll', 'pitch', 'yaw', 'custom',), abbrev = "Goto", name = "Goto Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    roll = _base.mutable_attr({'name': 'Roll', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.0, 'max': 6.283185307179586}, "The phi Euler angle in which the vehicle should set its attitude. Use '-1' for this field to be unconsidered. Otherwise the value spans from 0 to 2 Pi.")
    '''The phi Euler angle in which the vehicle should set its attitude. Use '-1' for this field to be unconsidered. Otherwise the value spans from 0 to 2 Pi. Type: fp64_t'''
    pitch = _base.mutable_attr({'name': 'Pitch', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.0, 'max': 6.283185307179586}, "The theta Euler angle in which the vehicle should set its attitude. Use '-1' for this field to be disconcerted. Otherwise the value spans from 0 to 2 Pi.")
    '''The theta Euler angle in which the vehicle should set its attitude. Use '-1' for this field to be disconcerted. Otherwise the value spans from 0 to 2 Pi. Type: fp64_t'''
    yaw = _base.mutable_attr({'name': 'Yaw', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.0, 'max': 6.283185307179586}, "The psi Euler angle in which the vehicle should set its attitude. Use '-1' for this field to be considered. Otherwise the value spans from 0 to 2 Pi.")
    '''The psi Euler angle in which the vehicle should set its attitude. Use '-1' for this field to be considered. Otherwise the value spans from 0 to 2 Pi. Type: fp64_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, roll = None, pitch = None, yaw = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            roll : fp64_t, unit: rad

            pitch : fp64_t, unit: rad

            yaw : fp64_t, unit: rad

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._custom = custom


class PopUp(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            duration : uint16_t, unit: s

            radius : fp32_t, unit: m

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        CURR_POS = 1
        '''Name: Start from current position'''
    
        WAIT_AT_SURFACE = 2
        '''Name: Wait at surface'''
    
        STATION_KEEP = 4
        '''Name: Station keeping'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_duration', '_radius', '_flags', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 451, stable = None, description = "The Pop Up maneuver makes the vehicle come to the surface at a specific waypoint. This maneuver is restricted to underwater vehicles.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'duration', 'radius', 'flags', 'custom',), abbrev = "PopUp", name = "PopUp Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run. If the maneuver is not completed in the amount of time specified an error will be generated.")
    '''The amount of time the maneuver is allowed to run. If the maneuver is not completed in the amount of time specified an error will be generated. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "The duration of this maneuver at surface level. Only used if flag WAIT_AT_SURFACE is on.")
    '''The duration of this maneuver at surface level. Only used if flag WAIT_AT_SURFACE is on. Type: uint16_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t', 'unit': 'm', 'min': 1, 'max': 100000}, "Radius of the maneuver. Only used if flag STATION_KEEP is on.")
    '''Radius of the maneuver. Only used if flag STATION_KEEP is on. Type: fp32_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'prefix': 'FLG', 'type': 'uint8_t', 'unit': 'Bitfield'}, "Flags of the maneuver. Bitfield (Local).")
    '''Flags of the maneuver. Bitfield (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, duration = None, radius = None, flags = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            duration : uint16_t, unit: s

            radius : fp32_t, unit: m

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._duration = duration
        self._radius = radius
        self._flags = flags
        self._custom = custom


class Teleoperation(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 452, stable = None, description = "The Teleoperation Maneuver lets the vehicle be controlled by an external human operator.", fields = ('custom',), abbrev = "Teleoperation", name = "Teleoperation Maneuver", flags = None)

    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    custom : plaintext, unit: TupleList'''
        self._custom = custom


class Loiter(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            type : uint8_t, unit: Enumerated (Local)

            radius : fp32_t, unit: m

            length : fp32_t, unit: m

            bearing : fp64_t, unit: rad

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''

    class TYPE(_enum.IntEnum):
        '''Full name: Loiter Type
        Prefix: LT'''
    
        DEFAULT = 0
        '''Name: Default'''
    
        CIRCULAR = 1
        '''Name: Circular'''
    
        RACETRACK = 2
        '''Name: Race track'''
    
        EIGHT = 3
        '''Name: Figure 8'''
    
        HOVER = 4
        '''Name: Hover'''
    
    
    class DIRECTION(_enum.IntEnum):
        '''Full name: Direction
        Prefix: LD'''
    
        VDEP = 0
        '''Name: Vehicle Dependent'''
    
        CLOCKW = 1
        '''Name: Clockwise'''
    
        CCLOCKW = 2
        '''Name: Counter Clockwise'''
    
        IWINDCURR = 3
        '''Name: Into the wind/current'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_duration', '_speed', '_speed_units', '_type', '_radius', '_length', '_bearing', '_direction', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 453, stable = None, description = "The Loiter maneuver makes the vehicle circle around a specific waypoint with fixed depth reference.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'duration', 'speed', 'speed_units', 'type', 'radius', 'length', 'bearing', 'direction', 'custom',), abbrev = "Loiter", name = "Loiter Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The timeout indicates the time that an error should occur if exceeded.")
    '''The timeout indicates the time that an error should occur if exceeded. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate.")
    '''WGS-84 Latitude coordinate. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate.")
    '''WGS-84 Longitude coordinate. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "The duration of this maneuver. Use '0' for unlimited duration time.")
    '''The duration of this maneuver. Use '0' for unlimited duration time. Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    type = _base.mutable_attr({'name': 'Loiter Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'LT', 'max': 4}, "Loiter maneuver type. Enumerated (Local).")
    '''Loiter maneuver type. Enumerated (Local). Type: uint8_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t', 'unit': 'm', 'min': 1, 'max': 100000}, "Radius of the maneuver.")
    '''Radius of the maneuver. Type: fp32_t'''
    length = _base.mutable_attr({'name': 'Length', 'type': 'fp32_t', 'unit': 'm', 'min': 1, 'max': 100000}, "Length of the maneuver.")
    '''Length of the maneuver. Type: fp32_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp64_t', 'unit': 'rad', 'min': 0, 'max': 6.283185307179586}, "Bearing of the maneuver.")
    '''Bearing of the maneuver. Type: fp64_t'''
    direction = _base.mutable_attr({'name': 'Direction', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'LD', 'max': 3}, "Desired direction. Enumerated (Local).")
    '''Desired direction. Enumerated (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, duration = None, speed = None, speed_units = None, type = None, radius = None, length = None, bearing = None, direction = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            type : uint8_t, unit: Enumerated (Local)

            radius : fp32_t, unit: m

            length : fp32_t, unit: m

            bearing : fp64_t, unit: rad

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._duration = duration
        self._speed = speed
        self._speed_units = speed_units
        self._type = type
        self._radius = radius
        self._length = length
        self._bearing = bearing
        self._direction = direction
        self._custom = custom


class IdleManeuver(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    duration : uint16_t, unit: s

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_duration', '_custom']
    Attributes = _base.MessageAttributes(source = "", usedby = None, category = "Maneuvering", id = 454, stable = None, description = "Causes the vehicle to stay idle for some time.", fields = ('duration', 'custom',), abbrev = "IdleManeuver", name = "Idle Maneuver", flags = None)

    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Optional duration of the Idle maneuver. Use '0' for unlimited duration time (maneuver will have to be explicitly stopped).")
    '''Optional duration of the Idle maneuver. Use '0' for unlimited duration time (maneuver will have to be explicitly stopped). Type: uint16_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, duration = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    duration : uint16_t, unit: s

            custom : plaintext, unit: TupleList'''
        self._duration = duration
        self._custom = custom


class LowLevelControl(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    control : message, unit: NOT FOUND

            duration : uint16_t, unit: s

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_control', '_duration', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 455, stable = None, description = "Low level maneuver that sends a (heading, roll, speed, ...) reference to a controller of the vehicle and then optionally lingers for some time.", fields = ('control', 'duration', 'custom',), abbrev = "LowLevelControl", name = "Low Level Control Maneuver", flags = None)

    control = _base.mutable_attr({'name': 'Control', 'type': 'message', 'message-type': 'ControlCommand'}, "Control command: can be of type DesiredZ, DesiredHeading, DesiredRoll, DesiredPitch, DesiredSpeed, DesiredThrottle or DesiredPath.")
    '''Control command: can be of type DesiredZ, DesiredHeading, DesiredRoll, DesiredPitch, DesiredSpeed, DesiredThrottle or DesiredPath. Type: message'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Duration of the control.")
    '''Duration of the control. Type: uint16_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, control = None, duration = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    control : message, unit: NOT FOUND

            duration : uint16_t, unit: s

            custom : plaintext, unit: TupleList'''
        self._control = control
        self._duration = duration
        self._custom = custom


class Rows(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            bearing : fp64_t, unit: rad

            cross_angle : fp64_t, unit: rad

            width : fp32_t, unit: m

            length : fp32_t, unit: m

            hstep : fp32_t, unit: m

            coff : uint8_t, unit: m

            alternation : uint8_t, unit: %

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        SQUARE_CURVE = 1
        '''Name: Square Curve'''
    
        CURVE_RIGHT = 2
        '''Name: First Curve Right'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_bearing', '_cross_angle', '_width', '_length', '_hstep', '_coff', '_alternation', '_flags', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 456, stable = None, description = "Rows maneuver (i.e: lawn mower type maneuver)", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'bearing', 'cross_angle', 'width', 'length', 'hstep', 'coff', 'alternation', 'flags', 'custom',), abbrev = "Rows", name = "Rows Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp64_t', 'unit': 'rad', 'min': 0, 'max': 6.283185307179586}, "Rows bearing angle.")
    '''Rows bearing angle. Type: fp64_t'''
    cross_angle = _base.mutable_attr({'name': 'Cross Angle', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.047197551197, 'max': 1.047197551197}, "Rows cross angle reference.")
    '''Rows cross angle reference. Type: fp64_t'''
    width = _base.mutable_attr({'name': 'Width', 'min': 0, 'type': 'fp32_t', 'unit': 'm'}, "Width of the maneuver.")
    '''Width of the maneuver. Type: fp32_t'''
    length = _base.mutable_attr({'name': 'Length', 'min': 0, 'type': 'fp32_t', 'unit': 'm'}, "Length of the maneuver.")
    '''Length of the maneuver. Type: fp32_t'''
    hstep = _base.mutable_attr({'name': 'Horizontal Step', 'type': 'fp32_t', 'unit': 'm', 'min': 0, 'value': 30}, "Horizontal step.")
    '''Horizontal step. Type: fp32_t'''
    coff = _base.mutable_attr({'name': 'Curve Offset', 'type': 'uint8_t', 'unit': 'm'}, "Desired curve offset.")
    '''Desired curve offset. Type: uint8_t'''
    alternation = _base.mutable_attr({'name': 'Alternation Parameter', 'type': 'uint8_t', 'unit': '%', 'max': 100, 'value': 50}, "Alternation parameter.")
    '''Alternation parameter. Type: uint8_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FLG'}, "Maneuver flags. Bitfield (Local).")
    '''Maneuver flags. Bitfield (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, bearing = None, cross_angle = None, width = None, length = None, hstep = None, coff = None, alternation = None, flags = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            bearing : fp64_t, unit: rad

            cross_angle : fp64_t, unit: rad

            width : fp32_t, unit: m

            length : fp32_t, unit: m

            hstep : fp32_t, unit: m

            coff : uint8_t, unit: m

            alternation : uint8_t, unit: %

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._bearing = bearing
        self._cross_angle = cross_angle
        self._width = width
        self._length = length
        self._hstep = hstep
        self._coff = coff
        self._alternation = alternation
        self._flags = flags
        self._custom = custom


class FollowPath(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            points : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_points', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 457, stable = None, description = "Maneuver constituted by a list of Path Points.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'points', 'custom',), abbrev = "FollowPath", name = "Follow Path Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of start point.")
    '''WGS-84 Latitude of start point. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of start point.")
    '''WGS-84 Longitude of start point. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    points = _base.mutable_attr({'name': 'Path Points', 'type': 'message-list', 'message-type': 'PathPoint'}, "List of PathPoint messages, encoding the path points.")
    '''List of PathPoint messages, encoding the path points. Type: message-list'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, points = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            points : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._points = points
        self._custom = custom


class PathPoint(_base.base_message):
    '''The Down offset of the North/East/Down coordinate of this point in relation to the path start point.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 458, stable = None, description = "Waypoint coordinate of a Follow Path maneuver.", fields = ('x', 'y', 'z',), abbrev = "PathPoint", name = "Path Point", flags = None)

    x = _base.mutable_attr({'name': 'North Offset (m)', 'type': 'fp32_t', 'unit': 'm'}, "The North offset of the North/East/Down coordinate of this point in relation to the path start point.")
    '''The North offset of the North/East/Down coordinate of this point in relation to the path start point. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'East Offset (m)', 'type': 'fp32_t', 'unit': 'm'}, "The East offset of the North/East/Down coordinate of this point in relation to the path start point.")
    '''The East offset of the North/East/Down coordinate of this point in relation to the path start point. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Down Offset (m)', 'type': 'fp32_t', 'unit': 'm'}, "The Down offset of the North/East/Down coordinate of this point in relation to the path start point.")
    '''The Down offset of the North/East/Down coordinate of this point in relation to the path start point. Type: fp32_t'''

    def __init__(self, x = None, y = None, z = None):
        '''Class constructor
        
        The Down offset of the North/East/Down coordinate of this point in relation to the path start point.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m'''
        self._x = x
        self._y = y
        self._z = z


class YoYo(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            amplitude : fp32_t, unit: m

            pitch : fp32_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_amplitude', '_pitch', '_speed', '_speed_units', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 459, stable = None, description = "A \"yo-yo\" is a maneuver specifying a vehicle movement to a target waypoint in which depth/altitude varies along the way between two values with a desired pitch angle.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'amplitude', 'pitch', 'speed', 'speed_units', 'custom',), abbrev = "YoYo", name = "Yo-Yo Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    amplitude = _base.mutable_attr({'name': 'Amplitude', 'type': 'fp32_t', 'unit': 'm'}, "Amplitude.")
    '''Amplitude. Type: fp32_t'''
    pitch = _base.mutable_attr({'name': 'Pitch Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': 0, 'max': 0.78539816}, "Pitch angle for use in ascent/descent.")
    '''Pitch angle for use in ascent/descent. Type: fp32_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, amplitude = None, pitch = None, speed = None, speed_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            amplitude : fp32_t, unit: m

            pitch : fp32_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._amplitude = amplitude
        self._pitch = pitch
        self._speed = speed
        self._speed_units = speed_units
        self._custom = custom


class TeleoperationDone(_base.base_message):
    '''Notification of completion of a Teleoperation maneuver.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 460, stable = None, description = "Notification of completion of a Teleoperation maneuver.", fields = [], abbrev = "TeleoperationDone", name = "Teleoperation Done", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Notification of completion of a Teleoperation maneuver.

       This message class contains the following fields and their respective types:
'''


class StationKeeping(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_radius', '_duration', '_speed', '_speed_units', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 461, stable = None, description = "The Station Keeping maneuver makes the vehicle come to the surface and then enter a given circular perimeter around a waypoint coordinate for a certain amount of time.", fields = ('lat', 'lon', 'z', 'z_units', 'radius', 'duration', 'speed', 'speed_units', 'custom',), abbrev = "StationKeeping", name = "Station Keeping", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t', 'unit': 'm'}, "Radius.")
    '''Radius. Type: fp32_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Duration (0 for unlimited).")
    '''Duration (0 for unlimited). Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "The value of the desired speed, in the scale specified by the \"Speed Units\" field.")
    '''The value of the desired speed, in the scale specified by the \"Speed Units\" field. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Indicates the units used for the speed value. Enumerated (Global).")
    '''Indicates the units used for the speed value. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, radius = None, duration = None, speed = None, speed_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._radius = radius
        self._duration = duration
        self._speed = speed
        self._speed_units = speed_units
        self._custom = custom


class Elevator(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            flags : uint8_t, unit: Bitfield (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            start_z : fp32_t, unit: m

            start_z_units : uint8_t, unit: Enumerated (Global)

            end_z : fp32_t, unit: m

            end_z_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        CURR_POS = 1
        '''Name: Start from current position'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_flags', '_lat', '_lon', '_start_z', '_start_z_units', '_end_z', '_end_z_units', '_radius', '_speed', '_speed_units', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 462, stable = None, description = "The Elevator maneuver specifies a vehicle to reach a target waypoint using a cruise altitude/depth and then ascend or descend to another target altitude/depth. The ascent/descent slope and radius can also be optionally specified.", fields = ('timeout', 'flags', 'lat', 'lon', 'start_z', 'start_z_units', 'end_z', 'end_z_units', 'radius', 'speed', 'speed_units', 'custom',), abbrev = "Elevator", name = "Elevator Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run. If the maneuver is not completed in the amount of time specified an error will be generated.")
    '''The amount of time the maneuver is allowed to run. If the maneuver is not completed in the amount of time specified an error will be generated. Type: uint16_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'prefix': 'FLG', 'type': 'uint8_t', 'unit': 'Bitfield'}, "Flags of the maneuver. Bitfield (Local).")
    '''Flags of the maneuver. Bitfield (Local). Type: uint8_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    start_z = _base.mutable_attr({'name': 'Start Point -- Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Altitude or depth of start point. This parameter will be ignored if the 'NO_Z' flag is set, or if the 'START' flag is not set.")
    '''Altitude or depth of start point. This parameter will be ignored if the 'NO_Z' flag is set, or if the 'START' flag is not set. Type: fp32_t'''
    start_z_units = _base.mutable_attr({'name': 'Start Point -- Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the start point's z reference. Enumerated (Global).")
    '''Units of the start point's z reference. Enumerated (Global). Type: uint8_t'''
    end_z = _base.mutable_attr({'name': 'End Point -- Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Depth or altitude for the end point. This parameter will be ignored if the 'NO_Z' flag is set.")
    '''Depth or altitude for the end point. This parameter will be ignored if the 'NO_Z' flag is set. Type: fp32_t'''
    end_z_units = _base.mutable_attr({'name': 'End Point -- Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the end point's z reference. Enumerated (Global).")
    '''Units of the end point's z reference. Enumerated (Global). Type: uint8_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t', 'unit': 'm', 'min': 0}, "Radius for use in ascent/descent. If 0 the controller to should use a custom control strategy.")
    '''Radius for use in ascent/descent. If 0 the controller to should use a custom control strategy. Type: fp32_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed.")
    '''Maneuver speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, flags = None, lat = None, lon = None, start_z = None, start_z_units = None, end_z = None, end_z_units = None, radius = None, speed = None, speed_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            flags : uint8_t, unit: Bitfield (Local)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            start_z : fp32_t, unit: m

            start_z_units : uint8_t, unit: Enumerated (Global)

            end_z : fp32_t, unit: m

            end_z_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._flags = flags
        self._lat = lat
        self._lon = lon
        self._start_z = start_z
        self._start_z_units = start_z_units
        self._end_z = end_z
        self._end_z_units = end_z_units
        self._radius = radius
        self._speed = speed
        self._speed_units = speed_units
        self._custom = custom


class FollowTrajectory(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            points : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_points', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 463, stable = None, description = "Maneuver constituted by a list of Trajectory Points.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'points', 'custom',), abbrev = "FollowTrajectory", name = "Follow Trajectory", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude for start point.")
    '''WGS-84 Latitude for start point. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude for start point.")
    '''WGS-84 Longitude for start point. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed.")
    '''Maneuver speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    points = _base.mutable_attr({'name': 'Trajectory Points', 'type': 'message-list', 'message-type': 'TrajectoryPoint'}, "List of trajectory points.")
    '''List of trajectory points. Type: message-list'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, points = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            points : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._points = points
        self._custom = custom


class TrajectoryPoint(_base.base_message):
    '''The time offset relative to the previous trajectory point.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            t : fp32_t, unit: s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_x', '_y', '_z', '_t']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 464, stable = None, description = "Waypoint coordinate of a Follow Trajectory maneuver.", fields = ('x', 'y', 'z', 't',), abbrev = "TrajectoryPoint", name = "Trajectory Point", flags = None)

    x = _base.mutable_attr({'name': 'North Offset (m)', 'type': 'fp32_t', 'unit': 'm'}, "The North offset of the North/East/Down coordinate of this point in relation to the trajectory start point.")
    '''The North offset of the North/East/Down coordinate of this point in relation to the trajectory start point. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'East Offset (m)', 'type': 'fp32_t', 'unit': 'm'}, "The East offset of the North/East/Down coordinate of this point in relation to the trajectory start point.")
    '''The East offset of the North/East/Down coordinate of this point in relation to the trajectory start point. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Down Offset (m)', 'type': 'fp32_t', 'unit': 'm'}, "The Down offset of the North/East/Down coordinate of this point in relation to the trajectory start point.")
    '''The Down offset of the North/East/Down coordinate of this point in relation to the trajectory start point. Type: fp32_t'''
    t = _base.mutable_attr({'name': 'Time Offset (s)', 'type': 'fp32_t', 'unit': 's'}, "The time offset relative to the previous trajectory point.")
    '''The time offset relative to the previous trajectory point. Type: fp32_t'''

    def __init__(self, x = None, y = None, z = None, t = None):
        '''Class constructor
        
        The time offset relative to the previous trajectory point.

       This message class contains the following fields and their respective types:
    x : fp32_t, unit: m

            y : fp32_t, unit: m

            z : fp32_t, unit: m

            t : fp32_t, unit: s'''
        self._x = x
        self._y = y
        self._z = z
        self._t = t


class CustomManeuver(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            name : plaintext, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_name', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 465, stable = None, description = "The Custom Maneuver message may be used as specification of a very specific maneuver not covered by the IMC scope. The settings of the maneuver are just its id, timeout and other settings encoded as a tuple list.", fields = ('timeout', 'name', 'custom',), abbrev = "CustomManeuver", name = "Custom Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run. If the maneuver is not completed in the amount of time specified an error will be generated.")
    '''The amount of time the maneuver is allowed to run. If the maneuver is not completed in the amount of time specified an error will be generated. Type: uint16_t'''
    name = _base.mutable_attr({'name': 'Maneuver Name', 'type': 'plaintext'}, "The maneuver name, used as key by an implementation to bind the maneuver to the corresponding controller.")
    '''The maneuver name, used as key by an implementation to bind the maneuver to the corresponding controller. Type: plaintext'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, name = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            name : plaintext, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._name = name
        self._custom = custom


class VehicleFormation(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            points : message-list, unit: NOT FOUND

            participants : message-list, unit: NOT FOUND

            start_time : fp64_t, unit: s

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_points', '_participants', '_start_time', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 466, stable = None, description = "Coordinate maneuver using two or more cooperating systems.", fields = ('lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'points', 'participants', 'start_time', 'custom',), abbrev = "VehicleFormation", name = "Vehicle Formation", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude for start point.")
    '''WGS-84 Latitude for start point. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude for start point.")
    '''WGS-84 Longitude for start point. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Reference speed.")
    '''Reference speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Reference speed units. Enumerated (Global).")
    '''Reference speed units. Enumerated (Global). Type: uint8_t'''
    points = _base.mutable_attr({'name': 'Trajectory Points', 'type': 'message-list', 'message-type': 'TrajectoryPoint'}, "List of trajectory points.")
    '''List of trajectory points. Type: message-list'''
    participants = _base.mutable_attr({'name': 'Formation Participants', 'type': 'message-list', 'message-type': 'VehicleFormationParticipant'}, "List of formation participants.")
    '''List of formation participants. Type: message-list'''
    start_time = _base.mutable_attr({'name': 'Start Time', 'type': 'fp64_t', 'unit': 's'}, "Optional start time hint for vehicle formation.")
    '''Optional start time hint for vehicle formation. Type: fp64_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, points = None, participants = None, start_time = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            points : message-list, unit: NOT FOUND

            participants : message-list, unit: NOT FOUND

            start_time : fp64_t, unit: s

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._points = points
        self._participants = participants
        self._start_time = start_time
        self._custom = custom


class VehicleFormationParticipant(_base.base_message):
    '''Distance that the system must respect along the zz axis.

       This message class contains the following fields and their respective types:
    vid : uint16_t, unit: NOT FOUND

            off_x : fp32_t, unit: m

            off_y : fp32_t, unit: m

            off_z : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_vid', '_off_x', '_off_y', '_off_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 467, stable = None, description = "Definition of a vehicle participant in a VehicleFormation maneuver.", fields = ('vid', 'off_x', 'off_y', 'off_z',), abbrev = "VehicleFormationParticipant", name = "Vehicle Formation Participant", flags = None)

    vid = _base.mutable_attr({'name': 'ID (IMC address)', 'type': 'uint16_t'}, "IMC address of vehicle.")
    '''IMC address of vehicle. Type: uint16_t'''
    off_x = _base.mutable_attr({'name': 'Formation offset -- Along-track', 'type': 'fp32_t', 'unit': 'm'}, "Distance that the system must respect along the xx axis.")
    '''Distance that the system must respect along the xx axis. Type: fp32_t'''
    off_y = _base.mutable_attr({'name': 'Formation offset -- Cross-track', 'type': 'fp32_t', 'unit': 'm'}, "Distance that the system must respect along the yy axis.")
    '''Distance that the system must respect along the yy axis. Type: fp32_t'''
    off_z = _base.mutable_attr({'name': 'Formation offset -- Depth/Altitude', 'type': 'fp32_t', 'unit': 'm'}, "Distance that the system must respect along the zz axis.")
    '''Distance that the system must respect along the zz axis. Type: fp32_t'''

    def __init__(self, vid = None, off_x = None, off_y = None, off_z = None):
        '''Class constructor
        
        Distance that the system must respect along the zz axis.

       This message class contains the following fields and their respective types:
    vid : uint16_t, unit: NOT FOUND

            off_x : fp32_t, unit: m

            off_y : fp32_t, unit: m

            off_z : fp32_t, unit: m'''
        self._vid = vid
        self._off_x = off_x
        self._off_y = off_y
        self._off_z = off_z


class StopManeuver(_base.base_message):
    '''Command used to stop currently executing maneuver.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Maneuvering", id = 468, stable = None, description = "Command used to stop currently executing maneuver.", fields = [], abbrev = "StopManeuver", name = "Stop Maneuver", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Command used to stop currently executing maneuver.

       This message class contains the following fields and their respective types:
'''


class RegisterManeuver(_base.base_message):
    '''IMC serialization ID of maneuver type.

       This message class contains the following fields and their respective types:
    mid : uint16_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_mid']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Maneuvering", id = 469, stable = None, description = "Command used to indicate maneuver can be executed in the vehicle.", fields = ('mid',), abbrev = "RegisterManeuver", name = "Register Maneuver", flags = None)

    mid = _base.mutable_attr({'name': 'Maneuver ID', 'type': 'uint16_t'}, "IMC serialization ID of maneuver type.")
    '''IMC serialization ID of maneuver type. Type: uint16_t'''

    def __init__(self, mid = None):
        '''Class constructor
        
        IMC serialization ID of maneuver type.

       This message class contains the following fields and their respective types:
    mid : uint16_t, unit: NOT FOUND'''
        self._mid = mid


class ManeuverControlState(_base.base_message):
    '''Complementary information, e.g., regarding errors.

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            eta : uint16_t, unit: s

            info : plaintext, unit: NOT FOUND'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: MCS'''
    
        EXECUTING = 0
        '''Name: Maneuver in progress'''
    
        DONE = 1
        '''Name: Maneuver completed'''
    
        ERROR = 2
        '''Name: Maneuver error'''
    
        STOPPED = 3
        '''Name: Maneuver stopped'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_state', '_eta', '_info']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Maneuvering", id = 470, stable = None, description = "Maneuver control state.", fields = ('state', 'eta', 'info',), abbrev = "ManeuverControlState", name = "Maneuver Control State", flags = None)

    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'MCS'}, "Code indicating maneuver state. Enumerated (Local).")
    '''Code indicating maneuver state. Enumerated (Local). Type: uint8_t'''
    eta = _base.mutable_attr({'name': 'Completion Time', 'type': 'uint16_t', 'unit': 's'}, "Estimated time to completion of the maneuver, when executing. The value will be 65535 if the time is unknown or undefined.")
    '''Estimated time to completion of the maneuver, when executing. The value will be 65535 if the time is unknown or undefined. Type: uint16_t'''
    info = _base.mutable_attr({'name': 'Info', 'type': 'plaintext'}, "Complementary information, e.g., regarding errors.")
    '''Complementary information, e.g., regarding errors. Type: plaintext'''

    def __init__(self, state = None, eta = None, info = None):
        '''Class constructor
        
        Complementary information, e.g., regarding errors.

       This message class contains the following fields and their respective types:
    state : uint8_t, unit: Enumerated (Local)

            eta : uint16_t, unit: s

            info : plaintext, unit: NOT FOUND'''
        self._state = state
        self._eta = eta
        self._info = info


class FollowSystem(_base.base_message):
    '''Units of the z reference. Enumerated (Global).

       This message class contains the following fields and their respective types:
    system : uint16_t, unit: NOT FOUND

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            x : fp32_t, unit: NOT FOUND

            y : fp32_t, unit: NOT FOUND

            z : fp32_t, unit: NOT FOUND

            z_units : uint8_t, unit: Enumerated (Global)'''

    __slots__ = ['_Attributes', '_header', '_footer', '_system', '_duration', '_speed', '_speed_units', '_x', '_y', '_z', '_z_units']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Maneuvering", id = 471, stable = None, description = "System-following maneuver.", fields = ('system', 'duration', 'speed', 'speed_units', 'x', 'y', 'z', 'z_units',), abbrev = "FollowSystem", name = "Follow System", flags = None)

    system = _base.mutable_attr({'name': 'System To Follow', 'type': 'uint16_t'}, "IMC address of system to follow.")
    '''IMC address of system to follow. Type: uint16_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Duration of maneuver, 0 for unlimited duration.")
    '''Duration of maneuver, 0 for unlimited duration. Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Reference speed.")
    '''Reference speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Reference speed units. Enumerated (Global).")
    '''Reference speed units. Enumerated (Global). Type: uint8_t'''
    x = _base.mutable_attr({'name': 'Offset -- X', 'type': 'fp32_t'}, "Along-track offset.")
    '''Along-track offset. Type: fp32_t'''
    y = _base.mutable_attr({'name': 'Offset -- Y', 'type': 'fp32_t'}, "Cross-track offset.")
    '''Cross-track offset. Type: fp32_t'''
    z = _base.mutable_attr({'name': 'Coordinate -- Z', 'type': 'fp32_t'}, "Coordinate z during follow maneuver. Use z_units to specify whether z represents depth, altitude or other.")
    '''Coordinate z during follow maneuver. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''

    def __init__(self, system = None, duration = None, speed = None, speed_units = None, x = None, y = None, z = None, z_units = None):
        '''Class constructor
        
        Units of the z reference. Enumerated (Global).

       This message class contains the following fields and their respective types:
    system : uint16_t, unit: NOT FOUND

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            x : fp32_t, unit: NOT FOUND

            y : fp32_t, unit: NOT FOUND

            z : fp32_t, unit: NOT FOUND

            z_units : uint8_t, unit: Enumerated (Global)'''
        self._system = system
        self._duration = duration
        self._speed = speed
        self._speed_units = speed_units
        self._x = x
        self._y = y
        self._z = z
        self._z_units = z_units


class CommsRelay(_base.base_message):
    '''Move only if the distance to the target is bigger than this threshold.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            duration : uint16_t, unit: s

            sys_a : uint16_t, unit: NOT FOUND

            sys_b : uint16_t, unit: NOT FOUND

            move_threshold : fp32_t, unit: m'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_speed', '_speed_units', '_duration', '_sys_a', '_sys_b', '_move_threshold']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 472, stable = None, description = "In this maneuver, a vehicle drives to the center of two other systems (a, b) in order to be used as a communications relay.", fields = ('lat', 'lon', 'speed', 'speed_units', 'duration', 'sys_a', 'sys_b', 'move_threshold',), abbrev = "CommsRelay", name = "Communications Relay", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude for start point.")
    '''WGS-84 Latitude for start point. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude for start point.")
    '''WGS-84 Longitude for start point. Type: fp64_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Reference speed.")
    '''Reference speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Reference speed units. Enumerated (Global).")
    '''Reference speed units. Enumerated (Global). Type: uint8_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Duration of maneuver, 0 for unlimited duration.")
    '''Duration of maneuver, 0 for unlimited duration. Type: uint16_t'''
    sys_a = _base.mutable_attr({'name': 'System A', 'type': 'uint16_t'}, "The IMC id of the system A.")
    '''The IMC id of the system A. Type: uint16_t'''
    sys_b = _base.mutable_attr({'name': 'System B', 'type': 'uint16_t'}, "The IMC id of the system B.")
    '''The IMC id of the system B. Type: uint16_t'''
    move_threshold = _base.mutable_attr({'name': 'Move threshold', 'type': 'fp32_t', 'unit': 'm'}, "Move only if the distance to the target is bigger than this threshold.")
    '''Move only if the distance to the target is bigger than this threshold. Type: fp32_t'''

    def __init__(self, lat = None, lon = None, speed = None, speed_units = None, duration = None, sys_a = None, sys_b = None, move_threshold = None):
        '''Class constructor
        
        Move only if the distance to the target is bigger than this threshold.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            duration : uint16_t, unit: s

            sys_a : uint16_t, unit: NOT FOUND

            sys_b : uint16_t, unit: NOT FOUND

            move_threshold : fp32_t, unit: m'''
        self._lat = lat
        self._lon = lon
        self._speed = speed
        self._speed_units = speed_units
        self._duration = duration
        self._sys_a = sys_a
        self._sys_b = sys_b
        self._move_threshold = move_threshold


class CoverArea(_base.base_message):
    '''Additional parameters to be used by the controller.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            polygon : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_polygon', '_custom']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 473, stable = None, description = "Given a polygonal area, generates trajectories to cover the area.", fields = ('lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'polygon', 'custom',), abbrev = "CoverArea", name = "Cover Area", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude for start point.")
    '''WGS-84 Latitude for start point. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude for start point.")
    '''WGS-84 Longitude for start point. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Reference speed.")
    '''Reference speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Reference speed units. Enumerated (Global).")
    '''Reference speed units. Enumerated (Global). Type: uint8_t'''
    polygon = _base.mutable_attr({'name': 'Polygon', 'type': 'message-list', 'message-type': 'PolygonVertex'}, "Message list of type PolygonVertex.")
    '''Message list of type PolygonVertex. Type: message-list'''
    custom = _base.mutable_attr({'name': 'CustomParameters', 'type': 'plaintext', 'unit': 'TupleList'}, "Additional parameters to be used by the controller.")
    '''Additional parameters to be used by the controller. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, polygon = None, custom = None):
        '''Class constructor
        
        Additional parameters to be used by the controller.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            polygon : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._polygon = polygon
        self._custom = custom


class PolygonVertex(_base.base_message):
    '''WGS-84 Longitude for start point.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 474, stable = None, description = "This message is used to store the various polygon vertices for CoverArea maneuvers.", fields = ('lat', 'lon',), abbrev = "PolygonVertex", name = "Polygon Vertex", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude for start point.")
    '''WGS-84 Latitude for start point. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude for start point.")
    '''WGS-84 Longitude for start point. Type: fp64_t'''

    def __init__(self, lat = None, lon = None):
        '''Class constructor
        
        WGS-84 Longitude for start point.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad'''
        self._lat = lat
        self._lon = lon


class CompassCalibration(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            pitch : fp32_t, unit: rad

            amplitude : fp32_t, unit: m

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''

    class DIRECTION(_enum.IntEnum):
        '''Full name: Direction
        Prefix: LD'''
    
        VDEP = 0
        '''Name: Vehicle Dependent'''
    
        CLOCKW = 1
        '''Name: Clockwise'''
    
        CCLOCKW = 2
        '''Name: Counter Clockwise'''
    
        IWINDCURR = 3
        '''Name: Into the wind/current'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_pitch', '_amplitude', '_duration', '_speed', '_speed_units', '_radius', '_direction', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 475, stable = None, description = "This maneuver is a mix between the Loiter maneuver and the YoYo maneuver. The vehicle cirlcles around a specific waypoint with a variable Z reference between a minimum and maximum value.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'pitch', 'amplitude', 'duration', 'speed', 'speed_units', 'radius', 'direction', 'custom',), abbrev = "CompassCalibration", name = "Compass Calibration Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The timeout indicates the time that an error should occur if exceeded.")
    '''The timeout indicates the time that an error should occur if exceeded. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate.")
    '''WGS-84 Latitude coordinate. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate.")
    '''WGS-84 Longitude coordinate. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    pitch = _base.mutable_attr({'name': 'Pitch', 'type': 'fp32_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "Pitch angle used to perform the maneuver.")
    '''Pitch angle used to perform the maneuver. Type: fp32_t'''
    amplitude = _base.mutable_attr({'name': 'Amplitude', 'type': 'fp32_t', 'unit': 'm'}, "Yoyo motion amplitude.")
    '''Yoyo motion amplitude. Type: fp32_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "The duration of this maneuver. Use '0' for unlimited duration time.")
    '''The duration of this maneuver. Use '0' for unlimited duration time. Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed.")
    '''Maneuver speed. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t', 'unit': 'm', 'min': 1, 'max': 100000}, "Radius of the maneuver.")
    '''Radius of the maneuver. Type: fp32_t'''
    direction = _base.mutable_attr({'name': 'Direction', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'LD', 'max': 3}, "Direction of the maneuver. Enumerated (Local).")
    '''Direction of the maneuver. Enumerated (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, pitch = None, amplitude = None, duration = None, speed = None, speed_units = None, radius = None, direction = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            pitch : fp32_t, unit: rad

            amplitude : fp32_t, unit: m

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._pitch = pitch
        self._amplitude = amplitude
        self._duration = duration
        self._speed = speed
        self._speed_units = speed_units
        self._radius = radius
        self._direction = direction
        self._custom = custom


class FormationParameters(_base.base_message):
    '''Custom settings for the formation configuration.

       This message class contains the following fields and their respective types:
    formation_name : plaintext, unit: NOT FOUND

            reference_frame : uint8_t, unit: Enumerated (Local)

            participants : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''

    class REFERENCE_FRAME(_enum.IntEnum):
        '''Full name: Formation Reference Frame
        Prefix: OP'''
    
        EARTH_FIXED = 0
        '''Name: Earth Fixed'''
    
        PATH_FIXED = 1
        '''Name: Path Fixed'''
    
        PATH_CURVED = 2
        '''Name: Path Curved'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_formation_name', '_reference_frame', '_participants', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 476, stable = None, description = "A \"Formation\" is defined by the relative positions of the vehicles inside the formation, and the reference frame where this positions are defined. The formation reference frame may be: - Earth Fixed: Where the vehicles relative position do not depend on the followed path. This results in all UAVs following the same path with an offset relative to each other; - Path Fixed: Where the vehicles relative position depends on the followed path, changing the inter-vehicle offset direction with the path direction. - Path Curved: Where the vehicles relative position depends on the followed path, changing the inter-vehicle offset direction with the path direction and direction change rate. An offset in the xx axis results in a distance over the curved path line. An offset in the yy axis results in an offset of the vehicle path line relative to the formation center path line.", fields = ('formation_name', 'reference_frame', 'participants', 'custom',), abbrev = "FormationParameters", name = "Formation Parameters", flags = None)

    formation_name = _base.mutable_attr({'name': 'Formation Name', 'type': 'plaintext'}, "Name of the formation configuration.")
    '''Name of the formation configuration. Type: plaintext'''
    reference_frame = _base.mutable_attr({'name': 'Formation Reference Frame', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Formation reference frame Enumerated (Local).")
    '''Formation reference frame Enumerated (Local). Type: uint8_t'''
    participants = _base.mutable_attr({'name': 'Formation Participants', 'type': 'message-list', 'message-type': 'VehicleFormationParticipant'}, "List of formation participants.")
    '''List of formation participants. Type: message-list'''
    custom = _base.mutable_attr({'name': 'Custom settings for formation', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for the formation configuration.")
    '''Custom settings for the formation configuration. Type: plaintext'''

    def __init__(self, formation_name = None, reference_frame = None, participants = None, custom = None):
        '''Class constructor
        
        Custom settings for the formation configuration.

       This message class contains the following fields and their respective types:
    formation_name : plaintext, unit: NOT FOUND

            reference_frame : uint8_t, unit: Enumerated (Local)

            participants : message-list, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''
        self._formation_name = formation_name
        self._reference_frame = reference_frame
        self._participants = participants
        self._custom = custom


class FormationPlanExecution(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    group_name : plaintext, unit: NOT FOUND

            formation_name : plaintext, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            description : plaintext, unit: NOT FOUND

            leader_speed : fp32_t, unit: m/s

            leader_bank_lim : fp32_t, unit: m/s

            pos_sim_err_lim : fp32_t, unit: m

            pos_sim_err_wrn : fp32_t, unit: m

            pos_sim_err_timeout : uint16_t, unit: s

            converg_max : fp32_t, unit: m

            converg_timeout : uint16_t, unit: s

            comms_timeout : uint16_t, unit: s

            turb_lim : fp32_t, unit: m/s

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_group_name', '_formation_name', '_plan_id', '_description', '_leader_speed', '_leader_bank_lim', '_pos_sim_err_lim', '_pos_sim_err_wrn', '_pos_sim_err_timeout', '_converg_max', '_converg_timeout', '_comms_timeout', '_turb_lim', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 477, stable = None, description = "A \"Formation Plan\" is a maneuver specifying a plan for a team of vehicles. The maneuver defines: - Vehicles included in the formation group - Formation shape configuration - Plan (set of maneuvers) to be followed by the formation center - Speed at which that plan is followed - Path contrains (virtual leader bank limit) - Supervision settings", fields = ('group_name', 'formation_name', 'plan_id', 'description', 'leader_speed', 'leader_bank_lim', 'pos_sim_err_lim', 'pos_sim_err_wrn', 'pos_sim_err_timeout', 'converg_max', 'converg_timeout', 'comms_timeout', 'turb_lim', 'custom',), abbrev = "FormationPlanExecution", name = "Formation Plan Execution", flags = None)

    group_name = _base.mutable_attr({'name': 'Target Group Name', 'type': 'plaintext'}, "Target group for the formation plan.")
    '''Target group for the formation plan. Type: plaintext'''
    formation_name = _base.mutable_attr({'name': 'Formation Name', 'type': 'plaintext'}, "Name of the formation configuration.")
    '''Name of the formation configuration. Type: plaintext'''
    plan_id = _base.mutable_attr({'name': 'Formation Plan ID', 'type': 'plaintext'}, "The flight plan's identifier. Flight plan defined to be tracked by the formation leader.")
    '''The flight plan's identifier. Flight plan defined to be tracked by the formation leader. Type: plaintext'''
    description = _base.mutable_attr({'name': 'Plan Description', 'type': 'plaintext'}, "Verbose text description of plan.")
    '''Verbose text description of plan. Type: plaintext'''
    leader_speed = _base.mutable_attr({'name': 'Formation Leader Flight Airspeed', 'type': 'fp32_t', 'unit': 'm/s'}, "Formation leader flight airspeed during the plan tracking.")
    '''Formation leader flight airspeed during the plan tracking. Type: fp32_t'''
    leader_bank_lim = _base.mutable_attr({'name': 'Formation leader flight bank limit', 'type': 'fp32_t', 'unit': 'm/s'}, "Formation leader flight bank limit during the plan tracking.")
    '''Formation leader flight bank limit during the plan tracking. Type: fp32_t'''
    pos_sim_err_lim = _base.mutable_attr({'name': 'Position mismatch limit', 'type': 'fp32_t', 'unit': 'm'}, "Limit for the position mismatch between real and simulated position, before a maneuver abort is asserted.")
    '''Limit for the position mismatch between real and simulated position, before a maneuver abort is asserted. Type: fp32_t'''
    pos_sim_err_wrn = _base.mutable_attr({'name': 'Position mismatch threshold', 'type': 'fp32_t', 'unit': 'm'}, "Warning threshold for the position mismatch between real and simulated position. Above this threshold a time-out limit is evaluated to assert a maneuver abort state.")
    '''Warning threshold for the position mismatch between real and simulated position. Above this threshold a time-out limit is evaluated to assert a maneuver abort state. Type: fp32_t'''
    pos_sim_err_timeout = _base.mutable_attr({'name': 'Position mismatch time-out', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run after the position mismatch threshold is surpassed.")
    '''The amount of time the maneuver is allowed to run after the position mismatch threshold is surpassed. Type: uint16_t'''
    converg_max = _base.mutable_attr({'name': 'Convergence threshold', 'type': 'fp32_t', 'unit': 'm'}, "Threshold for the convergence measure, above which a time-out limit is evaluated to assert a maneuver abort state.")
    '''Threshold for the convergence measure, above which a time-out limit is evaluated to assert a maneuver abort state. Type: fp32_t'''
    converg_timeout = _base.mutable_attr({'name': 'Convergence time-out', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run after the convergence threshold is surpassed.")
    '''The amount of time the maneuver is allowed to run after the convergence threshold is surpassed. Type: uint16_t'''
    comms_timeout = _base.mutable_attr({'name': 'Communications time-out', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run without any update on the other formation vehicles states.")
    '''The amount of time the maneuver is allowed to run without any update on the other formation vehicles states. Type: uint16_t'''
    turb_lim = _base.mutable_attr({'name': 'Turbulence limit', 'type': 'fp32_t', 'unit': 'm/s'}, "Turbulence limit above which a maneuver abort is asserted.")
    '''Turbulence limit above which a maneuver abort is asserted. Type: fp32_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, group_name = None, formation_name = None, plan_id = None, description = None, leader_speed = None, leader_bank_lim = None, pos_sim_err_lim = None, pos_sim_err_wrn = None, pos_sim_err_timeout = None, converg_max = None, converg_timeout = None, comms_timeout = None, turb_lim = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    group_name : plaintext, unit: NOT FOUND

            formation_name : plaintext, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            description : plaintext, unit: NOT FOUND

            leader_speed : fp32_t, unit: m/s

            leader_bank_lim : fp32_t, unit: m/s

            pos_sim_err_lim : fp32_t, unit: m

            pos_sim_err_wrn : fp32_t, unit: m

            pos_sim_err_timeout : uint16_t, unit: s

            converg_max : fp32_t, unit: m

            converg_timeout : uint16_t, unit: s

            comms_timeout : uint16_t, unit: s

            turb_lim : fp32_t, unit: m/s

            custom : plaintext, unit: TupleList'''
        self._group_name = group_name
        self._formation_name = formation_name
        self._plan_id = plan_id
        self._description = description
        self._leader_speed = leader_speed
        self._leader_bank_lim = leader_bank_lim
        self._pos_sim_err_lim = pos_sim_err_lim
        self._pos_sim_err_wrn = pos_sim_err_wrn
        self._pos_sim_err_timeout = pos_sim_err_timeout
        self._converg_max = converg_max
        self._converg_timeout = converg_timeout
        self._comms_timeout = comms_timeout
        self._turb_lim = turb_lim
        self._custom = custom


class FollowReference(_base.base_message):
    '''Similarly to Loiter Radius, this field is used to define the \"z\" distance considered to be inside the vacitiny of the target location. An AUV may, for instance, be floating until it more than z units above the current reference, in which case it actively changes its position in order to achieve the desired depth / altitude.

       This message class contains the following fields and their respective types:
    control_src : uint16_t, unit: NOT FOUND

            control_ent : uint8_t, unit: NOT FOUND

            timeout : fp32_t, unit: NOT FOUND

            loiter_radius : fp32_t, unit: NOT FOUND

            altitude_interval : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_control_src', '_control_ent', '_timeout', '_loiter_radius', '_altitude_interval']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 478, stable = None, description = "This maneuver follows a reference given by an external entity.", fields = ('control_src', 'control_ent', 'timeout', 'loiter_radius', 'altitude_interval',), abbrev = "FollowReference", name = "Follow Reference Maneuver", flags = None)

    control_src = _base.mutable_attr({'name': 'Controlling Source', 'type': 'uint16_t'}, "The IMC identifier of the source system that is allowed to provide references to this maneuver. If the value ''0xFFFF'' is used, any system is allowed to command references.")
    '''The IMC identifier of the source system that is allowed to provide references to this maneuver. If the value ''0xFFFF'' is used, any system is allowed to command references. Type: uint16_t'''
    control_ent = _base.mutable_attr({'name': 'Controlling Entity', 'type': 'uint8_t'}, "The entity identifier of the entity that is allowed to provide references to this maneuver. If the value ''0xFF'' is used, any entity is allowed to command references.")
    '''The entity identifier of the entity that is allowed to provide references to this maneuver. If the value ''0xFF'' is used, any entity is allowed to command references. Type: uint8_t'''
    timeout = _base.mutable_attr({'name': 'Reference Update Timeout', 'type': 'fp32_t'}, "The ammount of time, in seconds, after which the maneuver will be terminated if no reference has been received. In other words, the controlling entity should send reference updates in shorter periods than 'timeout'.")
    '''The ammount of time, in seconds, after which the maneuver will be terminated if no reference has been received. In other words, the controlling entity should send reference updates in shorter periods than 'timeout'. Type: fp32_t'''
    loiter_radius = _base.mutable_attr({'name': 'Loiter Radius', 'type': 'fp32_t'}, "Whenever an intended reference is achieved, this maneuver will maintain the vehicle in vaticiny of that location. The loiter radius is used to define the radius of this (xy) area.")
    '''Whenever an intended reference is achieved, this maneuver will maintain the vehicle in vaticiny of that location. The loiter radius is used to define the radius of this (xy) area. Type: fp32_t'''
    altitude_interval = _base.mutable_attr({'name': 'Altitude Interval', 'type': 'fp32_t'}, "Similarly to Loiter Radius, this field is used to define the \"z\" distance considered to be inside the vacitiny of the target location. An AUV may, for instance, be floating until it more than z units above the current reference, in which case it actively changes its position in order to achieve the desired depth / altitude.")
    '''Similarly to Loiter Radius, this field is used to define the \"z\" distance considered to be inside the vacitiny of the target location. An AUV may, for instance, be floating until it more than z units above the current reference, in which case it actively changes its position in order to achieve the desired depth / altitude. Type: fp32_t'''

    def __init__(self, control_src = None, control_ent = None, timeout = None, loiter_radius = None, altitude_interval = None):
        '''Class constructor
        
        Similarly to Loiter Radius, this field is used to define the \"z\" distance considered to be inside the vacitiny of the target location. An AUV may, for instance, be floating until it more than z units above the current reference, in which case it actively changes its position in order to achieve the desired depth / altitude.

       This message class contains the following fields and their respective types:
    control_src : uint16_t, unit: NOT FOUND

            control_ent : uint8_t, unit: NOT FOUND

            timeout : fp32_t, unit: NOT FOUND

            loiter_radius : fp32_t, unit: NOT FOUND

            altitude_interval : fp32_t, unit: NOT FOUND'''
        self._control_src = control_src
        self._control_ent = control_ent
        self._timeout = timeout
        self._loiter_radius = loiter_radius
        self._altitude_interval = altitude_interval


class Reference(_base.base_message):
    '''No description available

       This message class contains the following fields and their respective types:
    flags : uint8_t, unit: Bitfield (Local)

            speed : message, unit: NOT FOUND

            z : message, unit: NOT FOUND

            lat : fp64_t, unit: NOT FOUND

            lon : fp64_t, unit: NOT FOUND

            radius : fp32_t, unit: NOT FOUND'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLAG'''
    
        EMPTY = 0
        '''No active flags'''
    
        LOCATION = 1
        '''Name: Use Location Reference'''
    
        SPEED = 2
        '''Name: Use Speed Reference'''
    
        Z = 4
        '''Name: Use Z Reference'''
    
        RADIUS = 8
        '''Name: Use Radius Reference'''
    
        START_POINT = 16
        '''Name: Use this Reference as Start Position for PathControler'''
    
        DIRECT = 32
        '''Name: Use Current Position as Start Position for PathControler'''
    
        MANDONE = 128
        '''Name: Flag Maneuver Completion'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_flags', '_speed', '_z', '_lat', '_lon', '_radius']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Maneuvering", id = 479, stable = None, description = None, fields = ('flags', 'speed', 'z', 'lat', 'lon', 'radius',), abbrev = "Reference", name = "Reference To Follow", flags = None)

    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FLAG'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed Reference', 'type': 'message', 'message-type': 'DesiredSpeed'}, "No description available")
    '''No description available Type: message'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'message', 'message-type': 'DesiredZ'}, "No description available")
    '''No description available Type: message'''
    lat = _base.mutable_attr({'name': 'Latitude Reference', 'type': 'fp64_t'}, "No description available")
    '''No description available Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude Reference', 'type': 'fp64_t'}, "No description available")
    '''No description available Type: fp64_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t'}, "No description available")
    '''No description available Type: fp32_t'''

    def __init__(self, flags = None, speed = None, z = None, lat = None, lon = None, radius = None):
        '''Class constructor
        
        No description available

       This message class contains the following fields and their respective types:
    flags : uint8_t, unit: Bitfield (Local)

            speed : message, unit: NOT FOUND

            z : message, unit: NOT FOUND

            lat : fp64_t, unit: NOT FOUND

            lon : fp64_t, unit: NOT FOUND

            radius : fp32_t, unit: NOT FOUND'''
        self._flags = flags
        self._speed = speed
        self._z = z
        self._lat = lat
        self._lon = lon
        self._radius = radius


class FollowRefState(_base.base_message):
    '''No description available Bitfield (Local).

       This message class contains the following fields and their respective types:
    control_src : uint16_t, unit: NOT FOUND

            control_ent : uint8_t, unit: NOT FOUND

            reference : message, unit: NOT FOUND

            state : uint8_t, unit: Enumerated (Local)

            proximity : uint8_t, unit: Bitfield (Local)'''

    class STATE(_enum.IntEnum):
        '''Full name: State
        Prefix: FR'''
    
        WAIT = 1
        '''Name: Waiting for first reference'''
    
        GOTO = 2
        '''Name: Going towards received reference'''
    
        LOITER = 3
        '''Name: Loitering after arriving at the reference'''
    
        HOVER = 4
        '''Name: Hovering after arriving at the reference'''
    
        ELEVATOR = 5
        '''Name: Moving in z after arriving at the target cylinder'''
    
        TIMEOUT = 6
        '''Name: Controlling system timed out'''
    
    
    class PROXIMITY(_enum.IntFlag):
        '''Full name: Proximity
        Prefix: PROX'''
    
        EMPTY = 0
        '''No active flags'''
    
        FAR = 1
        '''Name: Far from the destination'''
    
        XY_NEAR = 2
        '''Name: Near in the horizontal plane'''
    
        Z_NEAR = 4
        '''Name: Near in the vertical plane'''
    
        XY_UNREACHABLE = 8
        '''Name: Unreachable in the horizontal plane'''
    
        Z_UNREACHABLE = 16
        '''Name: Unreachable in the vertical plane'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_control_src', '_control_ent', '_reference', '_state', '_proximity']
    Attributes = _base.MessageAttributes(source = "vehicle", usedby = None, category = "Maneuvering", id = 480, stable = None, description = None, fields = ('control_src', 'control_ent', 'reference', 'state', 'proximity',), abbrev = "FollowRefState", name = "Follow Reference State", flags = None)

    control_src = _base.mutable_attr({'name': 'Controlling Source', 'type': 'uint16_t'}, "The IMC identifier of the source system that is allowed to control the vehicle. If the value ''0xFFFF'' is used, any system is allowed to command references.")
    '''The IMC identifier of the source system that is allowed to control the vehicle. If the value ''0xFFFF'' is used, any system is allowed to command references. Type: uint16_t'''
    control_ent = _base.mutable_attr({'name': 'Controlling Entity', 'type': 'uint8_t'}, "The entity identifier of the entity that is allowed to control the vehicle. If the value ''0xFF'' is used, any entity is allowed to command references.")
    '''The entity identifier of the entity that is allowed to control the vehicle. If the value ''0xFF'' is used, any entity is allowed to command references. Type: uint8_t'''
    reference = _base.mutable_attr({'name': 'Reference', 'type': 'message', 'message-type': 'Reference'}, "Reference currently being followed.")
    '''Reference currently being followed. Type: message'''
    state = _base.mutable_attr({'name': 'State', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'FR'}, "No description available Enumerated (Local).")
    '''No description available Enumerated (Local). Type: uint8_t'''
    proximity = _base.mutable_attr({'name': 'Proximity', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'PROX'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''

    def __init__(self, control_src = None, control_ent = None, reference = None, state = None, proximity = None):
        '''Class constructor
        
        No description available Bitfield (Local).

       This message class contains the following fields and their respective types:
    control_src : uint16_t, unit: NOT FOUND

            control_ent : uint8_t, unit: NOT FOUND

            reference : message, unit: NOT FOUND

            state : uint8_t, unit: Enumerated (Local)

            proximity : uint8_t, unit: Bitfield (Local)'''
        self._control_src = control_src
        self._control_ent = control_ent
        self._reference = reference
        self._state = state
        self._proximity = proximity


class FormationMonitor(_base.base_message):
    '''List of RelativeState messages, encoding the inter-vehicle formation state.

       This message class contains the following fields and their respective types:
    ax_cmd : fp32_t, unit: NOT FOUND

            ay_cmd : fp32_t, unit: NOT FOUND

            az_cmd : fp32_t, unit: NOT FOUND

            ax_des : fp32_t, unit: NOT FOUND

            ay_des : fp32_t, unit: NOT FOUND

            az_des : fp32_t, unit: NOT FOUND

            virt_err_x : fp32_t, unit: NOT FOUND

            virt_err_y : fp32_t, unit: NOT FOUND

            virt_err_z : fp32_t, unit: NOT FOUND

            surf_fdbk_x : fp32_t, unit: NOT FOUND

            surf_fdbk_y : fp32_t, unit: NOT FOUND

            surf_fdbk_z : fp32_t, unit: NOT FOUND

            surf_unkn_x : fp32_t, unit: NOT FOUND

            surf_unkn_y : fp32_t, unit: NOT FOUND

            surf_unkn_z : fp32_t, unit: NOT FOUND

            ss_x : fp32_t, unit: NOT FOUND

            ss_y : fp32_t, unit: NOT FOUND

            ss_z : fp32_t, unit: NOT FOUND

            rel_state : message-list, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_ax_cmd', '_ay_cmd', '_az_cmd', '_ax_des', '_ay_des', '_az_des', '_virt_err_x', '_virt_err_y', '_virt_err_z', '_surf_fdbk_x', '_surf_fdbk_y', '_surf_fdbk_z', '_surf_unkn_x', '_surf_unkn_y', '_surf_unkn_z', '_ss_x', '_ss_y', '_ss_z', '_rel_state']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 481, stable = None, description = "Monitoring variables for the formation state and performance.", fields = ('ax_cmd', 'ay_cmd', 'az_cmd', 'ax_des', 'ay_des', 'az_des', 'virt_err_x', 'virt_err_y', 'virt_err_z', 'surf_fdbk_x', 'surf_fdbk_y', 'surf_fdbk_z', 'surf_unkn_x', 'surf_unkn_y', 'surf_unkn_z', 'ss_x', 'ss_y', 'ss_z', 'rel_state',), abbrev = "FormationMonitor", name = "Formation Monitoring Data", flags = None)

    ax_cmd = _base.mutable_attr({'name': 'Commanded X Acceleration (North)', 'type': 'fp32_t'}, "Commanded acceleration computed by the formation controller: northward direction. On the vehicle directional reference frame. Constrained by the vehicle operational limits.")
    '''Commanded acceleration computed by the formation controller: northward direction. On the vehicle directional reference frame. Constrained by the vehicle operational limits. Type: fp32_t'''
    ay_cmd = _base.mutable_attr({'name': 'Commanded Y Acceleration (East)', 'type': 'fp32_t'}, "Commanded acceleration computed by the formation controller: eastward direction. On the vehicle directional reference frame. Constrained by the vehicle operational limits.")
    '''Commanded acceleration computed by the formation controller: eastward direction. On the vehicle directional reference frame. Constrained by the vehicle operational limits. Type: fp32_t'''
    az_cmd = _base.mutable_attr({'name': 'Commanded Z Acceleration (Down)', 'type': 'fp32_t'}, "Commanded acceleration computed by the formation controller: downward direction. On the vehicle directional reference frame. Constrained by the vehicle operational limits.")
    '''Commanded acceleration computed by the formation controller: downward direction. On the vehicle directional reference frame. Constrained by the vehicle operational limits. Type: fp32_t'''
    ax_des = _base.mutable_attr({'name': 'Desired X Acceleration (North)', 'type': 'fp32_t'}, "Desired acceleration computed by the formation controller: northward direction. On the fixed reference frame.")
    '''Desired acceleration computed by the formation controller: northward direction. On the fixed reference frame. Type: fp32_t'''
    ay_des = _base.mutable_attr({'name': 'Desired Y Acceleration (East)', 'type': 'fp32_t'}, "Desired acceleration computed by the formation controller: eastward direction. On the fixed reference frame.")
    '''Desired acceleration computed by the formation controller: eastward direction. On the fixed reference frame. Type: fp32_t'''
    az_des = _base.mutable_attr({'name': 'Desired Z Acceleration (Down)', 'type': 'fp32_t'}, "Desired acceleration computed by the formation controller: downward direction. On the fixed reference frame.")
    '''Desired acceleration computed by the formation controller: downward direction. On the fixed reference frame. Type: fp32_t'''
    virt_err_x = _base.mutable_attr({'name': 'X Virtual Error (North)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Overall formation combined virtual error: northward direction. On the fixed reference frame.")
    '''Components of the vehicle desired acceleration. Overall formation combined virtual error: northward direction. On the fixed reference frame. Type: fp32_t'''
    virt_err_y = _base.mutable_attr({'name': 'Y Virtual Error (East)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Overall formation combined virtual error: eastward direction. On the fixed reference frame.")
    '''Components of the vehicle desired acceleration. Overall formation combined virtual error: eastward direction. On the fixed reference frame. Type: fp32_t'''
    virt_err_z = _base.mutable_attr({'name': 'Z Virtual Error (Down)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Overall formation combined virtual error: downward direction. On the fixed reference frame.")
    '''Components of the vehicle desired acceleration. Overall formation combined virtual error: downward direction. On the fixed reference frame. Type: fp32_t'''
    surf_fdbk_x = _base.mutable_attr({'name': 'X Sliding Surface Feedback (North)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Overall formation combined sliding surface feedback: northward direction. On the fixed reference frame.")
    '''Components of the vehicle desired acceleration. Overall formation combined sliding surface feedback: northward direction. On the fixed reference frame. Type: fp32_t'''
    surf_fdbk_y = _base.mutable_attr({'name': 'Y Sliding Surface Feedback (East)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Overall formation combined sliding surface feedback: eastward direction. On the fixed reference frame.")
    '''Components of the vehicle desired acceleration. Overall formation combined sliding surface feedback: eastward direction. On the fixed reference frame. Type: fp32_t'''
    surf_fdbk_z = _base.mutable_attr({'name': 'Z Sliding Surface Feedback (Down)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Overall formation combined sliding surface feedback: downward direction. On the fixed reference frame.")
    '''Components of the vehicle desired acceleration. Overall formation combined sliding surface feedback: downward direction. On the fixed reference frame. Type: fp32_t'''
    surf_unkn_x = _base.mutable_attr({'name': 'X Uncertainty Compensation (North)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Dynamics uncertainty compensation: northward direction.")
    '''Components of the vehicle desired acceleration. Dynamics uncertainty compensation: northward direction. Type: fp32_t'''
    surf_unkn_y = _base.mutable_attr({'name': 'Y Uncertainty Compensation (East)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Dynamics uncertainty compensation: eastward direction.")
    '''Components of the vehicle desired acceleration. Dynamics uncertainty compensation: eastward direction. Type: fp32_t'''
    surf_unkn_z = _base.mutable_attr({'name': 'Z Uncertainty Compensation (Down)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Dynamics uncertainty compensation: downward direction.")
    '''Components of the vehicle desired acceleration. Dynamics uncertainty compensation: downward direction. Type: fp32_t'''
    ss_x = _base.mutable_attr({'name': 'X Convergence Deviation (North)', 'type': 'fp32_t'}, "Combined deviation from convergence (sliding surface): North component.")
    '''Combined deviation from convergence (sliding surface): North component. Type: fp32_t'''
    ss_y = _base.mutable_attr({'name': 'Y Convergence Deviation (East)', 'type': 'fp32_t'}, "Combined deviation from convergence (sliding surface): East component.")
    '''Combined deviation from convergence (sliding surface): East component. Type: fp32_t'''
    ss_z = _base.mutable_attr({'name': 'Z Convergence Deviation (Down)', 'type': 'fp32_t'}, "Combined deviation from convergence (sliding surface): Down component.")
    '''Combined deviation from convergence (sliding surface): Down component. Type: fp32_t'''
    rel_state = _base.mutable_attr({'name': 'Relative State', 'type': 'message-list', 'message-type': 'RelativeState'}, "List of RelativeState messages, encoding the inter-vehicle formation state.")
    '''List of RelativeState messages, encoding the inter-vehicle formation state. Type: message-list'''

    def __init__(self, ax_cmd = None, ay_cmd = None, az_cmd = None, ax_des = None, ay_des = None, az_des = None, virt_err_x = None, virt_err_y = None, virt_err_z = None, surf_fdbk_x = None, surf_fdbk_y = None, surf_fdbk_z = None, surf_unkn_x = None, surf_unkn_y = None, surf_unkn_z = None, ss_x = None, ss_y = None, ss_z = None, rel_state = None):
        '''Class constructor
        
        List of RelativeState messages, encoding the inter-vehicle formation state.

       This message class contains the following fields and their respective types:
    ax_cmd : fp32_t, unit: NOT FOUND

            ay_cmd : fp32_t, unit: NOT FOUND

            az_cmd : fp32_t, unit: NOT FOUND

            ax_des : fp32_t, unit: NOT FOUND

            ay_des : fp32_t, unit: NOT FOUND

            az_des : fp32_t, unit: NOT FOUND

            virt_err_x : fp32_t, unit: NOT FOUND

            virt_err_y : fp32_t, unit: NOT FOUND

            virt_err_z : fp32_t, unit: NOT FOUND

            surf_fdbk_x : fp32_t, unit: NOT FOUND

            surf_fdbk_y : fp32_t, unit: NOT FOUND

            surf_fdbk_z : fp32_t, unit: NOT FOUND

            surf_unkn_x : fp32_t, unit: NOT FOUND

            surf_unkn_y : fp32_t, unit: NOT FOUND

            surf_unkn_z : fp32_t, unit: NOT FOUND

            ss_x : fp32_t, unit: NOT FOUND

            ss_y : fp32_t, unit: NOT FOUND

            ss_z : fp32_t, unit: NOT FOUND

            rel_state : message-list, unit: NOT FOUND'''
        self._ax_cmd = ax_cmd
        self._ay_cmd = ay_cmd
        self._az_cmd = az_cmd
        self._ax_des = ax_des
        self._ay_des = ay_des
        self._az_des = az_des
        self._virt_err_x = virt_err_x
        self._virt_err_y = virt_err_y
        self._virt_err_z = virt_err_z
        self._surf_fdbk_x = surf_fdbk_x
        self._surf_fdbk_y = surf_fdbk_y
        self._surf_fdbk_z = surf_fdbk_z
        self._surf_unkn_x = surf_unkn_x
        self._surf_unkn_y = surf_unkn_y
        self._surf_unkn_z = surf_unkn_z
        self._ss_x = ss_x
        self._ss_y = ss_y
        self._ss_z = ss_z
        self._rel_state = rel_state


class RelativeState(_base.base_message):
    '''Components of the vehicle desired acceleration. Relative virtual error: downward direction.

       This message class contains the following fields and their respective types:
    s_id : plaintext, unit: NOT FOUND

            dist : fp32_t, unit: NOT FOUND

            err : fp32_t, unit: NOT FOUND

            ctrl_imp : fp32_t, unit: NOT FOUND

            rel_dir_x : fp32_t, unit: NOT FOUND

            rel_dir_y : fp32_t, unit: NOT FOUND

            rel_dir_z : fp32_t, unit: NOT FOUND

            err_x : fp32_t, unit: NOT FOUND

            err_y : fp32_t, unit: NOT FOUND

            err_z : fp32_t, unit: NOT FOUND

            rf_err_x : fp32_t, unit: NOT FOUND

            rf_err_y : fp32_t, unit: NOT FOUND

            rf_err_z : fp32_t, unit: NOT FOUND

            rf_err_vx : fp32_t, unit: NOT FOUND

            rf_err_vy : fp32_t, unit: NOT FOUND

            rf_err_vz : fp32_t, unit: NOT FOUND

            ss_x : fp32_t, unit: NOT FOUND

            ss_y : fp32_t, unit: NOT FOUND

            ss_z : fp32_t, unit: NOT FOUND

            virt_err_x : fp32_t, unit: NOT FOUND

            virt_err_y : fp32_t, unit: NOT FOUND

            virt_err_z : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_s_id', '_dist', '_err', '_ctrl_imp', '_rel_dir_x', '_rel_dir_y', '_rel_dir_z', '_err_x', '_err_y', '_err_z', '_rf_err_x', '_rf_err_y', '_rf_err_z', '_rf_err_vx', '_rf_err_vy', '_rf_err_vz', '_ss_x', '_ss_y', '_ss_z', '_virt_err_x', '_virt_err_y', '_virt_err_z']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 482, stable = None, description = "Inter-vehicle formation state.", fields = ('s_id', 'dist', 'err', 'ctrl_imp', 'rel_dir_x', 'rel_dir_y', 'rel_dir_z', 'err_x', 'err_y', 'err_z', 'rf_err_x', 'rf_err_y', 'rf_err_z', 'rf_err_vx', 'rf_err_vy', 'rf_err_vz', 'ss_x', 'ss_y', 'ss_z', 'virt_err_x', 'virt_err_y', 'virt_err_z',), abbrev = "RelativeState", name = "Relative State", flags = None)

    s_id = _base.mutable_attr({'name': 'System Identifier', 'type': 'plaintext'}, "The identifier of the vehicle whose relative state is being reported.")
    '''The identifier of the vehicle whose relative state is being reported. Type: plaintext'''
    dist = _base.mutable_attr({'name': 'Distance', 'type': 'fp32_t'}, "Distance between vehicles.")
    '''Distance between vehicles. Type: fp32_t'''
    err = _base.mutable_attr({'name': 'Position Error', 'type': 'fp32_t'}, "Relative position error norm.")
    '''Relative position error norm. Type: fp32_t'''
    ctrl_imp = _base.mutable_attr({'name': 'Control Importance', 'type': 'fp32_t'}, "Weight in the computation of the desired acceleration.")
    '''Weight in the computation of the desired acceleration. Type: fp32_t'''
    rel_dir_x = _base.mutable_attr({'name': 'Relative Direction X (North)', 'type': 'fp32_t'}, "Inter-vehicle direction vector: North component.")
    '''Inter-vehicle direction vector: North component. Type: fp32_t'''
    rel_dir_y = _base.mutable_attr({'name': 'Relative Direction Y (East)', 'type': 'fp32_t'}, "Inter-vehicle direction vector: East component.")
    '''Inter-vehicle direction vector: East component. Type: fp32_t'''
    rel_dir_z = _base.mutable_attr({'name': 'Relative Direction Z (Down)', 'type': 'fp32_t'}, "Inter-vehicle direction vector: Down component.")
    '''Inter-vehicle direction vector: Down component. Type: fp32_t'''
    err_x = _base.mutable_attr({'name': 'X Position Error (North)', 'type': 'fp32_t'}, "Relative position error: North component.")
    '''Relative position error: North component. Type: fp32_t'''
    err_y = _base.mutable_attr({'name': 'Y Position Error (East)', 'type': 'fp32_t'}, "Relative position error: East component.")
    '''Relative position error: East component. Type: fp32_t'''
    err_z = _base.mutable_attr({'name': 'Z Position Error (Down)', 'type': 'fp32_t'}, "Relative position error: Down component.")
    '''Relative position error: Down component. Type: fp32_t'''
    rf_err_x = _base.mutable_attr({'name': 'X Position Error In Relative Frame (North)', 'type': 'fp32_t'}, "Relative position error: X component on the inter-vehicle reference frame.")
    '''Relative position error: X component on the inter-vehicle reference frame. Type: fp32_t'''
    rf_err_y = _base.mutable_attr({'name': 'Y Position Error In Relative Frame (East)', 'type': 'fp32_t'}, "Relative position error: Y component on the inter-vehicle reference frame.")
    '''Relative position error: Y component on the inter-vehicle reference frame. Type: fp32_t'''
    rf_err_z = _base.mutable_attr({'name': 'Z Position Error In Relative Frame (Down)', 'type': 'fp32_t'}, "Relative position error: Z component on the inter-vehicle reference frame.")
    '''Relative position error: Z component on the inter-vehicle reference frame. Type: fp32_t'''
    rf_err_vx = _base.mutable_attr({'name': 'X Velocity Error In Relative Frame (North)', 'type': 'fp32_t'}, "Relative veloctity error: X component in the inter-vehicle reference frame.")
    '''Relative veloctity error: X component in the inter-vehicle reference frame. Type: fp32_t'''
    rf_err_vy = _base.mutable_attr({'name': 'Y Velocity Error In Relative Frame (East)', 'type': 'fp32_t'}, "Relative velocity error: Y component on the inter-vehicle reference frame.")
    '''Relative velocity error: Y component on the inter-vehicle reference frame. Type: fp32_t'''
    rf_err_vz = _base.mutable_attr({'name': 'Z Velocity Error In Relative Frame (Down)', 'type': 'fp32_t'}, "Relative velocity error: Z component on the inter-vehicle reference frame.")
    '''Relative velocity error: Z component on the inter-vehicle reference frame. Type: fp32_t'''
    ss_x = _base.mutable_attr({'name': 'X Convergence Deviation (North)', 'type': 'fp32_t'}, "Deviation from convergence (sliding surface): X component on the inter-vehicle reference frame.")
    '''Deviation from convergence (sliding surface): X component on the inter-vehicle reference frame. Type: fp32_t'''
    ss_y = _base.mutable_attr({'name': 'Y Convergence Deviation (East)', 'type': 'fp32_t'}, "Deviation from convergence (sliding surface): Y component on the inter-vehicle reference frame.")
    '''Deviation from convergence (sliding surface): Y component on the inter-vehicle reference frame. Type: fp32_t'''
    ss_z = _base.mutable_attr({'name': 'Z Convergence Deviation (Down)', 'type': 'fp32_t'}, "Deviation from convergence (sliding surface): Z component on the inter-vehicle reference frame.")
    '''Deviation from convergence (sliding surface): Z component on the inter-vehicle reference frame. Type: fp32_t'''
    virt_err_x = _base.mutable_attr({'name': 'X Virtual Error (North)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Relative virtual error: northward direction.")
    '''Components of the vehicle desired acceleration. Relative virtual error: northward direction. Type: fp32_t'''
    virt_err_y = _base.mutable_attr({'name': 'Y Virtual Error (East)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Relative virtual error: eastward direction.")
    '''Components of the vehicle desired acceleration. Relative virtual error: eastward direction. Type: fp32_t'''
    virt_err_z = _base.mutable_attr({'name': 'Z Virtual Error (Down)', 'type': 'fp32_t'}, "Components of the vehicle desired acceleration. Relative virtual error: downward direction.")
    '''Components of the vehicle desired acceleration. Relative virtual error: downward direction. Type: fp32_t'''

    def __init__(self, s_id = None, dist = None, err = None, ctrl_imp = None, rel_dir_x = None, rel_dir_y = None, rel_dir_z = None, err_x = None, err_y = None, err_z = None, rf_err_x = None, rf_err_y = None, rf_err_z = None, rf_err_vx = None, rf_err_vy = None, rf_err_vz = None, ss_x = None, ss_y = None, ss_z = None, virt_err_x = None, virt_err_y = None, virt_err_z = None):
        '''Class constructor
        
        Components of the vehicle desired acceleration. Relative virtual error: downward direction.

       This message class contains the following fields and their respective types:
    s_id : plaintext, unit: NOT FOUND

            dist : fp32_t, unit: NOT FOUND

            err : fp32_t, unit: NOT FOUND

            ctrl_imp : fp32_t, unit: NOT FOUND

            rel_dir_x : fp32_t, unit: NOT FOUND

            rel_dir_y : fp32_t, unit: NOT FOUND

            rel_dir_z : fp32_t, unit: NOT FOUND

            err_x : fp32_t, unit: NOT FOUND

            err_y : fp32_t, unit: NOT FOUND

            err_z : fp32_t, unit: NOT FOUND

            rf_err_x : fp32_t, unit: NOT FOUND

            rf_err_y : fp32_t, unit: NOT FOUND

            rf_err_z : fp32_t, unit: NOT FOUND

            rf_err_vx : fp32_t, unit: NOT FOUND

            rf_err_vy : fp32_t, unit: NOT FOUND

            rf_err_vz : fp32_t, unit: NOT FOUND

            ss_x : fp32_t, unit: NOT FOUND

            ss_y : fp32_t, unit: NOT FOUND

            ss_z : fp32_t, unit: NOT FOUND

            virt_err_x : fp32_t, unit: NOT FOUND

            virt_err_y : fp32_t, unit: NOT FOUND

            virt_err_z : fp32_t, unit: NOT FOUND'''
        self._s_id = s_id
        self._dist = dist
        self._err = err
        self._ctrl_imp = ctrl_imp
        self._rel_dir_x = rel_dir_x
        self._rel_dir_y = rel_dir_y
        self._rel_dir_z = rel_dir_z
        self._err_x = err_x
        self._err_y = err_y
        self._err_z = err_z
        self._rf_err_x = rf_err_x
        self._rf_err_y = rf_err_y
        self._rf_err_z = rf_err_z
        self._rf_err_vx = rf_err_vx
        self._rf_err_vy = rf_err_vy
        self._rf_err_vz = rf_err_vz
        self._ss_x = ss_x
        self._ss_y = ss_y
        self._ss_z = ss_z
        self._virt_err_x = virt_err_x
        self._virt_err_y = virt_err_y
        self._virt_err_z = virt_err_z


class Dislodge(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            rpm : fp32_t, unit: NOT FOUND

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''

    class DIRECTION(_enum.IntEnum):
        '''Full name: Direction
        Prefix: DIR'''
    
        AUTO = 0
        '''Name: Let the vehicle decide'''
    
        FORWARD = 1
        '''Name: Attempt to move forward'''
    
        BACKWARD = 2
        '''Name: Attempt to move backward'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_rpm', '_direction', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 483, stable = None, description = "A \"Dislodge\" is a maneuver ordering the vehicle to attempt a series of thruster operations that will hopefully get it unstuck from an entangled condition. Parameters are RPMs for the motor when attempting dislodge and and a flag specifying whether the thrust burst should be attempted forward, backward or auto (letting the vehicle decide).", fields = ('timeout', 'rpm', 'direction', 'custom',), abbrev = "Dislodge", name = "Dislodge Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    rpm = _base.mutable_attr({'name': 'RPM', 'type': 'fp32_t'}, "Maneuver RPM reference.")
    '''Maneuver RPM reference. Type: fp32_t'''
    direction = _base.mutable_attr({'name': 'Direction', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'DIR'}, "Direction to which the vehicle should attempt to unstuck. Enumerated (Local).")
    '''Direction to which the vehicle should attempt to unstuck. Enumerated (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, rpm = None, direction = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            rpm : fp32_t, unit: NOT FOUND

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._rpm = rpm
        self._direction = direction
        self._custom = custom


class Formation(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    formation_name : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            group_name : plaintext, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            description : plaintext, unit: NOT FOUND

            reference_frame : uint8_t, unit: Enumerated (Local)

            participants : message-list, unit: NOT FOUND

            leader_bank_lim : fp32_t, unit: rad

            leader_speed_min : fp32_t, unit: m/s

            leader_speed_max : fp32_t, unit: m/s

            leader_alt_min : fp32_t, unit: m

            leader_alt_max : fp32_t, unit: m

            pos_sim_err_lim : fp32_t, unit: m

            pos_sim_err_wrn : fp32_t, unit: m

            pos_sim_err_timeout : uint16_t, unit: s

            converg_max : fp32_t, unit: m

            converg_timeout : uint16_t, unit: s

            comms_timeout : uint16_t, unit: s

            turb_lim : fp32_t, unit: m/s

            custom : plaintext, unit: TupleList'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: FC'''
    
        REQUEST = 0
        '''Name: Request'''
    
        REPORT = 1
        '''Name: Report'''
    
    
    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        START = 0
        '''Name: Start'''
    
        STOP = 1
        '''Name: Stop'''
    
        READY = 2
        '''Name: Ready'''
    
        EXECUTING = 3
        '''Name: Executing'''
    
        FAILURE = 4
        '''Name: Failure'''
    
    
    class REFERENCE_FRAME(_enum.IntEnum):
        '''Full name: Formation Reference Frame
        Prefix: OP'''
    
        EARTH_FIXED = 0
        '''Name: Earth Fixed'''
    
        PATH_FIXED = 1
        '''Name: Path Fixed'''
    
        PATH_CURVED = 2
        '''Name: Path Curved'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_formation_name', '_type', '_op', '_group_name', '_plan_id', '_description', '_reference_frame', '_participants', '_leader_bank_lim', '_leader_speed_min', '_leader_speed_max', '_leader_alt_min', '_leader_alt_max', '_pos_sim_err_lim', '_pos_sim_err_wrn', '_pos_sim_err_timeout', '_converg_max', '_converg_timeout', '_comms_timeout', '_turb_lim', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Maneuvering", id = 484, stable = None, description = "The \"Formation\" is a controller to execute a maneuver with a team of vehicles. It defines the: - Vehicles included in the formation group - Vehicles relative positions inside the formation - Reference frame where the relative positions are defined - Formation shape configuration - Plan (set of maneuvers) to be followed by the formation center - Plan contrains (virtual leader speed and bank limits) - Supervision settings The formation reference frame may be: - Earth Fixed: Where the vehicles relative position do not depend on the followed path. This results in all UAVs following the same path with an offset relative to each other; - Path Fixed: Where the vehicles relative position depends on the followed path, changing the inter-vehicle offset direction with the path direction. - Path Curved: Where the vehicles relative position depends on the followed path, changing the inter-vehicle offset direction with the path direction and direction change rate. An offset in the xx axis results in a distance over the curved path line. An offset in the yy axis results in an offset of the vehicle path line relative to the formation center path line.", fields = ('formation_name', 'type', 'op', 'group_name', 'plan_id', 'description', 'reference_frame', 'participants', 'leader_bank_lim', 'leader_speed_min', 'leader_speed_max', 'leader_alt_min', 'leader_alt_max', 'pos_sim_err_lim', 'pos_sim_err_wrn', 'pos_sim_err_timeout', 'converg_max', 'converg_timeout', 'comms_timeout', 'turb_lim', 'custom',), abbrev = "Formation", name = "Formation", flags = None)

    formation_name = _base.mutable_attr({'name': 'Formation Name', 'type': 'plaintext'}, "Name of the formation configuration.")
    '''Name of the formation configuration. Type: plaintext'''
    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'FC'}, "Indicates if the message is a request, or a reply to a previous request. Enumerated (Local).")
    '''Indicates if the message is a request, or a reply to a previous request. Enumerated (Local). Type: uint8_t'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    group_name = _base.mutable_attr({'name': 'Target Group Name', 'type': 'plaintext'}, "Target group for the formation plan.")
    '''Target group for the formation plan. Type: plaintext'''
    plan_id = _base.mutable_attr({'name': 'Formation Plan ID', 'type': 'plaintext'}, "The flight plan's identifier. Flight plan defined to be tracked by the formation leader.")
    '''The flight plan's identifier. Flight plan defined to be tracked by the formation leader. Type: plaintext'''
    description = _base.mutable_attr({'name': 'Plan Description', 'type': 'plaintext'}, "Verbose text description of plan.")
    '''Verbose text description of plan. Type: plaintext'''
    reference_frame = _base.mutable_attr({'name': 'Formation Reference Frame', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Formation reference frame Enumerated (Local).")
    '''Formation reference frame Enumerated (Local). Type: uint8_t'''
    participants = _base.mutable_attr({'name': 'Formation Participants', 'type': 'message-list', 'message-type': 'VehicleFormationParticipant'}, "List of formation participants.")
    '''List of formation participants. Type: message-list'''
    leader_bank_lim = _base.mutable_attr({'name': 'Formation Leader Bank Limit', 'type': 'fp32_t', 'unit': 'rad'}, "Maximum absolute bank allowed for the formation leader.")
    '''Maximum absolute bank allowed for the formation leader. Type: fp32_t'''
    leader_speed_min = _base.mutable_attr({'name': 'Formation Leader Minimum Speed', 'type': 'fp32_t', 'unit': 'm/s'}, "Minimum speed allowed for the formation leader flight.")
    '''Minimum speed allowed for the formation leader flight. Type: fp32_t'''
    leader_speed_max = _base.mutable_attr({'name': 'Formation Leader Maximum Speed', 'type': 'fp32_t', 'unit': 'm/s'}, "Maximum speed allowed for the formation leader flight.")
    '''Maximum speed allowed for the formation leader flight. Type: fp32_t'''
    leader_alt_min = _base.mutable_attr({'name': 'Formation Leader Minimum Altitude', 'type': 'fp32_t', 'unit': 'm'}, "Minimum altitude allowed for the formation leader flight.")
    '''Minimum altitude allowed for the formation leader flight. Type: fp32_t'''
    leader_alt_max = _base.mutable_attr({'name': 'Formation Leader Maximum Altitude', 'type': 'fp32_t', 'unit': 'm'}, "Maximum altitude allowed for the formation leader flight.")
    '''Maximum altitude allowed for the formation leader flight. Type: fp32_t'''
    pos_sim_err_lim = _base.mutable_attr({'name': 'Position mismatch limit', 'type': 'fp32_t', 'unit': 'm'}, "Limit for the position mismatch between real and simulated position, before a maneuver abort is asserted.")
    '''Limit for the position mismatch between real and simulated position, before a maneuver abort is asserted. Type: fp32_t'''
    pos_sim_err_wrn = _base.mutable_attr({'name': 'Position mismatch threshold', 'type': 'fp32_t', 'unit': 'm'}, "Warning threshold for the position mismatch between real and simulated position. Above this threshold a time-out limit is evaluated to assert a maneuver abort state.")
    '''Warning threshold for the position mismatch between real and simulated position. Above this threshold a time-out limit is evaluated to assert a maneuver abort state. Type: fp32_t'''
    pos_sim_err_timeout = _base.mutable_attr({'name': 'Position mismatch time-out', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run after the position mismatch threshold is surpassed.")
    '''The amount of time the maneuver is allowed to run after the position mismatch threshold is surpassed. Type: uint16_t'''
    converg_max = _base.mutable_attr({'name': 'Convergence threshold', 'type': 'fp32_t', 'unit': 'm'}, "Threshold for the convergence measure, above which a time-out limit is evaluated to assert a maneuver abort state.")
    '''Threshold for the convergence measure, above which a time-out limit is evaluated to assert a maneuver abort state. Type: fp32_t'''
    converg_timeout = _base.mutable_attr({'name': 'Convergence time-out', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run after the convergence threshold is surpassed.")
    '''The amount of time the maneuver is allowed to run after the convergence threshold is surpassed. Type: uint16_t'''
    comms_timeout = _base.mutable_attr({'name': 'Communications time-out', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run without any update on the other formation vehicles states.")
    '''The amount of time the maneuver is allowed to run without any update on the other formation vehicles states. Type: uint16_t'''
    turb_lim = _base.mutable_attr({'name': 'Turbulence limit', 'type': 'fp32_t', 'unit': 'm/s'}, "Turbulence limit above which a maneuver abort is asserted.")
    '''Turbulence limit above which a maneuver abort is asserted. Type: fp32_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, formation_name = None, type = None, op = None, group_name = None, plan_id = None, description = None, reference_frame = None, participants = None, leader_bank_lim = None, leader_speed_min = None, leader_speed_max = None, leader_alt_min = None, leader_alt_max = None, pos_sim_err_lim = None, pos_sim_err_wrn = None, pos_sim_err_timeout = None, converg_max = None, converg_timeout = None, comms_timeout = None, turb_lim = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    formation_name : plaintext, unit: NOT FOUND

            type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            group_name : plaintext, unit: NOT FOUND

            plan_id : plaintext, unit: NOT FOUND

            description : plaintext, unit: NOT FOUND

            reference_frame : uint8_t, unit: Enumerated (Local)

            participants : message-list, unit: NOT FOUND

            leader_bank_lim : fp32_t, unit: rad

            leader_speed_min : fp32_t, unit: m/s

            leader_speed_max : fp32_t, unit: m/s

            leader_alt_min : fp32_t, unit: m

            leader_alt_max : fp32_t, unit: m

            pos_sim_err_lim : fp32_t, unit: m

            pos_sim_err_wrn : fp32_t, unit: m

            pos_sim_err_timeout : uint16_t, unit: s

            converg_max : fp32_t, unit: m

            converg_timeout : uint16_t, unit: s

            comms_timeout : uint16_t, unit: s

            turb_lim : fp32_t, unit: m/s

            custom : plaintext, unit: TupleList'''
        self._formation_name = formation_name
        self._type = type
        self._op = op
        self._group_name = group_name
        self._plan_id = plan_id
        self._description = description
        self._reference_frame = reference_frame
        self._participants = participants
        self._leader_bank_lim = leader_bank_lim
        self._leader_speed_min = leader_speed_min
        self._leader_speed_max = leader_speed_max
        self._leader_alt_min = leader_alt_min
        self._leader_alt_max = leader_alt_max
        self._pos_sim_err_lim = pos_sim_err_lim
        self._pos_sim_err_wrn = pos_sim_err_wrn
        self._pos_sim_err_timeout = pos_sim_err_timeout
        self._converg_max = converg_max
        self._converg_timeout = converg_timeout
        self._comms_timeout = comms_timeout
        self._turb_lim = turb_lim
        self._custom = custom


class Launch(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 485, stable = None, description = "A \"Launch\" is a maneuver specifying a movement of the vehicle to a target waypoint after being launched from a ship or pier. The waypoint is described by the WGS-84 waypoint coordinate and target Z reference. Mandatory parameters defined for a \"Launch\" are timeout, speed and speed units.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'custom',), abbrev = "Launch", name = "Launch Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._custom = custom


class Drop(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 486, stable = None, description = "A \"Drop\" is a maneuver specifying a movement of the vehicle to a target waypoint. The waypoint is described by the WGS-84 waypoint coordinate and target Z reference. Mandatory parameters defined for a \"Goto\" are timeout, speed and speed units.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'custom',), abbrev = "Drop", name = "Drop Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._custom = custom


class ScheduledGoto(_base.base_message):
    '''What to do if the vehicle fails to arrive before or at the requested time. Enumerated (Local).

       This message class contains the following fields and their respective types:
    arrival_time : fp64_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            travel_z : fp32_t, unit: m

            travel_z_units : uint8_t, unit: Enumerated (Global)

            delayed : uint8_t, unit: Enumerated (Local)'''

    class DELAYED(_enum.IntEnum):
        '''Full name: Delayed Behavior
        Prefix: DBEH'''
    
        RESUME = 0
        '''Name: Resume'''
    
        SKIP = 1
        '''Name: Skip'''
    
        FAIL = 2
        '''Name: Fail'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_arrival_time', '_lat', '_lon', '_z', '_z_units', '_travel_z', '_travel_z_units', '_delayed']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 487, stable = None, description = "This maneuver is used to command the vehicle to arrive at some destination at a specified absolute time. The vehicle's speed will vary according to environment conditions and/or maneuver start time.", fields = ('arrival_time', 'lat', 'lon', 'z', 'z_units', 'travel_z', 'travel_z_units', 'delayed',), abbrev = "ScheduledGoto", name = "Scheduled Goto", flags = None)

    arrival_time = _base.mutable_attr({'name': 'Time of arrival', 'type': 'fp64_t', 'unit': 's'}, "Unix timestamp, in seconds, for the arrival at the destination.")
    '''Unix timestamp, in seconds, for the arrival at the destination. Type: fp64_t'''
    lat = _base.mutable_attr({'name': 'Destination Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Destination Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Destination Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the destination z reference. Enumerated (Global).")
    '''Units of the destination z reference. Enumerated (Global). Type: uint8_t'''
    travel_z = _base.mutable_attr({'name': 'Travel Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Z reference to use while travelling to the destination.")
    '''Z reference to use while travelling to the destination. Type: fp32_t'''
    travel_z_units = _base.mutable_attr({'name': 'Travel Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Z reference units to use while travelling to the destination. Enumerated (Global).")
    '''Z reference units to use while travelling to the destination. Enumerated (Global). Type: uint8_t'''
    delayed = _base.mutable_attr({'name': 'Delayed Behavior', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'prefix': 'DBEH'}, "What to do if the vehicle fails to arrive before or at the requested time. Enumerated (Local).")
    '''What to do if the vehicle fails to arrive before or at the requested time. Enumerated (Local). Type: uint8_t'''

    def __init__(self, arrival_time = None, lat = None, lon = None, z = None, z_units = None, travel_z = None, travel_z_units = None, delayed = None):
        '''Class constructor
        
        What to do if the vehicle fails to arrive before or at the requested time. Enumerated (Local).

       This message class contains the following fields and their respective types:
    arrival_time : fp64_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            travel_z : fp32_t, unit: m

            travel_z_units : uint8_t, unit: Enumerated (Global)

            delayed : uint8_t, unit: Enumerated (Local)'''
        self._arrival_time = arrival_time
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._travel_z = travel_z
        self._travel_z_units = travel_z_units
        self._delayed = delayed


class RowsCoverage(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            bearing : fp64_t, unit: rad

            cross_angle : fp64_t, unit: rad

            width : fp32_t, unit: m

            length : fp32_t, unit: m

            coff : uint8_t, unit: m

            angAperture : fp32_t, unit: rad

            range : uint16_t, unit: m

            overlap : uint8_t, unit: %

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        SQUARE_CURVE = 1
        '''Name: Square Curve'''
    
        CURVE_RIGHT = 2
        '''Name: First Curve Right'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_bearing', '_cross_angle', '_width', '_length', '_coff', '_angAperture', '_range', '_overlap', '_flags', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 488, stable = None, description = "Rows coverage (i.e: lawn mower type maneuver) but with adaptive cover", fields = ('lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'bearing', 'cross_angle', 'width', 'length', 'coff', 'angAperture', 'range', 'overlap', 'flags', 'custom',), abbrev = "RowsCoverage", name = "Rows Coverage", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp64_t', 'unit': 'rad', 'min': 0, 'max': 6.283185307179586}, "Rows bearing angle.")
    '''Rows bearing angle. Type: fp64_t'''
    cross_angle = _base.mutable_attr({'name': 'Cross Angle', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.047197551197, 'max': 1.047197551197}, "Rows cross angle reference.")
    '''Rows cross angle reference. Type: fp64_t'''
    width = _base.mutable_attr({'name': 'Width', 'min': 0, 'type': 'fp32_t', 'unit': 'm'}, "Width of the maneuver.")
    '''Width of the maneuver. Type: fp32_t'''
    length = _base.mutable_attr({'name': 'Length', 'min': 0, 'type': 'fp32_t', 'unit': 'm'}, "Length of the maneuver.")
    '''Length of the maneuver. Type: fp32_t'''
    coff = _base.mutable_attr({'name': 'Curve Offset', 'type': 'uint8_t', 'unit': 'm'}, "Desired curve offset.")
    '''Desired curve offset. Type: uint8_t'''
    angAperture = _base.mutable_attr({'name': 'Angular Aperture', 'type': 'fp32_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793, 'value': 2.094395}, "Angular aperture of the sensor (looking downwards).")
    '''Angular aperture of the sensor (looking downwards). Type: fp32_t'''
    range = _base.mutable_attr({'name': 'Range', 'type': 'uint16_t', 'unit': 'm'}, "Maximum range of the sensor (in meters downwards from the vehicle's position).")
    '''Maximum range of the sensor (in meters downwards from the vehicle's position). Type: uint16_t'''
    overlap = _base.mutable_attr({'name': 'Overlap', 'type': 'uint8_t', 'unit': '%', 'max': 100, 'value': 10}, "Amount of overlap among different transect surveys.")
    '''Amount of overlap among different transect surveys. Type: uint8_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'FLG'}, "Maneuver optional flags. Bitfield (Local).")
    '''Maneuver optional flags. Bitfield (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, bearing = None, cross_angle = None, width = None, length = None, coff = None, angAperture = None, range = None, overlap = None, flags = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            bearing : fp64_t, unit: rad

            cross_angle : fp64_t, unit: rad

            width : fp32_t, unit: m

            length : fp32_t, unit: m

            coff : uint8_t, unit: m

            angAperture : fp32_t, unit: rad

            range : uint16_t, unit: m

            overlap : uint8_t, unit: %

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._bearing = bearing
        self._cross_angle = cross_angle
        self._width = width
        self._length = length
        self._coff = coff
        self._angAperture = angAperture
        self._range = range
        self._overlap = overlap
        self._flags = flags
        self._custom = custom


class Sample(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            syringe0 : uint8_t, unit: Enumerated (Global)

            syringe1 : uint8_t, unit: Enumerated (Global)

            syringe2 : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_syringe0', '_syringe1', '_syringe2', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 489, stable = None, description = "A \"Sample\" is a maneuver specifying a movement of the vehicle to a target waypoint. The waypoint is described by the WGS-84 waypoint coordinate and target Z reference. Mandatory parameters defined for a \"Goto\" are timeout, speed and speed units.", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'syringe0', 'syringe1', 'syringe2', 'custom',), abbrev = "Sample", name = "Sample Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Target reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    syringe0 = _base.mutable_attr({'name': 'Syringe0', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'Boolean'}, "True when sampling with servo 0. Enumerated (Global).")
    '''True when sampling with servo 0. Enumerated (Global). Type: uint8_t'''
    syringe1 = _base.mutable_attr({'name': 'Syringe1', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'Boolean'}, "True when sampling with servo 1. Enumerated (Global).")
    '''True when sampling with servo 1. Enumerated (Global). Type: uint8_t'''
    syringe2 = _base.mutable_attr({'name': 'Syringe2', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'Boolean'}, "True when sampling with servo 2. Enumerated (Global).")
    '''True when sampling with servo 2. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, syringe0 = None, syringe1 = None, syringe2 = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            syringe0 : uint8_t, unit: Enumerated (Global)

            syringe1 : uint8_t, unit: Enumerated (Global)

            syringe2 : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._syringe0 = syringe0
        self._syringe1 = syringe1
        self._syringe2 = syringe2
        self._custom = custom


class ImageTracking(_base.base_message):
    '''A "ImageTracking" is a maneuver specifying a particular heading to the detected object.

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 490, stable = None, description = "A \"ImageTracking\" is a maneuver specifying a particular heading to the detected object.", fields = [], abbrev = "ImageTracking", name = "Image Tracking", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        A "ImageTracking" is a maneuver specifying a particular heading to the detected object.

       This message class contains the following fields and their respective types:
'''


class Takeoff(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            takeoff_pitch : fp32_t, unit: rad

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_takeoff_pitch', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 491, stable = None, description = "Automatic takeoff for UAVs. This maneuver specifies a target waypoint where to takeoff. Takeoff direction is set from the direction the plane is pointing when the auto takeoff command is started. It will remain that way until the vehicle reaches the target z reference. After that it will go to the target waypoint.", fields = ('lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'takeoff_pitch', 'custom',), abbrev = "Takeoff", name = "Takeoff Maneuver", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target altitude or height for the automatic takeoff.")
    '''Target altitude or height for the automatic takeoff. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    takeoff_pitch = _base.mutable_attr({'name': 'Pitch Angle', 'type': 'fp32_t', 'unit': 'rad', 'min': 0, 'max': 1.5707963267949}, "Minimum pitch angle during automatic takeoff.")
    '''Minimum pitch angle during automatic takeoff. Type: fp32_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, takeoff_pitch = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            takeoff_pitch : fp32_t, unit: rad

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._takeoff_pitch = takeoff_pitch
        self._custom = custom


class Land(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            abort_z : fp32_t, unit: m

            bearing : fp64_t, unit: rad

            glide_slope : uint8_t, unit: %

            glide_slope_alt : fp32_t, unit: m

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_abort_z', '_bearing', '_glide_slope', '_glide_slope_alt', '_custom']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 492, stable = None, description = "Automatic landing on the ground, for UAVs. This maneuver specifies the target touchdown location and sets the final approach based on the maneuver bearing and glide slope parameters.", fields = ('lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'abort_z', 'bearing', 'glide_slope', 'glide_slope_alt', 'custom',), abbrev = "Land", name = "Land Maneuver", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of touchdown waypoint.")
    '''WGS-84 Latitude of touchdown waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of touchdown waypoint.")
    '''WGS-84 Longitude of touchdown waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Target altitude or height for the automatic landing.")
    '''Target altitude or height for the automatic landing. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference and abort z reference. Enumerated (Global).")
    '''Units of the z reference and abort z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    abort_z = _base.mutable_attr({'name': 'Abort Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Abort altitude or height. If landing is aborted while executing, the UAV will maintain its course and attempt to climb to the abort z reference.")
    '''Abort altitude or height. If landing is aborted while executing, the UAV will maintain its course and attempt to climb to the abort z reference. Type: fp32_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp64_t', 'unit': 'rad', 'min': 0, 'max': 6.283185307179586}, "Land bearing angle.")
    '''Land bearing angle. Type: fp64_t'''
    glide_slope = _base.mutable_attr({'name': 'Glide Slope', 'type': 'uint8_t', 'unit': '%', 'max': 10}, "Ratio of the distance from the last waypoint to the landing point (touchdown) and the height difference between them.")
    '''Ratio of the distance from the last waypoint to the landing point (touchdown) and the height difference between them. Type: uint8_t'''
    glide_slope_alt = _base.mutable_attr({'name': 'Glide Slope Altitude', 'type': 'fp32_t', 'unit': 'm'}, "Height difference between the last waypoint to the landing point (touchdown).")
    '''Height difference between the last waypoint to the landing point (touchdown). Type: fp32_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, abort_z = None, bearing = None, glide_slope = None, glide_slope_alt = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            abort_z : fp32_t, unit: m

            bearing : fp64_t, unit: rad

            glide_slope : uint8_t, unit: %

            glide_slope_alt : fp32_t, unit: m

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._abort_z = abort_z
        self._bearing = bearing
        self._glide_slope = glide_slope
        self._glide_slope_alt = glide_slope_alt
        self._custom = custom


class AutonomousSection(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            limits : uint8_t, unit: Bitfield (Local)

            max_depth : fp64_t, unit: m

            min_alt : fp64_t, unit: m

            time_limit : fp64_t, unit: s

            area_limits : message-list, unit: NOT FOUND

            controller : plaintext, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''

    class LIMITS(_enum.IntFlag):
        '''Full name: Enforced Limits
        Prefix: ENFORCE'''
    
        EMPTY = 0
        '''No active flags'''
    
        DEPTH = 1
        '''Name: Maximum Depth Limit'''
    
        ALTITUDE = 2
        '''Name: Minimum Altitude Limit'''
    
        TIMEOUT = 4
        '''Name: Time Limit'''
    
        AREA2D = 8
        '''Name: Polygonal Area Limits'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_speed', '_speed_units', '_limits', '_max_depth', '_min_alt', '_time_limit', '_area_limits', '_controller', '_custom']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 493, stable = None, description = "This maneuver triggers an external controller that will guide the vehicle during a specified duration of time or until it relinquishes control using (ManeuverDone). The external controller is allowed to drive the vehicle only inside the specified boundaries.", fields = ('lat', 'lon', 'speed', 'speed_units', 'limits', 'max_depth', 'min_alt', 'time_limit', 'area_limits', 'controller', 'custom',), abbrev = "AutonomousSection", name = "Autonomous Section", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of the initial location.")
    '''WGS-84 Latitude of the initial location. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of the initial location.")
    '''WGS-84 Longitude of the initial location. Type: fp64_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    limits = _base.mutable_attr({'name': 'Enforced Limits', 'type': 'uint8_t', 'unit': 'Bitfield', 'prefix': 'ENFORCE'}, "No description available Bitfield (Local).")
    '''No description available Bitfield (Local). Type: uint8_t'''
    max_depth = _base.mutable_attr({'name': 'Maximum depth', 'type': 'fp64_t', 'unit': 'm'}, "Maximum depth the autonomous controller is allowed to drive to.")
    '''Maximum depth the autonomous controller is allowed to drive to. Type: fp64_t'''
    min_alt = _base.mutable_attr({'name': 'Minimum altitude', 'type': 'fp64_t', 'unit': 'm'}, "Minimum altitude the autonomous controller is allowed to drive to.")
    '''Minimum altitude the autonomous controller is allowed to drive to. Type: fp64_t'''
    time_limit = _base.mutable_attr({'name': 'Time Limit', 'type': 'fp64_t', 'unit': 's'}, "The time after which this maneuver should be stopped (if still active and TIMEOUT is enforced).")
    '''The time after which this maneuver should be stopped (if still active and TIMEOUT is enforced). Type: fp64_t'''
    area_limits = _base.mutable_attr({'name': 'Area Limits', 'type': 'message-list', 'message-type': 'PolygonVertex'}, "The boundaries of the admissable area for this autonomous section.")
    '''The boundaries of the admissable area for this autonomous section. Type: message-list'''
    controller = _base.mutable_attr({'name': 'Controller', 'type': 'plaintext'}, "The name of the controlling agent that will be allowed to guide the vehicle during the AutononousSection.")
    '''The name of the controlling agent that will be allowed to guide the vehicle during the AutononousSection. Type: plaintext'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, speed = None, speed_units = None, limits = None, max_depth = None, min_alt = None, time_limit = None, area_limits = None, controller = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            limits : uint8_t, unit: Bitfield (Local)

            max_depth : fp64_t, unit: m

            min_alt : fp64_t, unit: m

            time_limit : fp64_t, unit: s

            area_limits : message-list, unit: NOT FOUND

            controller : plaintext, unit: NOT FOUND

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._speed = speed
        self._speed_units = speed_units
        self._limits = limits
        self._max_depth = max_depth
        self._min_alt = min_alt
        self._time_limit = time_limit
        self._area_limits = area_limits
        self._controller = controller
        self._custom = custom


class FollowPoint(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            max_speed : fp32_t, unit: m/s

            speed_units : uint8_t, unit: Enumerated (Global)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_target', '_max_speed', '_speed_units', '_lat', '_lon', '_z', '_z_units', '_custom']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 494, stable = None, description = "This maneuver behaves by following a point.", fields = ('target', 'max_speed', 'speed_units', 'lat', 'lon', 'z', 'z_units', 'custom',), abbrev = "FollowPoint", name = "Follow Point Maneuver", flags = None)

    target = _base.mutable_attr({'name': 'Source To Follow', 'type': 'plaintext'}, "The identifier of the point source to follow (via RemoteSensorInfo or EstimatedState).")
    '''The identifier of the point source to follow (via RemoteSensorInfo or EstimatedState). Type: plaintext'''
    max_speed = _base.mutable_attr({'name': 'Maximum Speed', 'type': 'fp32_t', 'unit': 'm/s', 'min': 0}, "Use this speed when travelling from afar.")
    '''Use this speed when travelling from afar. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Indicates the units used for the maximum speed value. Enumerated (Global).")
    '''Indicates the units used for the maximum speed value. Enumerated (Global). Type: uint8_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of maneuver in the map. Ignored during execution.")
    '''WGS-84 Latitude of maneuver in the map. Ignored during execution. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of maneuver in the map. Ignored during execution.")
    '''WGS-84 Longitude of maneuver in the map. Ignored during execution. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z', 'type': 'fp32_t', 'unit': 'm'}, "Use z_units to specify whether z represents depth, altitude or other.")
    '''Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, target = None, max_speed = None, speed_units = None, lat = None, lon = None, z = None, z_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    target : plaintext, unit: NOT FOUND

            max_speed : fp32_t, unit: m/s

            speed_units : uint8_t, unit: Enumerated (Global)

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._target = target
        self._max_speed = max_speed
        self._speed_units = speed_units
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._custom = custom


class Alignment(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''

    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_speed', '_speed_units', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 495, stable = None, description = "An \"Alignment\" is a maneuver specifying a movement of the vehicle to a target waypoint intended to control activation of an IMU/INS in order to start aligning navigation for more precise dead reckoning operation.", fields = ('timeout', 'lat', 'lon', 'speed', 'speed_units', 'custom',), abbrev = "Alignment", name = "Alignment Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, speed = None, speed_units = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._speed = speed
        self._speed_units = speed_units
        self._custom = custom


class StationKeepingExtended(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            popup_period : uint16_t, unit: s

            popup_duration : uint16_t, unit: s

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''

    class FLAGS(_enum.IntFlag):
        '''Full name: Flags
        Prefix: FLG'''
    
        EMPTY = 0
        '''No active flags'''
    
        KEEP_SAFE = 1
        '''Name: Keep safe behaviour'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_lat', '_lon', '_z', '_z_units', '_radius', '_duration', '_speed', '_speed_units', '_popup_period', '_popup_duration', '_flags', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 496, stable = None, description = "The Station Keeping Extended maneuver makes the vehicle come to the surface and then enter a given circular perimeter around a waypoint coordinate for a certain amount of time. It extends the Station Keeping maneuver with the feature 'Keep Safe', which allows for the vehicle to hold position underwater and popup periodically to communicate.", fields = ('lat', 'lon', 'z', 'z_units', 'radius', 'duration', 'speed', 'speed_units', 'popup_period', 'popup_duration', 'flags', 'custom',), abbrev = "StationKeepingExtended", name = "Station Keeping Extended", flags = None)

    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude.")
    '''WGS-84 Latitude. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude.")
    '''WGS-84 Longitude. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    radius = _base.mutable_attr({'name': 'Radius', 'type': 'fp32_t', 'unit': 'm'}, "Radius.")
    '''Radius. Type: fp32_t'''
    duration = _base.mutable_attr({'name': 'Duration', 'type': 'uint16_t', 'unit': 's'}, "Duration (0 for unlimited).")
    '''Duration (0 for unlimited). Type: uint16_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "The value of the desired speed, in the scale specified by the \"Speed Units\" field.")
    '''The value of the desired speed, in the scale specified by the \"Speed Units\" field. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Indicates the units used for the speed value. Enumerated (Global).")
    '''Indicates the units used for the speed value. Enumerated (Global). Type: uint8_t'''
    popup_period = _base.mutable_attr({'name': 'PopUp Period', 'type': 'uint16_t', 'unit': 's'}, "The period at which the vehicle will popup to report its position. Only used if flag KEEP_SAFE is on.")
    '''The period at which the vehicle will popup to report its position. Only used if flag KEEP_SAFE is on. Type: uint16_t'''
    popup_duration = _base.mutable_attr({'name': 'PopUp Duration', 'type': 'uint16_t', 'unit': 's'}, "The duration of the station keeping at surface level when it pops up. Only used if flag KEEP_SAFE is on.")
    '''The duration of the station keeping at surface level when it pops up. Only used if flag KEEP_SAFE is on. Type: uint16_t'''
    flags = _base.mutable_attr({'name': 'Flags', 'prefix': 'FLG', 'type': 'uint8_t', 'unit': 'Bitfield'}, "Flags of the maneuver. Bitfield (Local).")
    '''Flags of the maneuver. Bitfield (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, lat = None, lon = None, z = None, z_units = None, radius = None, duration = None, speed = None, speed_units = None, popup_period = None, popup_duration = None, flags = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            radius : fp32_t, unit: m

            duration : uint16_t, unit: s

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            popup_period : uint16_t, unit: s

            popup_duration : uint16_t, unit: s

            flags : uint8_t, unit: Bitfield (Local)

            custom : plaintext, unit: TupleList'''
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._radius = radius
        self._duration = duration
        self._speed = speed
        self._speed_units = speed_units
        self._popup_period = popup_period
        self._popup_duration = popup_duration
        self._flags = flags
        self._custom = custom


class ManeuverDone(_base.base_message):
    '''Notification of completion of a maneuver (optional use).

       This message class contains the following fields and their respective types:
'''

    __slots__ = ['_Attributes', '_header', '_footer']
    Attributes = _base.MessageAttributes(source = "ccu, vehicle", usedby = None, category = "Maneuvering", id = 497, stable = None, description = "Notification of completion of a maneuver (optional use).", fields = [], abbrev = "ManeuverDone", name = "Maneuver Done", flags = None)


    def __init__(self, ):
        '''Class constructor
        
        Notification of completion of a maneuver (optional use).

       This message class contains the following fields and their respective types:
'''


class Magnetometer(_base.base_message):
    '''Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            bearing : fp64_t, unit: rad

            width : fp32_t, unit: m

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''

    class DIRECTION(_enum.IntEnum):
        '''Full name: Direction
        Prefix: MD'''
    
        CLOCKW_FIRST = 0
        '''Name: Clockwise First'''
    
        CCLOCKW_FIRST = 1
        '''Name: Counter Clockwise First'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_timeout', '_lat', '_lon', '_z', '_z_units', '_speed', '_speed_units', '_bearing', '_width', '_direction', '_custom']
    Attributes = _base.MessageAttributes(source = "ccu", usedby = None, category = "Maneuvering", id = 499, stable = None, description = "Magnetometer calibration maneuver (i.e: one square pattern in one direction, a second square in the opposite direction)", fields = ('timeout', 'lat', 'lon', 'z', 'z_units', 'speed', 'speed_units', 'bearing', 'width', 'direction', 'custom',), abbrev = "Magnetometer", name = "Magnetometer Maneuver", flags = None)

    timeout = _base.mutable_attr({'name': 'Timeout', 'type': 'uint16_t', 'unit': 's'}, "The amount of time the maneuver is allowed to run.")
    '''The amount of time the maneuver is allowed to run. Type: uint16_t'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude of target waypoint.")
    '''WGS-84 Latitude of target waypoint. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude of target waypoint.")
    '''WGS-84 Longitude of target waypoint. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other.")
    '''Maneuver reference in the z axis. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    speed = _base.mutable_attr({'name': 'Speed', 'type': 'fp32_t'}, "Maneuver speed reference.")
    '''Maneuver speed reference. Type: fp32_t'''
    speed_units = _base.mutable_attr({'name': 'Speed Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'SpeedUnits'}, "Speed units. Enumerated (Global).")
    '''Speed units. Enumerated (Global). Type: uint8_t'''
    bearing = _base.mutable_attr({'name': 'Bearing', 'type': 'fp64_t', 'unit': 'rad', 'min': 0, 'max': 6.283185307179586}, "Rows bearing angle.")
    '''Rows bearing angle. Type: fp64_t'''
    width = _base.mutable_attr({'name': 'Width', 'min': 50, 'type': 'fp32_t', 'unit': 'm'}, "Width of the maneuver.")
    '''Width of the maneuver. Type: fp32_t'''
    direction = _base.mutable_attr({'name': 'Direction', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'MD', 'max': 1}, "Desired direction. Enumerated (Local).")
    '''Desired direction. Enumerated (Local). Type: uint8_t'''
    custom = _base.mutable_attr({'name': 'Custom settings for maneuver', 'unit': 'TupleList', 'type': 'plaintext'}, "Custom settings for maneuver.")
    '''Custom settings for maneuver. Type: plaintext'''

    def __init__(self, timeout = None, lat = None, lon = None, z = None, z_units = None, speed = None, speed_units = None, bearing = None, width = None, direction = None, custom = None):
        '''Class constructor
        
        Custom settings for maneuver.

       This message class contains the following fields and their respective types:
    timeout : uint16_t, unit: s

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            speed : fp32_t, unit: NOT FOUND

            speed_units : uint8_t, unit: Enumerated (Global)

            bearing : fp64_t, unit: rad

            width : fp32_t, unit: m

            direction : uint8_t, unit: Enumerated (Local)

            custom : plaintext, unit: TupleList'''
        self._timeout = timeout
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._speed = speed
        self._speed_units = speed_units
        self._bearing = bearing
        self._width = width
        self._direction = direction
        self._custom = custom


class Target(_base.base_message):
    '''Speed Over Ground.

       This message class contains the following fields and their respective types:
    label : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            cog : fp32_t, unit: rad

            sog : fp32_t, unit: m/s'''

    __slots__ = ['_Attributes', '_header', '_footer', '_label', '_lat', '_lon', '_z', '_z_units', '_cog', '_sog']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 800, stable = None, description = "Target.", fields = ('label', 'lat', 'lon', 'z', 'z_units', 'cog', 'sog',), abbrev = "Target", name = "Target", flags = None)

    label = _base.mutable_attr({'name': 'Label', 'type': 'plaintext'}, "Target identifier.")
    '''Target identifier. Type: plaintext'''
    lat = _base.mutable_attr({'name': 'Latitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -1.5707963267948966, 'max': 1.5707963267948966}, "WGS-84 Latitude coordinate.")
    '''WGS-84 Latitude coordinate. Type: fp64_t'''
    lon = _base.mutable_attr({'name': 'Longitude WGS-84', 'type': 'fp64_t', 'unit': 'rad', 'min': -3.141592653589793, 'max': 3.141592653589793}, "WGS-84 Longitude coordinate.")
    '''WGS-84 Longitude coordinate. Type: fp64_t'''
    z = _base.mutable_attr({'name': 'Z Reference', 'type': 'fp32_t', 'unit': 'm'}, "Z axis reference. Use z_units to specify whether z represents depth, altitude or other.")
    '''Z axis reference. Use z_units to specify whether z represents depth, altitude or other. Type: fp32_t'''
    z_units = _base.mutable_attr({'name': 'Z Units', 'type': 'uint8_t', 'value': 0, 'unit': 'Enumerated', 'enum-def': 'ZUnits'}, "Units of the z reference. Enumerated (Global).")
    '''Units of the z reference. Enumerated (Global). Type: uint8_t'''
    cog = _base.mutable_attr({'name': 'Course Over Ground', 'type': 'fp32_t', 'unit': 'rad'}, "Course Over Ground (true).")
    '''Course Over Ground (true). Type: fp32_t'''
    sog = _base.mutable_attr({'name': 'Speed Over Ground', 'type': 'fp32_t', 'unit': 'm/s'}, "Speed Over Ground.")
    '''Speed Over Ground. Type: fp32_t'''

    def __init__(self, label = None, lat = None, lon = None, z = None, z_units = None, cog = None, sog = None):
        '''Class constructor
        
        Speed Over Ground.

       This message class contains the following fields and their respective types:
    label : plaintext, unit: NOT FOUND

            lat : fp64_t, unit: rad

            lon : fp64_t, unit: rad

            z : fp32_t, unit: m

            z_units : uint8_t, unit: Enumerated (Global)

            cog : fp32_t, unit: rad

            sog : fp32_t, unit: m/s'''
        self._label = label
        self._lat = lat
        self._lon = lon
        self._z = z
        self._z_units = z_units
        self._cog = cog
        self._sog = sog


class FormCtrlParam(_base.base_message):
    '''Collision avoidance and formation shape gain (position tracking relative to the other formation vehicles). Individual vehicle importance gain (relative to the leader), when the relative position or the velocity state indicate higher probability of collision.

       This message class contains the following fields and their respective types:
    Action : uint8_t, unit: Enumerated (Local)

            LonGain : fp32_t, unit: NOT FOUND

            LatGain : fp32_t, unit: NOT FOUND

            BondThick : uint32_t, unit: NOT FOUND

            LeadGain : fp32_t, unit: NOT FOUND

            DeconflGain : fp32_t, unit: NOT FOUND'''

    class ACTION(_enum.IntEnum):
        '''Full name: Action
        Prefix: OP'''
    
        REQ = 0
        '''Name: Request'''
    
        SET = 1
        '''Name: Set'''
    
        REP = 2
        '''Name: Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_Action', '_LonGain', '_LatGain', '_BondThick', '_LeadGain', '_DeconflGain']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Maneuvering", id = 820, stable = None, description = "Formation controller paramenters, as: trajectory gains, control boundary layer thickness, and formation shape gains.", fields = ('Action', 'LonGain', 'LatGain', 'BondThick', 'LeadGain', 'DeconflGain',), abbrev = "FormCtrlParam", name = "Formation Control Parameters", flags = None)

    Action = _base.mutable_attr({'name': 'Action', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Action on the vehicle formation control parameters. Enumerated (Local).")
    '''Action on the vehicle formation control parameters. Enumerated (Local). Type: uint8_t'''
    LonGain = _base.mutable_attr({'name': 'Longitudinal Gain', 'type': 'fp32_t'}, "Trajectory gain over the vehicle longitudinal direction.")
    '''Trajectory gain over the vehicle longitudinal direction. Type: fp32_t'''
    LatGain = _base.mutable_attr({'name': 'Lateral Gain', 'type': 'fp32_t'}, "Trajectory gain over the vehicle lateral direction.")
    '''Trajectory gain over the vehicle lateral direction. Type: fp32_t'''
    BondThick = _base.mutable_attr({'name': 'Boundary Layer Thickness', 'type': 'uint32_t'}, "Control sliding surface boundary layer thickness.")
    '''Control sliding surface boundary layer thickness. Type: uint32_t'''
    LeadGain = _base.mutable_attr({'name': 'Leader Gain', 'type': 'fp32_t'}, "Formation shape gain (absolute vehicle position tracking). Leader control importance gain (relative to the sum of every other formation vehicle).")
    '''Formation shape gain (absolute vehicle position tracking). Leader control importance gain (relative to the sum of every other formation vehicle). Type: fp32_t'''
    DeconflGain = _base.mutable_attr({'name': 'Deconfliction Gain', 'type': 'fp32_t'}, "Collision avoidance and formation shape gain (position tracking relative to the other formation vehicles). Individual vehicle importance gain (relative to the leader), when the relative position or the velocity state indicate higher probability of collision.")
    '''Collision avoidance and formation shape gain (position tracking relative to the other formation vehicles). Individual vehicle importance gain (relative to the leader), when the relative position or the velocity state indicate higher probability of collision. Type: fp32_t'''

    def __init__(self, Action = None, LonGain = None, LatGain = None, BondThick = None, LeadGain = None, DeconflGain = None):
        '''Class constructor
        
        Collision avoidance and formation shape gain (position tracking relative to the other formation vehicles). Individual vehicle importance gain (relative to the leader), when the relative position or the velocity state indicate higher probability of collision.

       This message class contains the following fields and their respective types:
    Action : uint8_t, unit: Enumerated (Local)

            LonGain : fp32_t, unit: NOT FOUND

            LatGain : fp32_t, unit: NOT FOUND

            BondThick : uint32_t, unit: NOT FOUND

            LeadGain : fp32_t, unit: NOT FOUND

            DeconflGain : fp32_t, unit: NOT FOUND'''
        self._Action = Action
        self._LonGain = LonGain
        self._LatGain = LatGain
        self._BondThick = BondThick
        self._LeadGain = LeadGain
        self._DeconflGain = DeconflGain


class FormationEval(_base.base_message):
    '''Mean minimum distance to any other vehicle in the formation.

       This message class contains the following fields and their respective types:
    err_mean : fp32_t, unit: NOT FOUND

            dist_min_abs : fp32_t, unit: NOT FOUND

            dist_min_mean : fp32_t, unit: NOT FOUND'''

    __slots__ = ['_Attributes', '_header', '_footer', '_err_mean', '_dist_min_abs', '_dist_min_mean']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 821, stable = None, description = "Formation control performance evaluation variables.", fields = ('err_mean', 'dist_min_abs', 'dist_min_mean',), abbrev = "FormationEval", name = "Formation Evaluation Data", flags = None)

    err_mean = _base.mutable_attr({'name': 'Mean position error', 'type': 'fp32_t'}, "Mean position error relative to the formation reference.")
    '''Mean position error relative to the formation reference. Type: fp32_t'''
    dist_min_abs = _base.mutable_attr({'name': 'Absolute minimum distance', 'type': 'fp32_t'}, "Overall minimum distance to any other vehicle in the formation.")
    '''Overall minimum distance to any other vehicle in the formation. Type: fp32_t'''
    dist_min_mean = _base.mutable_attr({'name': 'Mean minimum distance', 'type': 'fp32_t'}, "Mean minimum distance to any other vehicle in the formation.")
    '''Mean minimum distance to any other vehicle in the formation. Type: fp32_t'''

    def __init__(self, err_mean = None, dist_min_abs = None, dist_min_mean = None):
        '''Class constructor
        
        Mean minimum distance to any other vehicle in the formation.

       This message class contains the following fields and their respective types:
    err_mean : fp32_t, unit: NOT FOUND

            dist_min_abs : fp32_t, unit: NOT FOUND

            dist_min_mean : fp32_t, unit: NOT FOUND'''
        self._err_mean = err_mean
        self._dist_min_abs = dist_min_abs
        self._dist_min_mean = dist_min_mean


class FormationControlParams(_base.base_message):
    '''Maximum predicted longitudinal acceleration a vehicle can generate.

       This message class contains the following fields and their respective types:
    Action : uint8_t, unit: Enumerated (Local)

            lon_gain : fp32_t, unit: NOT FOUND

            lat_gain : fp32_t, unit: NOT FOUND

            bond_thick : fp32_t, unit: NOT FOUND

            lead_gain : fp32_t, unit: NOT FOUND

            deconfl_gain : fp32_t, unit: NOT FOUND

            accel_switch_gain : fp32_t, unit: NOT FOUND

            safe_dist : fp32_t, unit: NOT FOUND

            deconflict_offset : fp32_t, unit: NOT FOUND

            accel_safe_margin : fp32_t, unit: NOT FOUND

            accel_lim_x : fp32_t, unit: NOT FOUND'''

    class ACTION(_enum.IntEnum):
        '''Full name: Action
        Prefix: OP'''
    
        REQ = 0
        '''Name: Request'''
    
        SET = 1
        '''Name: Set'''
    
        REP = 2
        '''Name: Report'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_Action', '_lon_gain', '_lat_gain', '_bond_thick', '_lead_gain', '_deconfl_gain', '_accel_switch_gain', '_safe_dist', '_deconflict_offset', '_accel_safe_margin', '_accel_lim_x']
    Attributes = _base.MessageAttributes(source = "ccu,vehicle", usedby = None, category = "Maneuvering", id = 822, stable = None, description = "Formation controller paramenters, as: trajectory gains, control boundary layer thickness, and formation shape gains.", fields = ('Action', 'lon_gain', 'lat_gain', 'bond_thick', 'lead_gain', 'deconfl_gain', 'accel_switch_gain', 'safe_dist', 'deconflict_offset', 'accel_safe_margin', 'accel_lim_x',), abbrev = "FormationControlParams", name = "Formation Control Parameters", flags = None)

    Action = _base.mutable_attr({'name': 'Action', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Action on the vehicle formation control parameters. Enumerated (Local).")
    '''Action on the vehicle formation control parameters. Enumerated (Local). Type: uint8_t'''
    lon_gain = _base.mutable_attr({'name': 'Longitudinal Gain', 'type': 'fp32_t'}, "Trajectory gain over the vehicle longitudinal direction.")
    '''Trajectory gain over the vehicle longitudinal direction. Type: fp32_t'''
    lat_gain = _base.mutable_attr({'name': 'Lateral Gain', 'type': 'fp32_t'}, "Trajectory gain over the vehicle lateral direction.")
    '''Trajectory gain over the vehicle lateral direction. Type: fp32_t'''
    bond_thick = _base.mutable_attr({'name': 'Boundary Layer Thickness', 'type': 'fp32_t'}, "Control sliding surface boundary layer thickness.")
    '''Control sliding surface boundary layer thickness. Type: fp32_t'''
    lead_gain = _base.mutable_attr({'name': 'Leader Gain', 'type': 'fp32_t'}, "Formation shape gain (absolute vehicle position tracking). Leader control importance gain (relative to the sum of every other formation vehicle).")
    '''Formation shape gain (absolute vehicle position tracking). Leader control importance gain (relative to the sum of every other formation vehicle). Type: fp32_t'''
    deconfl_gain = _base.mutable_attr({'name': 'Deconfliction Gain', 'type': 'fp32_t'}, "Collision avoidance and formation shape gain (position tracking relative to the other formation vehicles). Individual vehicle importance gain (relative to the leader), when the relative position or the velocity state indicate higher probability of collision.")
    '''Collision avoidance and formation shape gain (position tracking relative to the other formation vehicles). Individual vehicle importance gain (relative to the leader), when the relative position or the velocity state indicate higher probability of collision. Type: fp32_t'''
    accel_switch_gain = _base.mutable_attr({'name': 'Acceleration Switch Gain', 'type': 'fp32_t'}, "Switch gain to compensate the worst case of the wind flow acceleration.")
    '''Switch gain to compensate the worst case of the wind flow acceleration. Type: fp32_t'''
    safe_dist = _base.mutable_attr({'name': 'Safety Distance', 'type': 'fp32_t'}, "Inter-vehicle safety distance.")
    '''Inter-vehicle safety distance. Type: fp32_t'''
    deconflict_offset = _base.mutable_attr({'name': 'Deconfliction Offset', 'type': 'fp32_t'}, "Distance offset which defines the buffer area beyond the safety distace.")
    '''Distance offset which defines the buffer area beyond the safety distace. Type: fp32_t'''
    accel_safe_margin = _base.mutable_attr({'name': 'Acceleration Safety Margin', 'type': 'fp32_t'}, "Safety margin to compensate for possible shortfalls from the predicted maximum acceleration that a vehicle can generate.")
    '''Safety margin to compensate for possible shortfalls from the predicted maximum acceleration that a vehicle can generate. Type: fp32_t'''
    accel_lim_x = _base.mutable_attr({'name': 'Maximum Longitudinal Acceleration', 'type': 'fp32_t'}, "Maximum predicted longitudinal acceleration a vehicle can generate.")
    '''Maximum predicted longitudinal acceleration a vehicle can generate. Type: fp32_t'''

    def __init__(self, Action = None, lon_gain = None, lat_gain = None, bond_thick = None, lead_gain = None, deconfl_gain = None, accel_switch_gain = None, safe_dist = None, deconflict_offset = None, accel_safe_margin = None, accel_lim_x = None):
        '''Class constructor
        
        Maximum predicted longitudinal acceleration a vehicle can generate.

       This message class contains the following fields and their respective types:
    Action : uint8_t, unit: Enumerated (Local)

            lon_gain : fp32_t, unit: NOT FOUND

            lat_gain : fp32_t, unit: NOT FOUND

            bond_thick : fp32_t, unit: NOT FOUND

            lead_gain : fp32_t, unit: NOT FOUND

            deconfl_gain : fp32_t, unit: NOT FOUND

            accel_switch_gain : fp32_t, unit: NOT FOUND

            safe_dist : fp32_t, unit: NOT FOUND

            deconflict_offset : fp32_t, unit: NOT FOUND

            accel_safe_margin : fp32_t, unit: NOT FOUND

            accel_lim_x : fp32_t, unit: NOT FOUND'''
        self._Action = Action
        self._lon_gain = lon_gain
        self._lat_gain = lat_gain
        self._bond_thick = bond_thick
        self._lead_gain = lead_gain
        self._deconfl_gain = deconfl_gain
        self._accel_switch_gain = accel_switch_gain
        self._safe_dist = safe_dist
        self._deconflict_offset = deconflict_offset
        self._accel_safe_margin = accel_safe_margin
        self._accel_lim_x = accel_lim_x


class FormationEvaluation(_base.base_message):
    '''Formation controller paramenters during the evaluation period.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            err_mean : fp32_t, unit: NOT FOUND

            dist_min_abs : fp32_t, unit: NOT FOUND

            dist_min_mean : fp32_t, unit: NOT FOUND

            roll_rate_mean : fp32_t, unit: NOT FOUND

            time : fp32_t, unit: NOT FOUND

            ControlParams : message, unit: NOT FOUND'''

    class TYPE(_enum.IntEnum):
        '''Full name: Type
        Prefix: FC'''
    
        REQUEST = 0
        '''Name: Request'''
    
        REPORT = 1
        '''Name: Report'''
    
    
    class OP(_enum.IntEnum):
        '''Full name: Operation
        Prefix: OP'''
    
        START = 0
        '''Name: Start'''
    
        STOP = 1
        '''Name: Stop'''
    
        READY = 2
        '''Name: Ready'''
    
        EXECUTING = 3
        '''Name: Executing'''
    
        FAILURE = 4
        '''Name: Failure'''
    
    
    __slots__ = ['_Attributes', '_header', '_footer', '_type', '_op', '_err_mean', '_dist_min_abs', '_dist_min_mean', '_roll_rate_mean', '_time', '_ControlParams']
    Attributes = _base.MessageAttributes(source = None, usedby = None, category = "Maneuvering", id = 823, stable = None, description = "Formation control performance evaluation variables.", fields = ('type', 'op', 'err_mean', 'dist_min_abs', 'dist_min_mean', 'roll_rate_mean', 'time', 'ControlParams',), abbrev = "FormationEvaluation", name = "Formation Evaluation Data", flags = None)

    type = _base.mutable_attr({'name': 'Type', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'FC'}, "Indicates if the message is a request, or a reply to a previous request. Enumerated (Local).")
    '''Indicates if the message is a request, or a reply to a previous request. Enumerated (Local). Type: uint8_t'''
    op = _base.mutable_attr({'name': 'Operation', 'type': 'uint8_t', 'unit': 'Enumerated', 'prefix': 'OP'}, "Operation to perform. Enumerated (Local).")
    '''Operation to perform. Enumerated (Local). Type: uint8_t'''
    err_mean = _base.mutable_attr({'name': 'Mean Position Error', 'type': 'fp32_t'}, "Mean position error relative to the formation reference.")
    '''Mean position error relative to the formation reference. Type: fp32_t'''
    dist_min_abs = _base.mutable_attr({'name': 'Absolute Minimum Distance', 'type': 'fp32_t'}, "Overall minimum distance to any other vehicle in the formation.")
    '''Overall minimum distance to any other vehicle in the formation. Type: fp32_t'''
    dist_min_mean = _base.mutable_attr({'name': 'Mean Minimum Distance', 'type': 'fp32_t'}, "Mean minimum distance to any other vehicle in the formation.")
    '''Mean minimum distance to any other vehicle in the formation. Type: fp32_t'''
    roll_rate_mean = _base.mutable_attr({'name': 'Mean Roll Rate', 'type': 'fp32_t'}, "Mean minimum distance to any other vehicle in the formation.")
    '''Mean minimum distance to any other vehicle in the formation. Type: fp32_t'''
    time = _base.mutable_attr({'name': 'Evaluation Time', 'type': 'fp32_t'}, "Period over which the evaluation data is averaged.")
    '''Period over which the evaluation data is averaged. Type: fp32_t'''
    ControlParams = _base.mutable_attr({'name': 'Formation Control Parameters', 'type': 'message', 'message-type': 'FormationControlParams'}, "Formation controller paramenters during the evaluation period.")
    '''Formation controller paramenters during the evaluation period. Type: message'''

    def __init__(self, type = None, op = None, err_mean = None, dist_min_abs = None, dist_min_mean = None, roll_rate_mean = None, time = None, ControlParams = None):
        '''Class constructor
        
        Formation controller paramenters during the evaluation period.

       This message class contains the following fields and their respective types:
    type : uint8_t, unit: Enumerated (Local)

            op : uint8_t, unit: Enumerated (Local)

            err_mean : fp32_t, unit: NOT FOUND

            dist_min_abs : fp32_t, unit: NOT FOUND

            dist_min_mean : fp32_t, unit: NOT FOUND

            roll_rate_mean : fp32_t, unit: NOT FOUND

            time : fp32_t, unit: NOT FOUND

            ControlParams : message, unit: NOT FOUND'''
        self._type = type
        self._op = op
        self._err_mean = err_mean
        self._dist_min_abs = dist_min_abs
        self._dist_min_mean = dist_min_mean
        self._roll_rate_mean = roll_rate_mean
        self._time = time
        self._ControlParams = ControlParams

